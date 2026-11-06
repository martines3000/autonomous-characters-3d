// Included inside vehicle::tests to share its existing real-device GPU helpers.
mod schooling_metrics {
    use super::{gpu_device, gpu_step};
    use crate::{
        formation::FormationFrame,
        gpu_flock::{gpu_params, BoidData, GpuFlock},
        predator::PredatorFrame,
        presentation::Presentation,
        scene::Appearance,
        schooling::SchoolMotion,
        swarm::{GroupRole, SwarmScene},
        vehicle,
        visitor::{self, PopulationTier, Visitor, VisitorScene},
        GlobalState, SimulationMode,
    };
    use bevy::{prelude::*, render::render_resource::PollType};
    use std::{collections::BTreeMap, time::Duration};

    const HZ: usize = 60;
    const SETTLE_SECONDS: usize = 10;
    const SAMPLE_SECONDS: usize = 30;
    const READBACK_EVERY: usize = HZ / 2;
    const NEIGHBOR_QUERY_BUDGET: usize = 128;
    const ALL_PREY: u32 = u32::MAX;

    struct SceneInput {
        state: GlobalState,
        motion: Option<SchoolMotion>,
        scene: SwarmScene,
        formation: FormationFrame,
        predators: PredatorFrame,
    }

    fn configured_scene(scene: VisitorScene, tier: PopulationTier) -> SceneInput {
        // The real visitor boundary supplies all physical settings. No synthetic tuning or
        // reduced population is substituted to make this probe cheaper or look better.
        let mut world = World::new();
        world.insert_resource(GlobalState::default());
        world.insert_resource(Appearance::default());
        world.insert_resource(Presentation::default());
        world.insert_resource(SwarmScene::default());
        world.insert_resource(Visitor::default());
        visitor::apply(&mut world, scene, tier);
        let state = world.resource::<GlobalState>().clone();
        assert_eq!(state.mode, SimulationMode::Gpu);
        assert_eq!(state.vehicle_count, scene.population(tier));
        SceneInput {
            state,
            // An absent controller also permits measuring the pre-extension Shark baseline.
            // Presence is printed explicitly; it does not imply that the controller is active.
            motion: world.get_resource::<SchoolMotion>().cloned(),
            scene: world.resource::<SwarmScene>().clone(),
            formation: *world.resource::<FormationFrame>(),
            predators: *world.resource::<PredatorFrame>(),
        }
    }

    #[derive(Clone, Copy)]
    struct NeighborSample {
        group: u32,
        same_group: bool,
        distance: f32,
    }

    fn effective_group(state: &GlobalState, boid: &BoidData) -> u32 {
        if state.groups.count == 0 {
            0
        } else {
            state.groups.mapped_group(boid.metadata.y)
        }
    }

    fn nearest_neighbors(
        all: &[&BoidData],
        groups: &BTreeMap<u32, Vec<&BoidData>>,
        state: &GlobalState,
    ) -> Vec<NeighborSample> {
        let per_group = NEIGHBOR_QUERY_BUDGET / groups.len().max(1);
        let mut samples = Vec::with_capacity(NEIGHBOR_QUERY_BUDGET);
        for (&group, members) in groups {
            let queries = members.len().min(per_group);
            for query in 0..queries {
                // Each group is sorted by stable spawn ID, so spatial sorting cannot bias
                // query selection or change which fish are sampled on the next readback.
                let me = members[query * members.len() / queries];
                let mut nearest = None;
                let mut best = f32::INFINITY;
                for &other in all {
                    if me.metadata.x == other.metadata.x {
                        continue;
                    }
                    let distance = me[0].truncate().distance_squared(other[0].truncate());
                    // all is sorted by spawn ID: exact distance ties choose the smaller ID.
                    if distance < best {
                        best = distance;
                        nearest = Some(other);
                    }
                }
                let nearest = nearest.expect("a school must contain at least two fish");
                samples.push(NeighborSample {
                    group,
                    same_group: effective_group(state, nearest) == group,
                    distance: best.sqrt(),
                });
            }
        }
        assert!(samples.len() <= NEIGHBOR_QUERY_BUDGET);
        samples
    }

    #[derive(Default)]
    struct Trajectory {
        centers: Vec<Vec3>,
        span: f32,
        length: f32,
    }

    impl Trajectory {
        fn push(&mut self, center: Vec3) {
            if let Some(previous) = self.centers.last() {
                self.length += previous.distance(center);
            }
            for previous in &self.centers {
                self.span = self.span.max(previous.distance(center));
            }
            self.centers.push(center);
        }
    }

    struct Metrics {
        center: Vec3,
        r80: f32,
        polarization: f32,
        neighbor_queries: usize,
        same_group_fraction: f32,
        median_neighbor_distance: f32,
        panic_fraction: f32,
        mean_speed: f32,
        max_speed: f32,
        outside_fraction: f32,
        max_overshoot: f32,
        elongation: f64,
    }

    fn median(sorted: &[f32]) -> f32 {
        assert!(!sorted.is_empty());
        let middle = sorted.len() / 2;
        if sorted.len().is_multiple_of(2) {
            sorted[middle - 1] + (sorted[middle] - sorted[middle - 1]) * 0.5
        } else {
            sorted[middle]
        }
    }

    fn elongation(positions: &[Vec3]) -> f64 {
        assert!(!positions.is_empty());
        let point = |p: Vec3| nalgebra::Vector3::new(p.x as f64, p.y as f64, p.z as f64);
        // Center and accumulate in f64 so translation and f32 summation error do not
        // manufacture a secondary axis in a narrow school.
        let center = positions
            .iter()
            .map(|&p| point(p))
            .sum::<nalgebra::Vector3<f64>>()
            / positions.len() as f64;
        let mut covariance = nalgebra::Matrix3::<f64>::zeros();
        for &position in positions {
            let offset = point(position) - center;
            covariance += offset * offset.transpose();
        }
        covariance /= positions.len() as f64;
        let values = covariance.symmetric_eigenvalues();
        assert!(values.iter().all(|value| value.is_finite()));
        let mut values = [values.x.max(0.0), values.y.max(0.0), values.z.max(0.0)];
        values.sort_unstable_by(f64::total_cmp);
        // The largest/second-largest ratio distinguishes a prolate school from a
        // flattened disc. Floor both axes for a finite degenerate-cloud result (1).
        let floor = values[2].max(1.0) * 1e-12;
        (values[2].max(floor) / values[1].max(floor)).sqrt()
    }

    fn metrics(
        boids: &[&BoidData],
        neighbors: &[NeighborSample],
        group: u32,
        bounds: Vec3,
    ) -> Metrics {
        assert!(!boids.is_empty());
        let count = boids.len() as f32;
        let positions: Vec<_> = boids.iter().map(|b| b[0].truncate()).collect();
        let center = positions.iter().copied().sum::<Vec3>() / count;
        let velocity = boids.iter().map(|b| b[1].truncate()).sum::<Vec3>();
        let mut radii: Vec<_> = positions
            .iter()
            .map(|position| position.distance(center))
            .collect();
        radii.sort_unstable_by(f32::total_cmp);
        let speeds: Vec<_> = boids.iter().map(|b| b[1].truncate().length()).collect();
        let total_speed: f32 = speeds.iter().sum();
        let queries: Vec<_> = neighbors
            .iter()
            .filter(|sample| group == ALL_PREY || sample.group == group)
            .collect();
        assert!(!queries.is_empty());
        let mut distances: Vec<_> = queries.iter().map(|sample| sample.distance).collect();
        distances.sort_unstable_by(f32::total_cmp);
        let overshoots: Vec<_> = boids
            .iter()
            .map(|b| {
                (b[0].truncate().abs() - bounds)
                    .max(Vec3::ZERO)
                    .max_element()
            })
            .collect();
        Metrics {
            center,
            r80: radii[(radii.len() - 1) * 4 / 5],
            polarization: if total_speed > 0.0 {
                (velocity.length() / total_speed).clamp(0.0, 1.0)
            } else {
                0.0
            },
            neighbor_queries: queries.len(),
            same_group_fraction: queries.iter().filter(|sample| sample.same_group).count() as f32
                / queries.len() as f32,
            median_neighbor_distance: median(&distances),
            panic_fraction: boids
                .iter()
                .filter(|boid| f32::from_bits(boid.metadata.w) > 0.0)
                .count() as f32
                / count,
            mean_speed: total_speed / count,
            max_speed: speeds.into_iter().fold(0.0, f32::max),
            outside_fraction: overshoots.iter().filter(|&&value| value > 0.0).count() as f32
                / count,
            max_overshoot: overshoots.into_iter().fold(0.0, f32::max),
            elongation: elongation(&positions),
        }
    }

    fn validate(boids: &[BoidData], state: &GlobalState) {
        assert_eq!(boids.len(), state.vehicle_count);
        let mut seen = vec![false; state.vehicle_count];
        for boid in boids {
            assert!(boid.iter().all(|value| value.is_finite()), "{boid:?}");
            let id = boid.metadata.x as usize;
            assert!(
                id < seen.len() && !seen[id],
                "invalid or duplicate spawn ID {id}"
            );
            seen[id] = true;
            if state.groups.count != 0 {
                assert!(boid.metadata.y < state.groups.count);
                assert_eq!(
                    state.groups.table[boid.metadata.y as usize].role,
                    GroupRole::Prey
                );
            }
            let panic = f32::from_bits(boid.metadata.w);
            assert!(panic.is_finite() && (0.0..=crate::hunter::PANIC_TIME).contains(&panic));
            let ceiling = state.vehicle_max_speed
                * state.groups.speed_scale(boid.metadata.y)
                * if panic > 0.0 { 1.35 } else { 1.0 };
            assert!(
                boid[1].truncate().length() <= ceiling + ceiling.max(1.0) * 0.001,
                "speed exceeds the kernel limit: {boid:?}"
            );
        }
    }

    fn report_snapshot(
        phase: &str,
        scene: VisitorScene,
        tier: PopulationTier,
        seconds: f64,
        boids: &[BoidData],
        state: &GlobalState,
        tracks: &mut BTreeMap<u32, Trajectory>,
    ) {
        validate(boids, state);
        let mut all: Vec<_> = boids.iter().collect();
        all.sort_unstable_by_key(|boid| boid.metadata.x);
        let mut groups: BTreeMap<u32, Vec<&BoidData>> = BTreeMap::new();
        for &boid in &all {
            groups
                .entry(effective_group(state, boid))
                .or_default()
                .push(boid);
        }
        let neighbors = nearest_neighbors(&all, &groups, state);
        groups.insert(ALL_PREY, all);
        for (group, members) in groups {
            let sample = metrics(
                &members,
                &neighbors,
                group,
                vehicle::walls(state.vehicle_count),
            );
            let track = tracks.entry(group).or_default();
            // The spawn row is diagnostic; travel starts after the settling period.
            if phase == "sample" {
                track.push(sample.center);
            }
            let label = if group == ALL_PREY {
                "all".into()
            } else {
                group.to_string()
            };
            println!(
                "SCHOOL_METRICS_CSV,{phase},{},{},{seconds:.6},{label},{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
                scene.name(), tier.name(), members.len(),
                sample.center.x, sample.center.y, sample.center.z,
                track.span, track.length, sample.r80, sample.polarization,
                sample.neighbor_queries, sample.same_group_fraction,
                sample.median_neighbor_distance, sample.panic_fraction,
                sample.mean_speed, sample.max_speed,
                sample.outside_fraction, sample.max_overshoot, sample.elongation,
            );
        }
    }

    #[test]
    fn nearest_neighbor_metric_searches_foreign_groups_and_bounds_query_count() {
        let mut state = GlobalState::default();
        state.groups.count = 2;
        let boids: Vec<_> = [(0, 0, 0.0), (1, 1, 1.0), (2, 0, 100.0), (3, 1, 101.0)]
            .into_iter()
            .map(|(id, group, x)| BoidData {
                state: [Vec4::new(x, 0.0, 0.0, 0.0), Vec4::ZERO, Vec4::W, Vec4::ZERO],
                metadata: UVec4::new(id, group, 0, 0),
            })
            .collect();
        let all: Vec<_> = boids.iter().collect();
        let groups = BTreeMap::from([
            (0, vec![&boids[0], &boids[2]]),
            (1, vec![&boids[1], &boids[3]]),
        ]);
        let nearest = nearest_neighbors(&all, &groups, &state);
        assert_eq!(nearest.len(), 4);
        assert!(nearest
            .iter()
            .all(|sample| !sample.same_group && sample.distance == 1.0));

        state.groups.count = 3;
        let boids: Vec<_> = (0..1024)
            .map(|id| BoidData {
                state: [
                    Vec4::new(id as f32, 0.0, 0.0, 0.0),
                    Vec4::ZERO,
                    Vec4::W,
                    Vec4::ZERO,
                ],
                metadata: UVec4::new(id, id % 3, 0, 0),
            })
            .collect();
        let all: Vec<_> = boids.iter().collect();
        let mut groups = BTreeMap::<u32, Vec<&BoidData>>::new();
        for &boid in &all {
            groups.entry(boid.metadata.y).or_default().push(boid);
        }
        let nearest = nearest_neighbors(&all, &groups, &state);
        assert!(nearest.len() <= NEIGHBOR_QUERY_BUDGET);
        for group in 0..3 {
            assert!(nearest.iter().any(|sample| sample.group == group));
        }
    }

    #[test]
    fn centroid_travel_metric_recognizes_a_closed_route() {
        let mut track = Trajectory::default();
        for center in [Vec3::ZERO, Vec3::X, Vec3::X + Vec3::Y, Vec3::Y, Vec3::ZERO] {
            track.push(center);
        }
        assert_eq!(track.length, 4.0);
        assert!((track.span - 2.0_f32.sqrt()).abs() < 1e-6);
        assert_eq!(track.centers.first(), track.centers.last());
    }

    #[test]
    fn nearest_distance_median_handles_even_samples() {
        assert_eq!(median(&[1.0, 3.0]), 2.0);
        assert_eq!(median(&[1.0, 3.0, 5.0]), 3.0);
    }

    #[test]
    fn elongation_distinguishes_prolate_schools_from_spheres_and_discs() {
        let sphere = [
            Vec3::X,
            Vec3::NEG_X,
            Vec3::Y,
            Vec3::NEG_Y,
            Vec3::Z,
            Vec3::NEG_Z,
        ];
        assert!((elongation(&sphere) - 1.0).abs() < 1e-8);
        let disc = sphere.map(|point| point * Vec3::new(1.0, 1.0, 0.01));
        assert!((elongation(&disc) - 1.0).abs() < 1e-8);
        let prolate = sphere.map(|point| point * Vec3::new(5.0, 1.0, 1.0));
        assert!((elongation(&prolate) - 5.0).abs() < 1e-8);
        let rotation = Quat::from_rotation_y(0.71) * Quat::from_rotation_z(0.42);
        let moved = prolate.map(|point| rotation * point + Vec3::new(100.0, -20.0, 7.0));
        assert!((elongation(&moved) - 5.0).abs() < 1e-4);
        assert_eq!(elongation(&[Vec3::ONE; 4]), 1.0);
        assert!(elongation(&[Vec3::NEG_X, Vec3::X]).is_finite());
    }

    #[test]
    #[ignore = "real GPU school probe; run release with --nocapture --test-threads=1"]
    fn gpu_schooling_scene_metrics() {
        let step = Duration::from_secs_f64(1.0 / HZ as f64);
        let selected_scene =
            std::env::var("SCHOOL_PROBE_SCENE")
                .ok()
                .map(|value| match value.as_str() {
                    "reef" => VisitorScene::ReefDrift,
                    "shark" => VisitorScene::SharkHunt,
                    _ => panic!("SCHOOL_PROBE_SCENE must be reef or shark"),
                });
        let selected_tier = std::env::var("SCHOOL_PROBE_TIER").ok().map(|value| {
            PopulationTier::parse(&value).expect("SCHOOL_PROBE_TIER must be low, medium or high")
        });
        println!(
            "SCHOOL_METRICS_META,os={},arch={},release={},spawn=default_gpu_pcg_frame_0,bitwise_replay_claim=false,step_seconds={:.9},settle_seconds={SETTLE_SECONDS},sample_seconds={SAMPLE_SECONDS},readback_hz=2,max_exact_neighbor_queries={NEIGHBOR_QUERY_BUDGET},polarization=speed_weighted_velocity_sum_over_speed_sum,elongation=sqrt_largest_over_second_position_covariance_eigenvalue,bounds=sampled_only,performance_timing=false",
            std::env::consts::OS, std::env::consts::ARCH,
            !cfg!(debug_assertions), step.as_secs_f64(),
        );
        println!(
            "SCHOOL_METRICS_CSV,phase,scene,tier,seconds,group,count,centroid_x,centroid_y,centroid_z,centroid_span,centroid_path_length,r80,polarization,neighbor_queries,same_group_nearest_fraction,median_nearest_distance,panic_fraction,mean_speed,max_speed,outside_fraction,max_overshoot,elongation"
        );
        let (device, queue) = gpu_device();
        for scene in [VisitorScene::ReefDrift, VisitorScene::SharkHunt] {
            if selected_scene.is_some_and(|selected| selected != scene) {
                continue;
            }
            for tier in PopulationTier::ALL {
                if selected_tier.is_some_and(|selected| selected != tier) {
                    continue;
                }
                let SceneInput {
                    mut state,
                    mut motion,
                    scene: swarm,
                    formation,
                    predators,
                } = configured_scene(scene, tier);
                assert!(swarm.obstacles.is_empty() && !formation.settings.enabled());
                assert!(!predators.active());
                println!(
                    "SCHOOL_METRICS_CASE,scene={},tier={},population={},controller_resource_present={},spawn_size={:.6},gpu_max_neighbors={}",
                    scene.name(), tier.name(), state.vehicle_count,
                    motion.is_some(), vehicle::spawn_size(state.vehicle_count),
                    state.gpu_max_neighbors,
                );
                let mut flock = GpuFlock::new(&device);
                flock.set_scene(&swarm);
                flock.set_predators(&predators);
                flock.set_group_scales(&state.groups);
                let params = |state: &GlobalState, dt| {
                    gpu_params(state, Vec3::ZERO, false, dt)
                        .with_scene(&swarm)
                        .with_formation(&formation, state.gpu_max_neighbors)
                        .with_predators(&predators)
                };
                gpu_step(&mut flock, &device, &queue, &params(&state, 0.0));
                let mut tracks = BTreeMap::new();
                report_snapshot(
                    "spawn",
                    scene,
                    tier,
                    0.0,
                    &flock.read_boids(&device, &queue),
                    &state,
                    &mut tracks,
                );
                for index in 1..=(SETTLE_SECONDS + SAMPLE_SECONDS) * HZ {
                    if let Some(motion) = &mut motion {
                        motion.advance(&mut state, step);
                    }
                    for group in &state.groups.table[..state.groups.count as usize] {
                        assert!(group.target.is_none_or(|target| target.is_finite()));
                    }
                    gpu_step(
                        &mut flock,
                        &device,
                        &queue,
                        &params(&state, step.as_secs_f32()),
                    );
                    if index % READBACK_EVERY == 0 {
                        if index >= SETTLE_SECONDS * HZ {
                            report_snapshot(
                                "sample",
                                scene,
                                tier,
                                index as f64 * step.as_secs_f64(),
                                &flock.read_boids(&device, &queue),
                                &state,
                                &mut tracks,
                            );
                        } else {
                            // Bound in-flight work during warmup too. Readback waits are not
                            // included in any rendered/GPU performance claim.
                            device.poll(PollType::wait_indefinitely()).unwrap();
                        }
                    }
                }
            }
        }
    }
}
