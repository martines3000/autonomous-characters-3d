//! Named flocking behaviors: one click sets how a school or flock moves, and its hero model.
//! Unlike showcases, a species leaves the environment, camera, Director, groups and obstacles alone.
//! Attack examples configure hunter groups; selecting another species clears those groups.

use bevy::prelude::*;

use crate::{
    formation::FormationFrame,
    predator::{PredatorFrame, PredatorPath},
    scene::Appearance,
    swarm::SwarmScene,
    vehicle_appearance::VehicleShape,
    GlobalState,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Species {
    BaitBall,
    TunaStream,
    ReefDrift,
    Murmuration,
    SwallowHunt,
    RookRoost,
    SharkAttack,
    FalconStrike,
    TunaRaid,
}

/// Everything a species controls. Unlisted steering settings return to their defaults.
pub struct Behavior {
    pub count: usize,
    pub mass: f32,
    pub max_speed: f32,
    pub wander_speed: f32,
    /// Separation, alignment and cohesion (distance, factor).
    pub separation: (f32, f32),
    pub alignment: (f32, f32),
    pub cohesion: (f32, f32),
    pub wander_factor: f32,
    pub wall_avoid_factor: f32,
    /// Bank, flap/swim amplitude and frequency.
    pub motion: (f32, f32, f32),
    pub predators: usize,
    pub predator_strength: f32,
    pub shape: VehicleShape,
    pub visual_size: f32,
    pub model: &'static str,
}

impl Species {
    pub const FISH: [Self; 3] = [Self::BaitBall, Self::TunaStream, Self::ReefDrift];
    pub const BIRDS: [Self; 3] = [Self::Murmuration, Self::SwallowHunt, Self::RookRoost];
    pub const HUNTS: [Self; 3] = [Self::SharkAttack, Self::FalconStrike, Self::TunaRaid];

    pub fn name(self) -> &'static str {
        match self {
            Self::BaitBall => "Sardine bait ball",
            Self::TunaStream => "Tuna stream",
            Self::ReefDrift => "Reef drift",
            Self::Murmuration => "Starling murmuration",
            Self::SwallowHunt => "Swallow hunt",
            Self::RookRoost => "Rook roost",
            Self::SharkAttack => "Shark attack",
            Self::FalconStrike => "Falcon strike",
            Self::TunaRaid => "Tuna raid",
        }
    }

    pub fn description(self) -> &'static str {
        match self {
            Self::BaitBall => "Dense, fast ball; three hunters punch holes through it.",
            Self::TunaStream => "Few heavy, fast fish in long aligned ribbons.",
            Self::ReefDrift => "Slow, loose fish idling and wandering.",
            Self::Murmuration => "Huge cloud of starlings waving around two falcons.",
            Self::SwallowHunt => "Small, very fast, agile birds darting loosely.",
            Self::RookRoost => "Large, heavy birds circling slowly in a loose crowd.",
            Self::SharkAttack => {
                "Three larger sharks pursue a sardine school; prey scatter and regroup."
            }
            Self::FalconStrike => "Two falcons slash through a starling murmuration, then recover.",
            Self::TunaRaid => {
                "Twelve fast tuna slash through a sardine bait ball; prey scatter and regroup."
            }
        }
    }

    pub fn parse(name: &str) -> Result<Self, String> {
        match name.to_ascii_lowercase().as_str() {
            "bait-ball" | "sardine" | "sardines" => Ok(Self::BaitBall),
            "tuna" | "tuna-stream" => Ok(Self::TunaStream),
            "reef" | "reef-drift" => Ok(Self::ReefDrift),
            "murmuration" | "starling" | "starlings" => Ok(Self::Murmuration),
            "swallow" | "swallow-hunt" => Ok(Self::SwallowHunt),
            "rook" | "rooks" | "rook-roost" => Ok(Self::RookRoost),
            "shark-attack" => Ok(Self::SharkAttack),
            "falcon-strike" => Ok(Self::FalconStrike),
            "tuna-raid" => Ok(Self::TunaRaid),
            _ => Err(format!(
                "invalid species '{name}'; expected sardine, tuna, reef, murmuration, swallow, rook, shark-attack, falcon-strike or tuna-raid"
            )),
        }
    }

    pub fn behavior(self) -> Behavior {
        const FISH: &str = "assets/models/fish.glb";
        const STARLING: &str = "assets/models/starling.glb";
        match self {
            Self::SharkAttack | Self::TunaRaid => Behavior {
                predators: 0,
                predator_strength: 0.0,
                ..Self::BaitBall.behavior()
            },
            Self::FalconStrike => Behavior {
                predators: 0,
                predator_strength: 0.0,
                ..Self::Murmuration.behavior()
            },
            Self::BaitBall => Behavior {
                count: 30_000,
                mass: 25.0,
                max_speed: 110.0,
                wander_speed: 30.0,
                separation: (2.5, 2.0),
                alignment: (25.0, 2.0),
                cohesion: (40.0, 6.0),
                wander_factor: 0.3,
                wall_avoid_factor: 1.5,
                motion: (0.8, 1.0, 1.5),
                predators: 3,
                predator_strength: 6.0,
                shape: VehicleShape::Fish,
                visual_size: 1.2,
                model: FISH,
            },
            Self::TunaStream => Behavior {
                count: 8_000,
                mass: 90.0,
                max_speed: 160.0,
                wander_speed: 120.0,
                separation: (6.0, 1.5),
                alignment: (80.0, 6.0),
                cohesion: (30.0, 1.0),
                wander_factor: 0.2,
                wall_avoid_factor: 1.5,
                motion: (0.6, 0.6, 0.8),
                predators: 0,
                predator_strength: 0.0,
                shape: VehicleShape::Fish,
                visual_size: 2.4,
                model: FISH,
            },
            Self::ReefDrift => Behavior {
                count: 3_000,
                mass: 40.0,
                max_speed: 25.0,
                wander_speed: 15.0,
                separation: (12.0, 2.0),
                alignment: (12.0, 0.3),
                cohesion: (15.0, 0.15),
                wander_factor: 3.0,
                wall_avoid_factor: 1.5,
                motion: (0.5, 0.5, 0.7),
                predators: 0,
                predator_strength: 0.0,
                shape: VehicleShape::Fish,
                visual_size: 2.0,
                model: FISH,
            },
            Self::Murmuration => Behavior {
                count: 40_000,
                mass: 50.0,
                max_speed: 90.0,
                wander_speed: 60.0,
                separation: (6.0, 1.5),
                alignment: (100.0, 3.0),
                cohesion: (60.0, 0.15),
                wander_factor: 0.1,
                wall_avoid_factor: 3.0,
                motion: (0.9, 0.6, 1.0),
                predators: 2,
                predator_strength: 5.0,
                shape: VehicleShape::Bird,
                visual_size: 1.6,
                model: STARLING,
            },
            Self::SwallowHunt => Behavior {
                count: 1_500,
                mass: 10.0,
                max_speed: 150.0,
                wander_speed: 100.0,
                separation: (14.0, 2.5),
                alignment: (12.0, 0.6),
                cohesion: (16.0, 0.2),
                wander_factor: 4.0,
                wall_avoid_factor: 1.5,
                motion: (1.0, 0.8, 1.3),
                predators: 0,
                predator_strength: 0.0,
                shape: VehicleShape::Bird,
                visual_size: 2.0,
                model: "assets/models/swallow.glb",
            },
            Self::RookRoost => Behavior {
                count: 5_000,
                mass: 120.0,
                max_speed: 45.0,
                wander_speed: 30.0,
                separation: (12.0, 2.0),
                alignment: (50.0, 1.0),
                cohesion: (60.0, 0.2),
                wander_factor: 0.6,
                wall_avoid_factor: 1.5,
                motion: (0.5, 0.35, 0.6),
                predators: 0,
                predator_strength: 0.0,
                shape: VehicleShape::Bird,
                visual_size: 2.6,
                model: "assets/models/rook.glb",
            },
        }
    }
}

/// Menu buttons and scenarios enter here.
pub(crate) fn apply(world: &mut World, species: Species) {
    let b = species.behavior();
    let old = world.resource::<GlobalState>();
    let mut state = GlobalState {
        attractor: old.attractor,
        groups: old.groups,
        mode: old.mode,
        gpu_max_neighbors: old.gpu_max_neighbors,
        octree_size: old.octree_size,
        ..default()
    };
    if state
        .groups
        .table
        .iter()
        .any(|group| group.role == crate::swarm::GroupRole::Hunter)
    {
        state.groups = default();
    }
    if Species::HUNTS.contains(&species) {
        state.groups = crate::swarm::Groups {
            count: 2,
            ..default()
        };
        if matches!(species, Species::SharkAttack | Species::TunaRaid) {
            state.groups.table[0].target = Some(Vec3::ZERO);
        }
        let hunter = &mut state.groups.table[1];
        hunter.role = crate::swarm::GroupRole::Hunter;
        match species {
            Species::SharkAttack => {
                hunter.hunter_count = 3;
                hunter.size_scale = 5.0;
                hunter.speed_scale = 1.3;
                hunter.hunter_model = crate::hunter::HunterModel::Shark;
            }
            Species::FalconStrike => {
                hunter.hunter_count = 2;
                hunter.size_scale = 3.0;
                hunter.speed_scale = 1.5;
                hunter.hunter_model = crate::hunter::HunterModel::Falcon;
            }
            Species::TunaRaid => {
                hunter.hunter_count = 12;
                hunter.size_scale = 2.5;
                hunter.speed_scale = 1.6;
                hunter.hunter_model = crate::hunter::HunterModel::Tuna;
            }
            _ => unreachable!(),
        }
    }
    state.vehicle_count = b.count.min(state.mode.max_vehicles());
    state.vehicle_mass = b.mass;
    state.vehicle_max_speed = b.max_speed;
    state.vehicle_wander_speed = b.wander_speed;
    (
        state.vehicle_separation_distance,
        state.vehicle_separation_factor,
    ) = b.separation;
    (
        state.vehicle_alignment_distance,
        state.vehicle_alignment_factor,
    ) = b.alignment;
    (
        state.vehicle_cohesion_distance,
        state.vehicle_cohesion_factor,
    ) = b.cohesion;
    state.vehicle_wander_factor = b.wander_factor;
    state.vehicle_wall_avoid_factor = b.wall_avoid_factor;
    (state.motion.bank, state.motion.flap, state.motion.frequency) = b.motion;
    world.insert_resource(state);

    let mut swarm = world.resource_mut::<SwarmScene>();
    swarm.formation = default();
    swarm.predators = default();
    for _ in 0..b.predators {
        swarm.predators.add(Vec3::ZERO, 70.0);
    }
    if !Species::HUNTS.contains(&species) {
        swarm.predators.path = PredatorPath::Orbit;
        swarm.predators.path_radius = 40.0;
        swarm.predators.path_speed = 30.0;
        swarm.predators.strength = b.predator_strength;
        swarm.predators.visible = false;
    }
    world.insert_resource(FormationFrame::default());
    world.insert_resource(PredatorFrame::default());
    world.insert_resource(crate::hunter::Hunters::default());

    let mut appearance = world.resource_mut::<Appearance>();
    appearance.vehicles.shape = b.shape;
    appearance.vehicles.visual_size = b.visual_size;
    // The hero model replaces the built-in shape once its asynchronous load lands.
    if let Some(mut models) = world.get_resource_mut::<crate::vehicle_model::VehicleModels>() {
        models.cancel();
        models.settings = default();
        models.request(b.model.into());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{environment::Environment, presentation::Presentation, SimulationMode};

    fn world() -> World {
        let mut world = World::new();
        world.insert_resource(GlobalState::default());
        world.insert_resource(Appearance::default());
        world.insert_resource(Presentation::default());
        world.insert_resource(SwarmScene::default());
        world
    }

    #[test]
    fn names_parse_and_every_species_is_listed_once() {
        let all: Vec<_> = Species::FISH.into_iter().chain(Species::BIRDS).collect();
        for (name, species) in [
            ("sardine", Species::BaitBall),
            ("tuna", Species::TunaStream),
            ("reef", Species::ReefDrift),
            ("murmuration", Species::Murmuration),
            ("swallow", Species::SwallowHunt),
            ("rook", Species::RookRoost),
        ] {
            assert_eq!(Species::parse(name), Ok(species));
        }
        assert!(Species::parse("shark").is_err());
        assert_eq!(all.len(), 6);
    }

    #[test]
    fn species_replace_behavior_but_keep_scene_and_presentation() {
        let mut world = world();
        world.resource_mut::<GlobalState>().mode = SimulationMode::Octree;
        world.resource_mut::<GlobalState>().vehicle_seek_factor = 99.0;
        world.resource_mut::<Appearance>().environment = Environment::Underwater;
        world.resource_mut::<Presentation>().orbit = true;
        world
            .resource_mut::<SwarmScene>()
            .add_obstacle(Vec3::ONE, 10.0);
        world.resource_mut::<SwarmScene>().formation.mode = crate::formation::FormationMode::Sphere;
        apply(&mut world, Species::BaitBall);
        let state = world.resource::<GlobalState>();
        assert_eq!(state.mode, SimulationMode::Octree);
        // Octree caps the count below the species' 30k.
        assert_eq!(
            state.vehicle_count,
            SimulationMode::Octree.max_vehicles().min(30_000)
        );
        assert_eq!(
            state.vehicle_seek_factor,
            GlobalState::default().vehicle_seek_factor
        );
        assert_eq!(state.vehicle_cohesion_factor, 6.0);
        let swarm = world.resource::<SwarmScene>();
        assert_eq!(swarm.obstacles.len(), 1);
        assert_eq!(swarm.formation, default());
        assert_eq!(swarm.predators.objects.len(), 3);
        assert_eq!(
            world.resource::<Appearance>().environment,
            Environment::Underwater
        );
        assert!(world.resource::<Presentation>().orbit);
        assert_eq!(
            world.resource::<Appearance>().vehicles.shape,
            VehicleShape::Fish
        );
    }

    #[test]
    fn species_are_order_independent() {
        let all: Vec<_> = Species::FISH.into_iter().chain(Species::BIRDS).collect();
        let mut expected = Vec::new();
        for &species in &all {
            let mut world = world();
            apply(&mut world, species);
            expected.push((
                world.resource::<GlobalState>().clone(),
                world.resource::<SwarmScene>().clone(),
            ));
        }
        let mut world = world();
        for &first in &all {
            for (i, &second) in all.iter().enumerate() {
                apply(&mut world, first);
                apply(&mut world, second);
                assert_eq!(*world.resource::<GlobalState>(), expected[i].0);
                assert_eq!(*world.resource::<SwarmScene>(), expected[i].1);
            }
        }
    }

    #[test]
    fn attack_examples_configure_visible_hunters_and_preserve_scene() {
        for (species, count, model, size, speed) in [
            (
                Species::SharkAttack,
                3,
                crate::hunter::HunterModel::Shark,
                5.0,
                1.3,
            ),
            (
                Species::FalconStrike,
                2,
                crate::hunter::HunterModel::Falcon,
                3.0,
                1.5,
            ),
            (
                Species::TunaRaid,
                12,
                crate::hunter::HunterModel::Tuna,
                2.5,
                1.6,
            ),
        ] {
            let mut world = world();
            world.resource_mut::<Appearance>().environment = Environment::Underwater;
            world.resource_mut::<Presentation>().orbit = true;
            apply(&mut world, species);
            let state = world.resource::<GlobalState>();
            assert_eq!(state.vehicle_count, species.behavior().count);
            assert_eq!(
                species.behavior().model,
                if species == Species::FalconStrike {
                    "assets/models/starling.glb"
                } else {
                    "assets/models/fish.glb"
                }
            );
            assert_eq!(state.groups.count, 2);
            assert_eq!(state.groups.hunter_count(), count);
            assert_eq!(state.groups.table[0].role, crate::swarm::GroupRole::Prey);
            assert_eq!(
                state.groups.table[0].target,
                matches!(species, Species::SharkAttack | Species::TunaRaid).then_some(Vec3::ZERO)
            );
            let hunter = state.groups.table[1];
            assert_eq!(hunter.role, crate::swarm::GroupRole::Hunter);
            assert_eq!(hunter.hunter_model, model);
            assert_eq!(hunter.size_scale, size);
            assert_eq!(hunter.speed_scale, speed);
            assert_eq!(world.resource::<SwarmScene>().predators, default());
            assert_eq!(
                world.resource::<Appearance>().environment,
                Environment::Underwater
            );
            assert!(world.resource::<Presentation>().orbit);
            assert!(world.contains_resource::<crate::hunter::Hunters>());
            apply(&mut world, Species::ReefDrift);
            assert_eq!(world.resource::<GlobalState>().groups, default());
        }
    }

    #[test]
    fn attack_names_parse_and_examples_are_order_independent() {
        assert_eq!(Species::parse("shark-attack"), Ok(Species::SharkAttack));
        assert_eq!(Species::parse("falcon-strike"), Ok(Species::FalconStrike));
        assert_eq!(Species::parse("tuna-raid"), Ok(Species::TunaRaid));
        assert_eq!(Species::parse("TUNA-RAID"), Ok(Species::TunaRaid));
        let all: Vec<_> = Species::FISH
            .into_iter()
            .chain(Species::BIRDS)
            .chain(Species::HUNTS)
            .collect();
        for &second in &Species::HUNTS {
            let mut expected = world();
            apply(&mut expected, second);
            for &first in &all {
                let mut actual = world();
                apply(&mut actual, first);
                apply(&mut actual, second);
                assert_eq!(
                    actual.resource::<GlobalState>(),
                    expected.resource::<GlobalState>()
                );
                assert_eq!(
                    actual.resource::<SwarmScene>(),
                    expected.resource::<SwarmScene>()
                );
            }
        }
    }
}
