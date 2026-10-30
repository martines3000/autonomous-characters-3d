// Sixteen splits resolve 1/65,536 of each root axis while bounding coincident-point growth.
const MAX_DEPTH: usize = 16;

// Entries are append-only until the whole tree is discarded.
pub struct Octree<T> {
    pub max_points: usize,
    pub root: Option<usize>,
    pub nodes: Vec<OctreeNode>,
    pub points: Vec<PointWrapper<T>>,
}

#[derive(Debug)]
pub struct PointWrapper<T> {
    pub point: Point,
    pub data: T,
}

#[derive(Debug, Clone)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

pub struct OctreeNode {
    pub points: Vec<usize>,
    pub children: Vec<usize>,
    // center, min, max
    pub bounds: (Point, Point, Point),
}

// Octree impl
impl<T> Octree<T> {
    pub fn new(max_points: usize) -> Self {
        Self {
            max_points: max_points,
            root: None,
            nodes: Vec::new(),
            points: Vec::new(),
        }
    }

    pub fn create_root(&mut self, center: Point, min: Point, max: Point) -> usize {
        let index = self.nodes.len();
        self.nodes.push(OctreeNode {
            points: vec![],
            children: vec![],
            bounds: (center, min, max),
        });

        self.root = Some(index);

        index
    }

    pub fn insert(&mut self, point: Point, data: T) -> usize {
        let index = self.points.len();
        self.points.push(PointWrapper { data, point });

        if let Some(root) = self.root {
            let mut node_index = root;
            let mut depth = 0;
            loop {
                let node = &mut self.nodes[node_index];
                if node.children.is_empty() {
                    node.points.push(index);
                    if node.points.len() > self.max_points && depth < MAX_DEPTH {
                        self.subdivide(node_index);
                    }
                    break;
                }
                node_index = self.child_for_point(node_index, &self.points[index].point);
                depth += 1;
            }
        }

        index
    }

    fn child_for_point(&self, node_index: usize, point: &Point) -> usize {
        let node = &self.nodes[node_index];
        let center = &node.bounds.0;
        // Squared center distances can round to ties across a split in thin boxes.
        let octant = usize::from(point.x >= center.x)
            | (usize::from(point.y < center.y) << 1)
            | (usize::from(point.z < center.z) << 2);
        node.children[octant]
    }

    fn subdivide(&mut self, node_index: usize) {
        let node = self.nodes.get(node_index).unwrap();
        let (center, min, max) = node.bounds.clone();
        let mut children = vec![];

        // Front top left
        children.push(self.create_node(
            Point {
                x: center.x - (center.x - min.x) / 2.0,
                y: center.y + (max.y - center.y) / 2.0,
                z: center.z + (max.z - center.z) / 2.0,
            },
            Point {
                x: min.x,
                y: center.y,
                z: center.z,
            },
            Point {
                x: center.x,
                y: max.y,
                z: max.z,
            },
        ));

        // Front top right
        children.push(self.create_node(
            Point {
                x: center.x + (max.x - center.x) / 2.0,
                y: center.y + (max.y - center.y) / 2.0,
                z: center.z + (max.z - center.z) / 2.0,
            },
            Point {
                x: center.x,
                y: center.y,
                z: center.z,
            },
            Point {
                x: max.x,
                y: max.y,
                z: max.z,
            },
        ));

        // Front bottom left
        children.push(self.create_node(
            Point {
                x: center.x - (center.x - min.x) / 2.0,
                y: center.y - (center.y - min.y) / 2.0,
                z: center.z + (max.z - center.z) / 2.0,
            },
            Point {
                x: min.x,
                y: min.y,
                z: center.z,
            },
            Point {
                x: center.x,
                y: center.y,
                z: max.z,
            },
        ));

        // Front bottom right
        children.push(self.create_node(
            Point {
                x: center.x + (max.x - center.x) / 2.0,
                y: center.y - (center.y - min.y) / 2.0,
                z: center.z + (max.z - center.z) / 2.0,
            },
            Point {
                x: center.x,
                y: min.y,
                z: center.z,
            },
            Point {
                x: max.x,
                y: center.y,
                z: max.z,
            },
        ));

        // Back top left
        children.push(self.create_node(
            Point {
                x: center.x - (center.x - min.x) / 2.0,
                y: center.y + (max.y - center.y) / 2.0,
                z: center.z - (center.z - min.z) / 2.0,
            },
            Point {
                x: min.x,
                y: center.y,
                z: min.z,
            },
            Point {
                x: center.x,
                y: max.y,
                z: center.z,
            },
        ));

        // Back top right
        children.push(self.create_node(
            Point {
                x: center.x + (max.x - center.x) / 2.0,
                y: center.y + (max.y - center.y) / 2.0,
                z: center.z - (center.z - min.z) / 2.0,
            },
            Point {
                x: center.x,
                y: center.y,
                z: min.z,
            },
            Point {
                x: max.x,
                y: max.y,
                z: center.z,
            },
        ));

        // Back bottom left
        children.push(self.create_node(
            Point {
                x: center.x - (center.x - min.x) / 2.0,
                y: center.y - (center.y - min.y) / 2.0,
                z: center.z - (center.z - min.z) / 2.0,
            },
            Point {
                x: min.x,
                y: min.y,
                z: min.z,
            },
            Point {
                x: center.x,
                y: center.y,
                z: center.z,
            },
        ));

        // Back bottom right
        children.push(self.create_node(
            Point {
                x: center.x + (max.x - center.x) / 2.0,
                y: center.y - (center.y - min.y) / 2.0,
                z: center.z - (center.z - min.z) / 2.0,
            },
            Point {
                x: center.x,
                y: min.y,
                z: min.z,
            },
            Point {
                x: max.x,
                y: center.y,
                z: center.z,
            },
        ));

        let mut_node = self.nodes.get_mut(node_index).unwrap();
        mut_node.children = children.clone();

        let points = mut_node.points.clone();
        mut_node.points = vec![];

        for point_index in points {
            let child_index = self.child_for_point(node_index, &self.points[point_index].point);
            self.nodes[child_index].points.push(point_index);
        }
    }

    fn create_node(&mut self, center: Point, min: Point, max: Point) -> usize {
        let node = OctreeNode {
            bounds: (center, min, max),
            children: vec![],
            points: vec![],
        };

        let index = self.nodes.len();
        self.nodes.push(node);
        index
    }

    /// Returns points at squared distance <= radius²; inserted points must lie in the root bounds.
    pub fn query_radius(&self, point: &Point, radius: f32) -> Vec<&T> {
        let mut neighbors = vec![];
        let Some(root) = self.root else {
            return vec![];
        };
        let radius_squared = radius * radius;
        let mut pending = vec![root];
        while let Some(node_index) = pending.pop() {
            let node = &self.nodes[node_index];
            let (_, min, max) = &node.bounds;
            let distance_to_bounds_squared = (point.x - point.x.clamp(min.x, max.x)).powi(2)
                + (point.y - point.y.clamp(min.y, max.y)).powi(2)
                + (point.z - point.z.clamp(min.z, max.z)).powi(2);
            if distance_to_bounds_squared > radius_squared {
                continue;
            }
            for &point_index in &node.points {
                let candidate = &self.points[point_index];
                let distance_squared = (candidate.point.x - point.x).powi(2)
                    + (candidate.point.y - point.y).powi(2)
                    + (candidate.point.z - point.z).powi(2);
                if distance_squared <= radius_squared {
                    neighbors.push(point_index);
                }
            }
            pending.extend(&node.children);
        }
        // Preserve insertion order so flocking sums match the naive iteration order.
        neighbors.sort_unstable();
        neighbors
            .into_iter()
            .map(|index| &self.points[index].data)
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::{rngs::StdRng, RngExt, SeedableRng};

    #[test]
    fn point_indices_and_leaf_contents_survive_subdivision() {
        let mut tree = Octree::new(1);
        tree.create_root(
            Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            Point {
                x: -10.0,
                y: -10.0,
                z: -10.0,
            },
            Point {
                x: 10.0,
                y: 10.0,
                z: 10.0,
            },
        );
        for i in 0..8 {
            let point = Point {
                x: if i & 1 == 0 { -5.0 } else { 5.0 },
                y: if i & 2 == 0 { -5.0 } else { 5.0 },
                z: if i & 4 == 0 { -5.0 } else { 5.0 },
            };
            assert_eq!(tree.insert(point, i), i);
        }
        assert_eq!(tree.nodes.len(), 9);
        for (index, point) in tree.points.iter().enumerate() {
            assert_eq!(point.data, index);
            assert_eq!(tree.query_radius(&point.point, 0.0), vec![&index]);
        }
    }

    #[test]
    fn radius_queries_match_brute_force_on_seeded_points() {
        let mut rng = StdRng::seed_from_u64(0x0C7B01D);
        let uniform: Vec<_> = (0..256)
            .map(|_| Point {
                x: rng.random_range(-10.0..10.0),
                y: rng.random_range(-10.0..10.0),
                z: rng.random_range(-10.0..10.0),
            })
            .collect();
        let clustered: Vec<_> = (0..128)
            .map(|_| Point {
                x: 0.25 + rng.random_range(-1e-4..1e-4),
                y: -0.5 + rng.random_range(-1e-4..1e-4),
                z: 0.75 + rng.random_range(-1e-4..1e-4),
            })
            .collect();
        let mut split_planes_and_boundary = vec![];
        for x in [-10.0, 0.0, 10.0] {
            for y in [-10.0, 0.0, 10.0] {
                for z in [-10.0, 0.0, 10.0] {
                    split_planes_and_boundary.push(Point { x, y, z });
                }
            }
        }
        for axis in 0..3 {
            for _ in 0..32 {
                let mut coords = [
                    rng.random_range(-10.0..10.0),
                    rng.random_range(-10.0..10.0),
                    rng.random_range(-10.0..10.0),
                ];
                coords[axis] = 0.0;
                split_planes_and_boundary.push(Point {
                    x: coords[0],
                    y: coords[1],
                    z: coords[2],
                });
            }
        }
        split_planes_and_boundary.push(Point {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        });
        let coincident = vec![
            Point {
                x: 0.25,
                y: -0.5,
                z: 0.75
            };
            64
        ];
        for points in [uniform, clustered, split_planes_and_boundary, coincident] {
            let mut queries = points.clone();
            queries.push(Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            });
            queries.extend((0..64).map(|_| Point {
                x: rng.random_range(-15.0..15.0),
                y: rng.random_range(-15.0..15.0),
                z: rng.random_range(-15.0..15.0),
            }));
            for capacity in [1, 2, 8] {
                let mut tree = Octree::new(capacity);
                tree.create_root(
                    Point {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    Point {
                        x: -10.0,
                        y: -10.0,
                        z: -10.0,
                    },
                    Point {
                        x: 10.0,
                        y: 10.0,
                        z: 10.0,
                    },
                );
                for (index, point) in points.iter().enumerate() {
                    tree.insert(point.clone(), index);
                }
                for query in &queries {
                    for radius in [0.0_f32, 0.001, 0.5, 1.0, 5.0, 40.0] {
                        let expected: Vec<_> = points
                            .iter()
                            .enumerate()
                            .filter(|(_, p)| {
                                (p.x - query.x).powi(2)
                                    + (p.y - query.y).powi(2)
                                    + (p.z - query.z).powi(2)
                                    <= radius * radius
                            })
                            .map(|(index, _)| index)
                            .collect();
                        let mut actual: Vec<_> = tree
                            .query_radius(query, radius)
                            .into_iter()
                            .copied()
                            .collect();
                        actual.sort_unstable();
                        assert_eq!(
                            actual, expected,
                            "capacity={capacity}, query={query:?}, radius={radius}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn thin_root_keeps_points_in_containing_children() {
        let mut tree = Octree::new(1);
        tree.create_root(
            Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            Point {
                x: -1.0,
                y: -1e6,
                z: -1.0,
            },
            Point {
                x: 1.0,
                y: 1e6,
                z: 1.0,
            },
        );
        let point = Point {
            x: 0.5,
            y: 1e6,
            z: 0.5,
        };
        for index in 0..3 {
            tree.insert(point.clone(), index);
        }
        for node in &tree.nodes {
            let (_, min, max) = &node.bounds;
            for &index in &node.points {
                let p = &tree.points[index].point;
                assert!(p.x >= min.x && p.x <= max.x);
                assert!(p.y >= min.y && p.y <= max.y);
                assert!(p.z >= min.z && p.z <= max.z);
            }
        }
        let mut actual: Vec<_> = tree
            .query_radius(&point, 0.0)
            .into_iter()
            .copied()
            .collect();
        actual.sort_unstable();
        assert_eq!(actual, vec![0, 1, 2]);
    }

    #[test]
    fn coincident_points_have_bounded_node_count() {
        let mut tree = Octree::new(2);
        let point = Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        assert!(tree.query_radius(&point, 1.0).is_empty());
        tree.create_root(
            point.clone(),
            Point {
                x: -1.0,
                y: -1.0,
                z: -1.0,
            },
            Point {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
        );
        for index in 0..5000 {
            tree.insert(point.clone(), index);
        }
        // One occupied path: the root plus eight children for each permitted split.
        assert_eq!(tree.nodes.len(), 1 + 8 * MAX_DEPTH);
        assert_eq!(tree.query_radius(&point, 0.0).len(), 5000);
    }
}
