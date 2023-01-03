use generational_arena::{Arena, Index};

// Octree struct usng arena allocator
pub struct Octree<T> {
    pub max_points: usize,
    pub root: Option<Index>,
    pub nodes: Arena<OctreeNode>,
    pub points: Arena<PointWrapper<T>>,
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
    pub points: Vec<Index>,
    pub children: Vec<Index>,
    // center, min, max
    pub bounds: (Point, Point, Point),
}

// Octree impl
impl<T> Octree<T> {
    pub fn new(max_points: usize) -> Self {
        Self {
            max_points: max_points,
            root: None,
            nodes: Arena::new(),
            points: Arena::new(),
        }
    }

    pub fn create_root(&mut self, center: Point, min: Point, max: Point) -> Index {
        let index = self.nodes.insert(OctreeNode {
            points: vec![],
            children: vec![],
            bounds: (center, min, max),
        });

        self.root = Some(index);

        index
    }

    pub fn insert(&mut self, point: Point, data: T) -> Index {
        let index = self.points.insert(PointWrapper { data, point });

        let point = self.points.get(index).unwrap();

        if let Some(root) = self.root {
            let mut node_index = root;
            let mut node = self.nodes.get(node_index).unwrap();

            loop {
                // Check if point is in bounds
                // if !Self::is_point_in_bounds(node, &point.point) {
                //     continue;
                // }

                if node.children.len() == 0 {
                    let mut_node = self.nodes.get_mut(node_index).unwrap();
                    mut_node.points.push(index);

                    if mut_node.points.len() > self.max_points {
                        self.subdivide(node_index);
                    }

                    break;
                } else {
                    let mut selected_index = -1;
                    let mut min_distance = f32::MAX;

                    for (i, child_index) in node.children.iter().enumerate() {
                        let child = self.nodes.get(*child_index).unwrap();

                        // Find closest center to child
                        let distance = (child.bounds.0.x - point.point.x).powi(2)
                            + (child.bounds.0.y - point.point.y).powi(2)
                            + (child.bounds.0.z - point.point.z).powi(2);

                        if distance < min_distance {
                            min_distance = distance;
                            selected_index = i as i32;
                        }

                        // if Self::is_point_in_bounds(child, &point.point) {
                        //     selected_index = i as i32;
                        //     break;
                        // }
                    }

                    if selected_index == -1 {
                        break;
                    }

                    node_index = node.children[selected_index as usize];
                    node = self.nodes.get(node_index).unwrap();
                }
            }
        }

        index
    }

    fn is_point_in_bounds(node: &OctreeNode, point: &Point) -> bool {
        let (_, min, max) = &node.bounds;

        point.x >= min.x
            && point.x <= max.x
            && point.y >= min.y
            && point.y <= max.y
            && point.z >= min.z
            && point.z <= max.z
    }

    fn subdivide(&mut self, node_index: Index) {
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

        let mut mut_node = self.nodes.get_mut(node_index).unwrap();
        mut_node.children = children.clone();

        let points = mut_node.points.clone();
        mut_node.points = vec![];

        // Fix points
        // for child_index in children.iter() {
        // let child = self.nodes.get_mut(*child_index).unwrap();

        // Assign points to children nodes (closest center)
        for point_index in points.iter() {
            let point = &self.points.get(*point_index).unwrap().point;
            let mut closest_child_index = None;
            let mut closest_distance = f32::MAX;

            for child_index in children.iter() {
                let child = self.nodes.get(*child_index).unwrap();
                let distance = (child.bounds.0.x - point.x).powi(2)
                    + (child.bounds.0.y - point.y).powi(2)
                    + (child.bounds.0.z - point.z).powi(2);

                if distance < closest_distance {
                    closest_child_index = Some(child_index);
                    closest_distance = distance;
                }
            }

            let closest_child = self.nodes.get_mut(*closest_child_index.unwrap()).unwrap();
            closest_child.points.push(*point_index);
        }

        // for point_index in points.iter() {
        //     if Octree::<T>::is_point_in_bounds(
        //         &child,
        //         &self.points.get(*point_index).unwrap().point,
        //     ) {
        //         child.points.push(*point_index);
        //     }
        // }
        // }
    }

    fn create_node(&mut self, center: Point, min: Point, max: Point) -> Index {
        let node = OctreeNode {
            bounds: (center, min, max),
            children: vec![],
            points: vec![],
        };

        self.nodes.insert(node)
    }

    pub fn find_neighbors(&self, point: &Point) -> Vec<&T> {
        let mut neighbors = vec![];

        if let Some(root) = self.root {
            let mut node_index = root;
            let mut node = self.nodes.get(node_index).unwrap();

            while node.children.len() > 0 {
                // Find the child that contains the point (closest center)
                let mut closest_child_index = None;
                let mut closest_distance = f32::MAX;

                for child_index in node.children.iter() {
                    let child = self.nodes.get(*child_index).unwrap();
                    let distance = (child.bounds.0.x - point.x).powi(2)
                        + (child.bounds.0.y - point.y).powi(2)
                        + (child.bounds.0.z - point.z).powi(2);

                    if distance < closest_distance {
                        closest_child_index = Some(child_index);
                        closest_distance = distance;
                    }

                    // if Octree::<T>::is_point_in_bounds(&child, point) {
                    //     node_index = *child_index;
                    //     node = self.nodes.get(node_index).unwrap();
                    //     break;
                    // }
                }

                node = self.nodes.get(*closest_child_index.unwrap()).unwrap();
            }

            for point_index in node.points.iter() {
                let neighbor = self.points.get(*point_index).unwrap();

                neighbors.push(&neighbor.data);
            }
        }
        neighbors
    }
}
