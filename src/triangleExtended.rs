use crate::triangle::Triangle;
use na::{Matrix4, Vector3, Vector4};
use nalgebra as na;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

#[derive(Debug, Clone)]
pub struct TriangleExtended {
    pub indices: [usize; 3],
    pub triangle: Triangle,
    pub triangle_unfolded: Option<Triangle>,
    pub sub_triangles: Vec<Rc<RefCell<TriangleExtended>>>,
    pub parent: RefCell<Weak<RefCell<TriangleExtended>>>,
}

impl TriangleExtended {
    pub fn get_basis(&self, edge: [usize; 2], triangle: Triangle) -> Matrix4<f32> {
        let tr = triangle.to_array();
        let a_index = self.indices.iter().position(|&x| x == edge[0]).unwrap();
        let b_index = self.indices.iter().position(|&x| x == edge[1]).unwrap();
        let c_index = 3 - a_index - b_index;

        let a = tr[a_index];
        let b = tr[b_index];
        let c = tr[c_index];

        let x = (b - a).normalize();
        let z;
        if b_index == 1 {
            z = (c - a).cross(&x).normalize();
        } else {
            z = x.cross(&(c - a)).normalize();
        }
        let y = z.cross(&x);
        Matrix4::from_columns(&[
            x.to_homogeneous(),
            y.to_homogeneous(),
            z.to_homogeneous(),
            a.push(1.0),
        ])
    }

    pub fn transform(&mut self, from: Matrix4<f32>, to: Matrix4<f32>) {
        let m = to * from.try_inverse().unwrap();

        let mut transformed = self.triangle.clone();
        transformed.a = v3_from_v4(m * transformed.a.push(1.0));
        transformed.b = v3_from_v4(m * transformed.b.push(1.0));
        transformed.c = v3_from_v4(m * transformed.c.push(1.0));
        self.triangle_unfolded = Some(transformed);

        for sub_triangle in self.sub_triangles.iter() {
            sub_triangle.borrow_mut().transform(from, to);
        }
    }

    pub fn add_child(&mut self, child: TriangleExtended) {
        self.sub_triangles.push(Rc::new(RefCell::new(child)));
    }

    pub fn cut(
        &self,
        first: (usize, Vector3<f32>),
        second: (usize, Vector3<f32>),
    ) -> Vec<TriangleExtended> {
        let mut result: Vec<TriangleExtended> = vec![];
        let a = (self.indices[0], self.triangle.a);
        let b = (self.indices[1], self.triangle.b);
        let c = (self.indices[2], self.triangle.c);

        let sides = [(b, a, c), (c, a, b), (c, b, a)];

        let side1 = if !self.triangle.to_array().iter().any(|&x| x == first.1) {
            sides
                .iter()
                .find(|(a, b, _)| (b.1 - a.1).cross(&(first.1 - a.1)).magnitude().abs() < 0.1)
        } else {
            None
        };

        match side1 {
            Some((a, b, c)) => {
                result.push(TriangleExtended {
                    indices: [a.0, first.0, c.0],
                    triangle: Triangle::new(a.1, first.1, c.1),
                    triangle_unfolded: None,
                    sub_triangles: vec![],
                    parent: RefCell::new(Weak::new()),
                });
                result.push(TriangleExtended {
                    indices: [first.0, b.0, c.0],
                    triangle: Triangle::new(first.1, b.1, c.1),
                    triangle_unfolded: None,
                    sub_triangles: vec![],
                    parent: RefCell::new(Weak::new()),
                });
            }
            None => {}
        }

        let side2 = if !self.triangle.to_array().iter().any(|&x| x == second.1) {
            sides
                .iter()
                .find(|(a, b, _)| (b.1 - a.1).cross(&(second.1 - a.1)).magnitude().abs() < 0.1)
        } else {
            None
        };

        match side2 {
            Some((l, m, n)) => match side1 {
                Some(_) => {
                    let tr: Vec<TriangleExtended> = vec![
                        TriangleExtended {
                            indices: [second.0, first.0, l.0],
                            triangle: Triangle::new(second.1, first.1, l.1),
                            triangle_unfolded: None,
                            sub_triangles: vec![],
                            parent: RefCell::new(Weak::new()),
                        },
                        TriangleExtended {
                            indices: [second.0, m.0, first.0],
                            triangle: Triangle::new(second.1, m.1, first.1),
                            triangle_unfolded: None,
                            sub_triangles: vec![],
                            parent: RefCell::new(Weak::new()),
                        },
                    ];
                    let s2 = [second.0, first.0, l.0, m.0];

                    let index = result
                        .iter()
                        .enumerate()
                        .find(|(_, x)| x.indices.iter().all(|index| s2.contains(index)))
                        .unwrap()
                        .0;

                    if index == 0 {
                        result.splice(0..1, tr);
                    } else {
                        result.splice(1.., tr);
                    }
                }
                None => {
                    result.push(TriangleExtended {
                        indices: [l.0, second.0, n.0],
                        triangle: Triangle::new(l.1, first.1, n.1),
                        triangle_unfolded: None,
                        sub_triangles: vec![],
                        parent: RefCell::new(Weak::new()),
                    });
                    result.push(TriangleExtended {
                        indices: [second.0, m.0, n.0],
                        triangle: Triangle::new(second.1, m.1, n.1),
                        triangle_unfolded: None,
                        sub_triangles: vec![],
                        parent: RefCell::new(Weak::new()),
                    });
                }
            },
            None => {}
        }
        result
    }
}

fn v3_from_v4(v: Vector4<f32>) -> Vector3<f32> {
    Vector3::new(v.x, v.y, v.z)
}
