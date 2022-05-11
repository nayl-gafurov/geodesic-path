use na::Vector3;
use nalgebra as na;
use std::ops::{Add, Sub};

type Point = Vector3<f32>;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Triangle {
    pub a: Vector3<f32>,
    pub b: Vector3<f32>,
    pub c: Vector3<f32>,
}
impl Triangle {
    pub fn new(a: Point, b: Point, c: Point) -> Triangle {
        Triangle { a, b, c }
    }

    pub fn to_array(&self) -> [Vector3<f32>; 3] {
        [self.a, self.b, self.c]
    }

    ///Creates new Triangle from array of Points.
    pub fn from_array(points: [Point; 3]) -> Triangle {
        Triangle {
            a: points[0],
            b: points[1],
            c: points[2],
        }
    }
    ///Returns two opposite points of axis-aligned bounding box.
    pub fn aabb(&self) -> [Point; 2] {
        let mut c_x = [self.a.x, self.b.x, self.c.x];
        let mut c_y = [self.a.y, self.b.y, self.c.y];
        let mut c_z = [self.a.z, self.b.z, self.c.z];
        c_x.sort_by(|i, j| i.partial_cmp(j).unwrap());
        c_y.sort_by(|i, j| i.partial_cmp(j).unwrap());
        c_z.sort_by(|i, j| i.partial_cmp(j).unwrap());
        [
            Point::new(c_x[0], c_y[0], c_z[0]),
            Point::new(c_x[2], c_y[2], c_z[2]),
        ]
    }

    ///Gets angles of the triangle.
    pub fn angles(&self) -> Option<[f32; 3]> {
        if self.is_collinear() {
            return None;
        }
        let [la, lb, lc] = self.sides();
        let alpha = ((lb.powi(2) + lc.powi(2) - la.powi(2)) / (2.0 * lb * lc)).acos();
        let beta = ((la.powi(2) + lc.powi(2) - lb.powi(2)) / (2.0 * la * lc)).acos();
        let gamma = std::f32::consts::PI - alpha - beta;
        Some([alpha, beta, gamma])
    }

    ///Gets area of the triangle.
    pub fn area(&self) -> f32 {
        let s = self.semiperimeter();
        let [la, lb, lc] = self.sides();
        (s * (s - la) * (s - lb) * (s - lc)).sqrt()
    }

    ///Converts barycentric coordinates of given point to cartesian coordinate system.
    pub fn barycentric_to_cartesian(&self, pt: &Point) -> Point {
        let x = pt.x * self.a.x + pt.y * self.b.x + pt.z * self.c.x;
        let y = pt.x * self.a.y + pt.y * self.b.y + pt.z * self.c.y;
        let z = pt.x * self.a.z + pt.y * self.b.z + pt.z * self.c.z;
        Point::new(x, y, z)
    }

    ///Converts cartesian coordinates of given point to barycentric coordinate system.
    pub fn cartesian_to_barycentric(&self, pt: &Point) -> Point {
        let v0 = Point::new(
            self.b.x - self.a.x,
            self.b.y - self.a.y,
            self.b.z - self.a.z,
        );
        let v1 = Point::new(
            self.c.x - self.a.x,
            self.c.y - self.a.y,
            self.c.z - self.a.z,
        );
        let v2 = Point::new(pt.x - self.a.x, pt.y - self.a.y, pt.z - self.a.z);
        let den = 1.0 / (v0.x * v1.y - v1.x * v0.y);
        let v = (v2.x * v1.y - v1.x * v2.y) * den;
        let w = (v0.x * v2.y - v2.x * v0.y) * den;
        let u = 1.0 - v - w;
        Point::new(u, v, w)
    }

    ///Gets centroid of the triangle.
    pub fn centroid(&self) -> Point {
        Point::new(
            (self.a.x + self.b.x + self.c.x) / 3.0,
            (self.a.y + self.b.y + self.c.y) / 3.0,
            (self.a.z + self.b.z + self.c.z) / 3.0,
        )
    }

    ///Gets radius of a circle that passes through all of the triangle's vertices, so called
    ///circumradius.
    pub fn circumradius(&self) -> Option<f32> {
        if self.is_collinear() {
            return None;
        }
        Some(self.sides().iter().product::<f32>() / (4.0 * self.area()))
    }

    ///Checks whether a given point lies inside the triangle.
    pub fn has_point(&self, pt: Point) -> bool {
        fn sign(a: &Point, b: &Point, c: &Point) -> f32 {
            ((a.x - c.x) * (b.y - c.y) - (b.x - c.x) * (a.y - c.y)) as f32
        }
        let d1 = sign(&pt, &self.a, &self.b);
        let d2 = sign(&pt, &self.b, &self.c);
        let d3 = sign(&pt, &self.c, &self.a);
        let has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
        let has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);
        !(has_neg && has_pos)
    }

    ///Gets the heights of the triangle.
    pub fn heights(&self) -> Option<[f32; 3]> {
        if self.is_collinear() {
            return None;
        }
        let double_area = 2.0 * self.area();
        let [la, lb, lc] = self.sides();
        Some([double_area / la, double_area / lb, double_area / lc])
    }

    ///Gets radius of a circle which is tangent to each side of the triangle, so called inradius.
    pub fn inradius(&self) -> Option<f32> {
        if self.is_collinear() {
            return None;
        }
        Some(self.area() / self.semiperimeter())
    }

    ///Checks if points of triangle are collinear.
    pub fn is_collinear(&self) -> bool {
        self.area().eq(&0.0)
    }

    ///Checks if the triangle is equilateral.
    pub fn is_equilateral(&self) -> bool {
        let sides = self.sides();
        sides[0].eq(&sides[1]) && sides[1].eq(&sides[2])
    }

    ///Checks if the triangle is golden or sublime.
    pub fn is_golden(&self) -> bool {
        if !self.is_isosceles() {
            return false;
        }
        let mut sides = self.sides();
        sides.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let min = sides[0];
        let max = sides[2];
        (max / min).eq(&((1.0 + 5.0_f32.sqrt()) / 2.0))
    }

    ///Checks if the triangle is isosceles.
    pub fn is_isosceles(&self) -> bool {
        let sides = self.sides();
        sides[0].eq(&sides[1]) || sides[1].eq(&sides[2]) || sides[2].eq(&sides[0])
    }

    ///Checks if the triangle is right-angled.
    pub fn is_right(&self) -> bool {
        if self.is_collinear() {
            return false;
        }
        let angles = self.angles().unwrap();
        let half_pi = std::f32::consts::PI / 2.0;
        angles[0].eq(&half_pi) || angles[1].eq(&half_pi) || angles[2].eq(&half_pi)
    }

    ///Gets medians of the triangle.
    pub fn medians(&self) -> [f32; 3] {
        let [la, lb, lc] = self.sides();
        let ma = (2.0 * lb.powi(2) + 2.0 * lc.powi(2) - la.powi(2)).sqrt() / 2.0;
        let mb = (2.0 * lc.powi(2) + 2.0 * la.powi(2) - lb.powi(2)).sqrt() / 2.0;
        let mc = (2.0 * la.powi(2) + 2.0 * lb.powi(2) - lc.powi(2)).sqrt() / 2.0;
        [ma, mb, mc]
    }

    ///Gets normal of the triangle, depending on vertices order.
    pub fn normal(&self) -> Option<Point> {
        if self.is_collinear() {
            return None;
        }
        let u = self.b - self.a;
        let v = self.c - self.a;
        let n = Point::new(
            u.y * v.z - u.z * v.y,
            u.z * v.x - u.x * v.z,
            u.x * v.y - u.y * v.x,
        );
        Some(n.normalize())
    }

    ///Gets perimeter of the triangle.
    pub fn perimeter(&self) -> f32 {
        return self.sides().iter().sum();
    }

    ///Gets distance from ray origin to intersection with triangle. Möller & f32rumbore algorithm.
    pub fn ray_intersection(&self, ray_orig: &Point, ray_dir: &Point) -> Option<f32> {
        if self.is_collinear() {
            return None;
        }

        let e1 = self.b - self.a;
        let e2 = self.c - self.a;
        let pvec = ray_dir.cross(&e2);
        let det = e1.dot(&pvec);
        if det.abs() < f32::MIN {
            return None;
        }

        let inv_det = 1.0 / det;
        let tvec = *ray_orig - self.a;
        let u = tvec.dot(&pvec) * inv_det;
        if u < 0.0 || u > 1.0 {
            return None;
        }

        let qvec = tvec.cross(&e1);
        let v = ray_dir.dot(&qvec) * inv_det;
        if v < 0.0 || (u + v) > 1.0 {
            return None;
        }

        Some(e2.dot(&qvec) * inv_det)
    }

    ///Gets semiperimeter of the triangle.
    pub fn semiperimeter(&self) -> f32 {
        self.perimeter() / 2.0
    }

    ///Gets lengths of sides opposite to points.
    pub fn sides(&self) -> [f32; 3] {
        [
            (self.b - self.c).magnitude(),
            (self.c - self.a).magnitude(),
            (self.a - self.b).magnitude(),
        ]
    }

    ///Checks if Triangle Points are sorted by axis.
    pub fn is_sorted_by(self, axis_name: char) -> bool {
        match axis_name {
            'x' | 'X' | '0' => self.a.x <= self.b.x && self.b.x <= self.c.x,
            'z' | 'Z' | '2' => self.a.z <= self.b.z && self.b.z <= self.c.z,
            _ => self.a.y <= self.b.y && self.b.y <= self.c.y,
        }
    }

    ///Creates new Triangle with Points sorted by axis.
    pub fn sorted_by(self, axis_name: char) -> Triangle {
        let mut sorted = [self.a, self.b, self.c];
        match axis_name {
            'x' | 'X' | '0' => sorted.sort_by(|a, b| a.x.partial_cmp(&b.x).unwrap()),
            'z' | 'Z' | '2' => sorted.sort_by(|a, b| a.z.partial_cmp(&b.z).unwrap()),
            _ => sorted.sort_by(|a, b| a.y.partial_cmp(&b.y).unwrap()),
        };
        Triangle::from_array(sorted)
    }
}
