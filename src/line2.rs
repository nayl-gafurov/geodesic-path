use na::Vector2;
use nalgebra as na;

pub struct Line2 {
    pub start: Vector2<f32>,
    pub end: Vector2<f32>,
}

impl Line2 {
    pub fn new(start: Vector2<f32>, end: Vector2<f32>) -> Line2 {
        Line2 { start, end }
    }
    fn get_dir(&self) -> Vector2<f32> {
        self.end - self.start
    }

    fn length(&self) -> f32 {
        self.get_dir().magnitude()
    }

    fn get_eq(&self) -> (f32, f32) {
        let v = self.get_dir();
        let k = v.y / v.x;
        let b = self.start.y - k * self.start.x;
        (k, b)
    }

    fn is_collinear(&self, line: &Line2) -> bool {
        let a = self.get_dir();
        let b = line.get_dir();
        (a.x * b.y - a.y * b.x).abs() < std::f32::EPSILON
    }
    pub fn intersect(&self, line: &Line2) -> Option<Vector2<f32>> {
        if self.is_collinear(line) {
            return None;
        } else {
            let dir = self.get_dir();
            let eq1 = self.get_eq();
            let eq2 = line.get_eq();
            let x = (eq2.1 - eq1.1) / (eq1.0 - eq2.0);
            let y = eq1.0 * x + eq1.1;
            let intersect = Vector2::new(x, y);
            let sign_length = (intersect - self.start).dot(&dir.normalize());
            if sign_length < dir.magnitude() - 0.01 && sign_length > 0.01 {
                return Some(intersect);
            }
            return None;
        }
    }
}
