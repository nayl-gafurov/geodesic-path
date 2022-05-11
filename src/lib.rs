#![feature(map_try_insert)]
mod utils;
use na::{Matrix4, Vector2, Vector3};
use nalgebra as na;
use pathfinding::prelude::dijkstra;
use std::collections::HashMap;
mod triangle;
use triangle::Triangle;
use wasm_bindgen::prelude::*;
mod triangleExtended;
use triangleExtended::TriangleExtended;
mod line2;
use line2::Line2;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

#[cfg(feature = "wee_alloc")]
#[global_allocator]
static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;

type PathGraph = HashMap<usize, Vec<(usize, i32)>>;
type Wedge = Vec<Rc<RefCell<TriangleExtended>>>;
const PI: f32 = std::f32::consts::PI - 0.2;

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console, js_name = log)]
    fn log_usize(a: usize);
}

#[wasm_bindgen]
pub fn get_path(
    start: usize,
    end: usize,
    raw_coordinates: Vec<f32>,
    indices: Vec<usize>,
) -> Vec<f32> {
    let data = Data::new(raw_coordinates, indices);
    data.get_path(start, end)
}

struct Data {
    coordinates: Vec<Vector3<f32>>,
    path_graph: PathGraph,
    triangles: Vec<Rc<RefCell<TriangleExtended>>>,
}

impl Data {
    pub fn new(raw_coordinates: Vec<f32>, indices: Vec<usize>) -> Data {
        console_error_panic_hook::set_once();
        let mut coordinates: Vec<Vector3<f32>> = Vec::with_capacity(raw_coordinates.len());
        let mut path_graph: PathGraph = HashMap::new();
        let mut triangles = vec![];

        for coord_chunk in raw_coordinates.chunks(3) {
            coordinates
                .push(Vector3::new(coord_chunk[0], coord_chunk[1], coord_chunk[2]) * 50000.0);
        }

        for chunk in indices.chunks(3) {
            let indices = [chunk[0], chunk[1], chunk[2]];
            let [a, b, c] = indices.map(|i| coordinates[i]);
            triangles.push(Rc::new(RefCell::new(TriangleExtended {
                indices: [chunk[0], chunk[1], chunk[2]],
                triangle: Triangle::new(a, b, c),
                triangle_unfolded: None,
                sub_triangles: vec![],
                parent: RefCell::new(Weak::new()),
            })));

            for (i, centeral_index) in chunk.iter().enumerate() {
                let indices: [usize; 2] = if i == 0 {
                    [1, 2]
                } else if i == 1 {
                    [0, 2]
                } else {
                    [0, 1]
                };
                match path_graph.get_mut(centeral_index) {
                    Some(vec) => indices.iter().for_each(|index| {
                        vec.push((
                            chunk[*index],
                            ((coordinates[*centeral_index] - coordinates[chunk[*index]])
                                .magnitude()
                                * 1000.0) as i32,
                        ))
                    }),
                    None => {
                        path_graph.insert(
                            *centeral_index,
                            indices
                                .iter()
                                .map(|index| {
                                    (
                                        chunk[*index],
                                        ((coordinates[*centeral_index]
                                            - coordinates[chunk[*index]])
                                            .magnitude()
                                            * 1000.0)
                                            as i32,
                                    )
                                })
                                .collect(),
                        );
                    }
                };
            }
        }
        Data {
            coordinates,
            path_graph,
            triangles,
        }
    }

    pub fn get_path(mut self, start: usize, end: usize) -> Vec<f32> {
        let mut dijkstra_path = self.compute_dijkstra(start, end).unwrap();
        let mut finish = true;
        let mut counter = 0;
        loop {
            for (index, path) in dijkstra_path.windows(3).enumerate() {
                let new_path_segment = self.get_path_segment(path);
                match new_path_segment {
                    Some(val) => {
                        dijkstra_path.splice(index..index + 3, val);

                        let mut result = vec![];
                        for index in dijkstra_path.iter() {
                            let point = self.coordinates[*index];
                            result.push(point.x);
                            result.push(point.y);
                            result.push(point.z);
                        }
                        finish = false;
                        break;
                    }
                    None => finish = true,
                }
            }
            counter += 1;
            log_usize(counter);
            if finish == true {
                break;
            }
        }
        let mut result = vec![];
        for index in dijkstra_path.iter() {
            let point = self.coordinates[*index] / 50000.0;
            result.push(point.x);
            result.push(point.y);
            result.push(point.z);
        }

        result
    }

    fn get_path_segment(&mut self, path: &[usize]) -> Option<Vec<usize>> {
        let mut new_path = vec![];
        match self.get_wedge(path) {
            Some(mut wedge) => {
                if wedge.len() == 2 {
                    let a1 = wedge[0].borrow().indices[0];
                    let b1 = wedge[0].borrow().indices[1];
                    let c2 = wedge[1].borrow().indices[2];

                    match wedge[0].borrow().parent.borrow().upgrade() {
                        Some(parent1) => match wedge[1].borrow().parent.borrow().upgrade() {
                            Some(parent2) => {
                                let parent1_indices = parent1.borrow().indices;
                                let parent2_indices = parent2.borrow().indices;
                                if !parent1_indices.contains(&a1)
                                    && !parent1_indices.contains(&b1)
                                    && !parent2_indices.contains(&c2)
                                {
                                    if wedge[0].borrow().triangle.area() < 2.0
                                        && wedge[1].borrow().triangle.area() < 2.0
                                    {
                                        new_path.push(b1);
                                        new_path.push(wedge[0].borrow().indices[2]);
                                        new_path.push(c2);

                                        return Some(new_path);
                                    }
                                }
                            }
                            None => {}
                        },
                        None => {}
                    }
                }

                Data::unfold_wedge(&mut wedge);

                for index in self.cut(&wedge).iter() {
                    new_path.push(*index);
                }

                if new_path.len() > 0 {
                    if new_path.len() < 2 {
                        new_path = vec![path[0], path[2]];
                        match wedge[0].borrow().parent.borrow().upgrade() {
                            Some(parent) => {
                                parent.borrow_mut().sub_triangles.clear();
                            }
                            None => {}
                        }
                    }
                    return Some(new_path);
                }
                None
            }
            None => None,
        }
    }

    fn compute_dijkstra(&self, start: usize, end: usize) -> Option<Vec<usize>> {
        match dijkstra(
            &start,
            |index| -> Vec<(usize, i32)> { self.path_graph.get(index).unwrap().clone() },
            |p| *p == end,
        ) {
            Some(val) => Some(val.0),
            None => None,
        }
    }

    fn get_triangle_pair_by_edge(
        &mut self,
        a: usize,
        b: usize,
        neighbor: Option<Rc<RefCell<TriangleExtended>>>,
    ) -> Vec<Rc<RefCell<TriangleExtended>>> {
        let mut result = Vec::with_capacity(2);
        for triangle in self.triangles.iter() {
            let mut try_on_parent = true;
            let sub_triangles_len = triangle.borrow().sub_triangles.len();
            if sub_triangles_len > 0 {
                let sub_triangles = &triangle.borrow().sub_triangles;
                for sub_triangle in sub_triangles.iter() {
                    let indices = sub_triangle.borrow().indices;
                    let checker: Vec<(usize, &usize)> = indices
                        .iter()
                        .enumerate()
                        .filter(|&(_, index)| *index == a || *index == b)
                        .collect();
                    if checker.len() == 2 {
                        match &neighbor {
                            Some(val) => {
                                if Rc::ptr_eq(sub_triangle, val) {
                                    try_on_parent = false;
                                    continue;
                                }
                            }
                            None => {}
                        }
                        let c = sub_triangle.borrow().indices[3 - checker[0].0 - checker[1].0];
                        let indices = [a, b, c];
                        sub_triangle.borrow_mut().indices = [a, b, c];
                        sub_triangle.borrow_mut().triangle = self.get_triangle(indices);
                        sub_triangle.borrow_mut().triangle_unfolded = None;
                        result.push(Rc::clone(sub_triangle));
                        try_on_parent = false;
                    }
                }
            }

            if try_on_parent == true {
                let indices = triangle.borrow().indices;

                let checker: Vec<(usize, &usize)> = indices
                    .iter()
                    .enumerate()
                    .filter(|&(_, index)| *index == a || *index == b)
                    .collect();

                if checker.len() == 2 {
                    match &neighbor {
                        Some(val) => {
                            if Rc::ptr_eq(triangle, val) {
                                continue;
                            }
                        }
                        None => {}
                    }
                    let c = triangle.borrow().indices[3 - checker[0].0 - checker[1].0];
                    let indices = [a, b, c];
                    triangle.borrow_mut().indices = [a, b, c];
                    triangle.borrow_mut().triangle = self.get_triangle(indices);
                    triangle.borrow_mut().triangle_unfolded = None;
                    result.push(Rc::clone(triangle));
                }
            }
        }
        result
    }

    fn get_wedge(&mut self, path: &[usize]) -> Option<Wedge> {
        let mut start = path[0];
        let middle = path[1];
        let end = path[2];

        let triangle_pair = self.get_triangle_pair_by_edge(middle, start, None);

        let mut wedges: Vec<Vec<Rc<RefCell<TriangleExtended>>>> =
            triangle_pair.into_iter().map(|x| vec![x]).collect();

        for wedge in wedges.iter_mut() {
            start = wedge.last().unwrap().borrow().indices[2];

            while start != end {
                let last_triangle = wedge.last().unwrap();
                let pair =
                    self.get_triangle_pair_by_edge(middle, start, Some(Rc::clone(last_triangle)));
                let next_triangle = Rc::clone(&pair[0]);
                start = pair[0].borrow().indices[2];
                wedge.push(next_triangle);
            }
        }
        let wedge = wedges
            .iter()
            .zip(wedges.iter().map(|wedge| {
                wedge
                    .iter()
                    .map(|triangle| match triangle.borrow().triangle.angles() {
                        Some(val) => val[0].abs(),
                        None => std::f32::consts::PI,
                    })
                    .sum::<f32>()
            }))
            .reduce(|min, item| {
                if min.1.abs() <= item.1.abs() {
                    min
                } else {
                    item
                }
            })
            .unwrap();
        if wedge.1.abs() < PI {
            return Some(wedge.0.clone());
        }
        None
    }

    fn unfold_wedge(wedge: &mut Wedge) {
        let mut basis_to = Matrix4::default();
        basis_to.fill_with_identity();
        wedge.iter_mut().for_each(|triangle| {
            let basis_from = triangle.borrow().get_basis(
                [triangle.borrow().indices[0], triangle.borrow().indices[1]],
                triangle.borrow().triangle,
            );
            triangle.borrow_mut().transform(basis_from, basis_to);
            basis_to = triangle.borrow().get_basis(
                [triangle.borrow().indices[0], triangle.borrow().indices[2]],
                triangle.borrow().triangle_unfolded.unwrap(),
            )
        });
    }

    fn cut(&mut self, wedge: &Vec<Rc<RefCell<TriangleExtended>>>) -> Vec<usize> {
        let start = wedge[0].borrow().triangle_unfolded.unwrap().b;
        let end = wedge.last().unwrap().borrow().triangle_unfolded.unwrap().c;
        let mut path = vec![(wedge[0].borrow().indices[1], wedge[0].borrow().triangle.b)];
        let last_triangle = wedge.last().unwrap();

        let path_line = Line2::new(v2_from_v3(start), v2_from_v3(end));

        for i in 0..wedge.len() {
            let mut triangle = Rc::clone(&wedge[i]);
            let parent = triangle.borrow().parent.borrow_mut().upgrade();
            if i != wedge.len() - 1 {
                match &parent {
                    Some(parent) => {
                        let next_triangle = Rc::clone(&wedge[i + 1]);
                        let next_triangle_parent =
                            next_triangle.borrow().parent.borrow_mut().upgrade();
                        match next_triangle_parent {
                            Some(next_triangle_parent) => {
                                if Rc::ptr_eq(&parent, &next_triangle_parent) {
                                    continue;
                                }
                            }
                            None => {}
                        }
                    }
                    None => {}
                }
            }

            let edge = Line2::new(
                v2_from_v3(triangle.borrow().triangle_unfolded.unwrap().a),
                v2_from_v3(triangle.borrow().triangle_unfolded.unwrap().c),
            );

            match path_line.intersect(&edge) {
                Some(intersection) => {
                    let a = self.coordinates[triangle.borrow().indices[0]];
                    let b = self.coordinates[triangle.borrow().indices[2]];
                    let point = a + (b - a).normalize() * ((intersection - edge.start).magnitude());

                    self.coordinates.push(point);
                    let index = self.coordinates.len() - 1;

                    match parent {
                        Some(parent) => {
                            triangle = parent;
                        }
                        None => {}
                    }

                    triangle.borrow_mut().sub_triangles.clear();
                    let sub_triangles =
                        triangle.borrow().cut(*path.last().unwrap(), (index, point));
                    if sub_triangles.len() > 0 {
                        for sub_triangle in sub_triangles {
                            *sub_triangle.parent.borrow_mut() = Rc::downgrade(&triangle);
                            triangle.borrow_mut().add_child(sub_triangle);
                        }
                        path.push((index, point));
                    }
                }
                None => {
                    if Rc::ptr_eq(&triangle, last_triangle) {
                        match parent {
                            Some(parent) => {
                                triangle = parent;
                            }
                            None => {}
                        }
                        let index = last_triangle.borrow().indices[2];
                        let point = last_triangle.borrow().triangle.c;
                        triangle.borrow_mut().sub_triangles.clear();
                        let sub_triangles =
                            triangle.borrow().cut(*path.last().unwrap(), (index, point));
                        if sub_triangles.len() > 0 {
                            for sub_triangle in sub_triangles {
                                *sub_triangle.parent.borrow_mut() = Rc::downgrade(&triangle);
                                triangle.borrow_mut().add_child(sub_triangle);
                            }
                            path.push((index, point));
                        }
                    }
                }
            }
        }

        path.iter().map(|x| x.0).collect()
    }

    fn get_triangle(&self, indices: [usize; 3]) -> Triangle {
        Triangle::from_array(indices.map(|i| self.coordinates[i]))
    }
}

fn v2_from_v3(v: Vector3<f32>) -> Vector2<f32> {
    Vector2::new(v.x, v.y)
}
