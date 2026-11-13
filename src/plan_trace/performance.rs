//! Retained scene buffers follow the same renderer path as Matrix Grid.
use super::*;
use mini_fem::geo::{BooleanOps, BoundingRect, MultiPolygon, Polygon};
use rstar::{primitives::GeomWithData, primitives::Rectangle, RTree, AABB};
use std::{cell::RefCell, sync::Arc};
#[derive(Default)]
pub(super) struct Perf {
    pub(super) model_stats: RefCell<Option<((u64, u64), fem_ui::ModelStats)>>,
    pub(super) membrane_thickness: RefCell<Option<((u64, u64), Arc<Vec<f64>>)>>,
    load_fill: RefCell<LoadFillCache>,
    load_boundaries: RefCell<BTreeMap<u64, LoadBoundary>>,
    edges: RefCell<Option<Retained>>,
    results: RefCell<Option<Retained>>,
    result_world: RefCell<Option<WorldResults>>,
    range: RefCell<Option<([u64; 8], [f64; 2], [f64; 2])>>,
    pub(super) load_shapes: RefCell<Option<([u64; 8], Arc<Vec<super::load_mosaics::Shape>>)>>,
    pub(super) reactions: RefCell<Option<([u64; 8], Arc<Vec<super::support_reactions::Total>>)>>,
    smoothed: RefCell<Option<([u64; 8], Arc<Vec<[f64; 3]>>)>>,
    geometry: RefCell<Option<(u64, Arc<ResultGeometry>)>>,
    label_values: RefCell<Option<([u64; 8], Arc<Vec<(Point, f64)>>)>>,
    last_frame: Option<f64>,
    last_activity: f64,
    fps: f64,
    next_frame: Option<Instant>,
}
struct LoadBoundary {
    triangles: std::sync::Weak<Vec<[Point; 3]>>,
    edges: Arc<Vec<[Point; 2]>>,
}
#[derive(Default)]
struct LoadFillCache {
    frame: u64,
    domain_index: Option<BTreeMap<u64, Vec<(f64, usize)>>>,
    planes: BTreeMap<(u64, u64), LoadPlaneFill>,
}
#[derive(Default)]
struct LoadPlaneFill {
    checked_frame: u64,
    domain: Vec<Contour>,
    slab_area: Option<LoadSlabArea>,
    self_weight_areas: Option<BTreeMap<u64, MultiPolygon<f64>>>,
    fills: BTreeMap<u64, (Contour, Arc<Vec<[Point; 3]>>)>,
}

type IndexedSlab = GeomWithData<Rectangle<[f64; 2]>, Polygon<f64>>;

struct LoadSlabArea(RTree<IndexedSlab>);

impl LoadSlabArea {
    fn new(domain: &[Contour]) -> Self {
        let slabs = model::visible_load_domain(domain)
            .0
            .into_iter()
            .filter_map(|slab| {
                let bounds = slab.bounding_rect()?;
                Some(GeomWithData::new(
                    Rectangle::from_aabb(AABB::from_corners(
                        [bounds.min().x, bounds.min().y],
                        [bounds.max().x, bounds.max().y],
                    )),
                    slab,
                ))
            })
            .collect();
        Self(RTree::bulk_load(slabs))
    }

    fn clip(&self, load: &MultiPolygon<f64>) -> MultiPolygon<f64> {
        let Some(bounds) = load.bounding_rect() else {
            return MultiPolygon(vec![]);
        };
        let search = AABB::from_corners(
            [bounds.min().x, bounds.min().y],
            [bounds.max().x, bounds.max().y],
        );
        MultiPolygon(
            self.0
                .locate_in_envelope_intersecting(&search)
                .flat_map(|slab| load.intersection(&slab.data).0)
                .collect(),
        )
    }
}
impl App {
    pub(super) fn cached_load_boundary(
        &self,
        id: u64,
        triangles: &Arc<Vec<[Point; 3]>>,
    ) -> Arc<Vec<[Point; 2]>> {
        let mut cache = self.perf.load_boundaries.borrow_mut();
        if let Some(boundary) = cache.get(&id) {
            if boundary.triangles.ptr_eq(&Arc::downgrade(triangles)) {
                return boundary.edges.clone();
            }
        }
        if cache.len() >= 4096 {
            cache.clear();
        }
        let edges = Arc::new(spatial_loads::boundary(triangles));
        cache.insert(
            id,
            LoadBoundary {
                triangles: Arc::downgrade(triangles),
                edges: edges.clone(),
            },
        );
        edges
    }
    pub(super) fn start_load_fill_frame(&self) {
        let mut cache = self.perf.load_fill.borrow_mut();
        cache.frame = cache.frame.wrapping_add(1);
        cache.domain_index = None;
    }

    fn load_fill_floor(&self, id: u64) -> u64 {
        if self.building.enabled {
            self.building.owner(id)
        } else {
            0
        }
    }

    fn load_fill_domain_index(&self) -> BTreeMap<u64, Vec<(f64, usize)>> {
        let mut index: BTreeMap<u64, Vec<(f64, usize)>> = BTreeMap::new();
        for (i, contour) in self.contours.iter().enumerate() {
            if matches!(contour.kind, Kind::Slab | Kind::Hole | Kind::LoadHole) {
                index
                    .entry(self.load_fill_floor(contour.id))
                    .or_default()
                    .push((self.source_heights(contour)[0], i));
            }
        }
        for entries in index.values_mut() {
            entries.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
        }
        index
    }

    pub(super) fn cached_load_fill(&self, load: &Contour) -> Arc<Vec<[Point; 3]>> {
        const ELEVATION_TOLERANCE_MM: f64 = 0.05;
        let floor = if self.building.enabled
            && load.load.is_some()
            && load.placement.slab != Default::default()
        {
            0
        } else {
            self.load_fill_floor(load.id)
        };
        let z = self.source_heights(load)[0];
        let key = (floor, if z == 0. { 0 } else { z.to_bits() });
        let mut cache = self.perf.load_fill.borrow_mut();
        let frame = cache.frame;
        if cache
            .planes
            .get(&key)
            .is_none_or(|p| p.checked_frame != frame)
        {
            let index = cache
                .domain_index
                .get_or_insert_with(|| self.load_fill_domain_index());
            let mut domain = index
                .iter()
                .filter(|(&owner, _)| floor == 0 || owner == floor)
                .flat_map(|(_, entries)| {
                    let lo = entries.partition_point(|&(h, _)| h < z - ELEVATION_TOLERANCE_MM);
                    let hi = entries.partition_point(|&(h, _)| h <= z + ELEVATION_TOLERANCE_MM);
                    entries[lo..hi]
                        .iter()
                        .filter_map(|&(h, i)| ((h - z).abs() < ELEVATION_TOLERANCE_MM).then_some(i))
                })
                .collect::<Vec<_>>();
            // Self-weight ownership follows the order of the source slabs.
            domain.sort_unstable();
            let plane = cache.planes.entry(key).or_default();
            let domain = domain.into_iter().map(|i| &self.contours[i]);
            if !plane.domain.iter().eq(domain.clone()) {
                *plane = LoadPlaneFill {
                    domain: domain.cloned().collect(),
                    ..Default::default()
                };
            }
            plane.checked_frame = frame;
        }
        let plane = cache.planes.get_mut(&key).unwrap();
        if let Some((old, triangles)) = plane.fills.get(&load.id) {
            if old == load {
                return triangles.clone();
            }
        }
        let load_area;
        let polygons = if load.kind == Kind::LoadArea && load.load.is_none() {
            plane
                .self_weight_areas
                .get_or_insert_with(|| model::visible_self_weight_areas(&plane.domain))
                .get(&load.id)
        } else {
            let slab_area = plane
                .slab_area
                .get_or_insert_with(|| LoadSlabArea::new(&plane.domain));
            load_area = slab_area.clip(&model::load_area_material(&plane.domain, load));
            Some(&load_area)
        };
        let triangles = Arc::new(
            polygons
                .into_iter()
                .flat_map(|area| &area.0)
                .flat_map(model::polygon_triangles)
                .collect::<Vec<_>>(),
        );
        plane
            .fills
            .insert(load.id, (load.clone(), triangles.clone()));
        triangles
    }
}
impl Perf {
    fn sample_fps(&mut self, now: f64, active: bool) -> bool {
        if let Some(last) = self.last_frame {
            let dt = now - last;
            if dt > 0. && dt < 0.35 {
                let sample = (1. / dt).min(60.);
                self.fps = if self.fps > 0. {
                    self.fps * 0.8 + sample * 0.2
                } else {
                    sample
                };
            } else {
                self.fps = 0.;
            }
        }
        self.last_frame = Some(now);
        if active {
            self.last_activity = now;
        }
        if now - self.last_activity >= 0.35 {
            self.fps = 0.;
        }
        active
    }
}
pub(super) struct ResultGeometry {
    pub ids: BTreeMap<u64, usize>,
    pub quad_by_display: BTreeMap<u64, usize>,
    pub quads: Vec<Option<crate::plan_trace::fem::mitc4::Quad4>>,
    bins: BTreeMap<(i64, i64), Vec<usize>>,
    large: Vec<usize>,
    step: f64,
}
impl ResultGeometry {
    pub fn candidates(&self, p: Point) -> Vec<usize> {
        let key = (
            (p[0] / self.step).floor() as i64,
            (p[1] / self.step).floor() as i64,
        );
        let mut candidates = self.bins.get(&key).cloned().unwrap_or_default();
        candidates.extend(&self.large);
        candidates.sort_unstable();
        candidates.dedup();
        candidates
    }
}
struct WorldResults {
    key: Vec<u64>,
    triangles: Vec<[(Point, Color32); 3]>,
    gpu: Arc<super::result_gpu::Geometry>,
}
struct Retained {
    key: Vec<u64>,
    mesh: Arc<egui::Mesh>,
}
impl App {
    pub(super) fn result_geometry(&self) -> Option<Arc<ResultGeometry>> {
        let mesh = self.fe_mesh.as_ref()?;
        if let Some((generation, geometry)) = self.perf.geometry.borrow().as_ref() {
            if *generation == self.mesh_generation {
                return Some(geometry.clone());
            }
        }
        let ids = mesh
            .elements
            .iter()
            .enumerate()
            .map(|(i, e)| (e.id, i))
            .collect();
        let quad_by_display = mesh
            .quads
            .iter()
            .enumerate()
            .flat_map(|(i, q)| q.display_triangles.map(|id| (id, i)))
            .collect();
        let quads = mesh
            .quads
            .iter()
            .map(|q| {
                crate::plan_trace::fem::mitc4::Quad4::new(q.nodes.map(|n| mesh.nodes[n].xy)).ok()
            })
            .collect();
        let step = self.mesh_built_size.max(1.);
        let mut bins = BTreeMap::<(i64, i64), Vec<usize>>::new();
        let mut large = vec![];
        for (i, e) in mesh.elements.iter().enumerate() {
            if !self.spatial.enabled && !self.mesh_element_on_floor(i) {
                continue;
            }
            let p = e.nodes.map(|n| mesh.nodes[n].xy);
            let lo: [i64; 2] = std::array::from_fn(|a| {
                (p.iter().map(|p| p[a]).fold(f64::INFINITY, f64::min) / step).floor() as i64
            });
            let hi: [i64; 2] = std::array::from_fn(|a| {
                (p.iter().map(|p| p[a]).fold(f64::NEG_INFINITY, f64::max) / step).floor() as i64
            });
            if (hi[0] - lo[0] + 1) as f64 * (hi[1] - lo[1] + 1) as f64 > 256. {
                large.push(i);
                continue;
            }
            for x in lo[0]..=hi[0] {
                for y in lo[1]..=hi[1] {
                    bins.entry((x, y)).or_default().push(i);
                }
            }
        }
        let geometry = Arc::new(ResultGeometry {
            ids,
            quad_by_display,
            quads,
            bins,
            large,
            step,
        });
        *self.perf.geometry.borrow_mut() = Some((self.mesh_generation, geometry.clone()));
        Some(geometry)
    }
    pub(super) fn result_label_positions(&self) -> Arc<Vec<(Point, f64)>> {
        let key = self.view_key();
        if let Some((old, positions)) = self.perf.label_values.borrow().as_ref() {
            if *old == key {
                return positions.clone();
            }
        }
        let (Some(mesh), Some(case), Some(geometry)) = (
            &self.fe_mesh,
            self.selected_result(),
            self.result_geometry(),
        ) else {
            return Arc::new(vec![]);
        };
        let nodal = self.smoothed_efforts();
        let factor = self.result_display_factor();
        let support_nodes = self.support_node_indices();
        let positions: Vec<(Point, f64)> = if self.result_field == 16 {
            self.support_distance_data().map_or_else(Vec::new, |data| {
                data.areas
                    .iter()
                    .filter_map(|(id, &area)| data.centers.get(id).map(|&point| (point, area)))
                    .collect()
            })
        } else if self.result_field == 15
            || crate::plan_trace::fem::displacement_field(self.result_field)
        {
            mesh.nodes
                .iter()
                .enumerate()
                .filter(|(i, _)| self.mesh_node_on_floor(*i))
                .map(|(i, n)| (n.xy, self.result_node_value(case, i)))
                .collect()
        } else if self.result_field == 6 {
            support_nodes
                .iter()
                .map(|&i| (mesh.nodes[i].xy, case.reaction_kn[i] * factor))
                .collect()
        } else {
            mesh.elements
                .iter()
                .enumerate()
                .filter(|(i, _)| self.mesh_element_on_floor(*i))
                .filter_map(|(i, e)| {
                    let xy = e.nodes.map(|n| mesh.nodes[n].xy);
                    let position = if let Some(quad) =
                        geometry.quad_by_display.get(&e.id).map(|&q| &mesh.quads[q])
                    {
                        if e.id != quad.display_triangles[0] {
                            return None;
                        }
                        quad.nodes.iter().fold([0., 0.], |p, &n| {
                            [
                                p[0] + mesh.nodes[n].xy[0] / 4.,
                                p[1] + mesh.nodes[n].xy[1] / 4.,
                            ]
                        })
                    } else {
                        [
                            xy.iter().map(|p| p[0] / 3.).sum(),
                            xy.iter().map(|p| p[1] / 3.).sum(),
                        ]
                    };
                    Some((
                        position,
                        nodal
                            .as_ref()
                            .map(|v| {
                                if let Some(quad) =
                                    geometry.quad_by_display.get(&e.id).map(|&q| &mesh.quads[q])
                                {
                                    super::super::isofields::quad_corner_values(
                                        mesh,
                                        quad,
                                        v,
                                        &geometry.ids,
                                    )
                                    .map(|v| v.iter().sum::<f64>() / 4.)
                                    .unwrap_or_else(|| v[i].iter().sum::<f64>() / 3.)
                                } else {
                                    v[i].iter().sum::<f64>() / 3.
                                }
                            })
                            .unwrap_or_else(|| self.result_element_value(case, i) * factor),
                    ))
                })
                .collect()
        };

        let positions = Arc::new(
            positions
                .into_iter()
                .filter(|(_, value)| value.is_finite())
                .collect::<Vec<_>>(),
        );
        *self.perf.label_values.borrow_mut() = Some((key, positions.clone()));
        positions
    }
    pub(super) fn smoothed_efforts(&self) -> Option<Arc<Vec<[f64; 3]>>> {
        if !self.iso_efforts || !self.result_has_element_field() {
            return None;
        }
        let case = self.selected_result()?;
        let mesh = self.fe_mesh.as_ref()?;
        let key = self.view_key();
        if let Some((old, values)) = self.perf.smoothed.borrow().as_ref() {
            if *old == key {
                return Some(values.clone());
            }
        }
        let slabs = self
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Slab)
            .map(|c| {
                let material = self.materials.get(&c.id).copied().unwrap_or_default();
                let outer = c
                    .mesh_boundary(self.mesh_built_size)
                    .unwrap_or_else(|_| c.points.clone());
                let holes = self
                    .contours
                    .iter()
                    .filter(|h| h.active_hole() && h.parent == Some(c.id))
                    .map(|h| {
                        h.mesh_boundary(self.mesh_built_size)
                            .unwrap_or_else(|_| h.points.clone())
                    })
                    .collect::<Vec<_>>();
                (
                    outer,
                    holes,
                    [
                        material.e_mpa.to_bits(),
                        material.nu.to_bits(),
                        material.thickness_mm.to_bits(),
                    ],
                )
            })
            .collect::<Vec<_>>();
        let materials = mesh
            .elements
            .iter()
            .enumerate()
            .map(|(index, e)| {
                if let Some(data) = &mesh.building {
                    let m = self
                        .materials
                        .get(&data.element_slabs[index])
                        .copied()
                        .unwrap_or_default();
                    return [m.e_mpa.to_bits(), m.nu.to_bits(), m.thickness_mm.to_bits()];
                }
                let xy = e.nodes.map(|n| mesh.nodes[n].xy);
                let p = std::array::from_fn(|a| xy.iter().map(|p| p[a] / 3.).sum());
                slabs
                    .iter()
                    .find(|(outer, holes, _)| {
                        geometry::inside(p, outer) && !holes.iter().any(|h| geometry::inside(p, h))
                    })
                    .map(|s| s.2)
                    .unwrap_or([0; 3])
            })
            .collect::<Vec<_>>();
        let fixed = self.support_node_indices();
        let mut segments = vec![];
        for c in self.contours.iter().filter(|c| c.kind == Kind::Support) {
            let mut p = c
                .mesh_boundary(self.mesh_built_size)
                .unwrap_or_else(|_| c.points.clone());
            if c.closed() && !p.is_empty() {
                p.push(p[0]);
            }
            segments.extend(p.windows(2).map(|p| [p[0], p[1]]));
        }
        let on = |p: Point, a: Point, b: Point| {
            let d = [b[0] - a[0], b[1] - a[1]];
            let len = d[0].hypot(d[1]);
            if len < 1e-12 {
                return false;
            }
            let t = ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / (len * len);
            (-1e-8..=1. + 1e-8).contains(&t)
                && ((p[0] - a[0]) * d[1] - (p[1] - a[1]) * d[0]).abs() / len < 1e-5
        };
        let barriers = mesh
            .edges
            .iter()
            .filter(|e| fixed.contains(&e[0]) && fixed.contains(&e[1]))
            .filter(|e| {
                segments
                    .iter()
                    .any(|&[a, b]| on(mesh.nodes[e[0]].xy, a, b) && on(mesh.nodes[e[1]].xy, a, b))
            })
            .map(|e| if e[0] < e[1] { *e } else { [e[1], e[0]] })
            .collect();
        let values = (0..mesh.elements.len())
            .map(|i| self.result_element_value(case, i) * self.result_display_factor())
            .collect::<Vec<_>>();
        let recovered = Arc::new(crate::plan_trace::isofields::recover(
            mesh, &values, &materials, &barriers,
        ));
        *self.perf.smoothed.borrow_mut() = Some((key, recovered.clone()));
        Some(recovered)
    }
    pub(super) fn view_key(&self) -> [u64; 8] {
        [
            self.result_generation,
            self.mesh_generation
                ^ (self.view_visibility.revision as u64).wrapping_mul(0xd6e8feb86659fd93)
                ^ if self.building.enabled {
                    self.building.active.wrapping_mul(0x9e3779b97f4a7c15)
                } else {
                    0
                },
            self.result_by_combinations as u64,
            if self.result_by_combinations {
                self.active_combination
            } else {
                self.active_load
            },
            self.combination_variant as u64,
            self.result_field as u64,
            self.result_display_factor().to_bits(),
            self.iso_efforts as u64
                | ((self.sum_reactions as u64) << 1)
                | ((self.iso_displacements as u64) << 2)
                | ((self.spatial.enabled as u64) << 3)
                | (((self.membrane_area && matches!(self.result_field, 12..=14)) as u64) << 4)
                | ((self.drawing_visibility.beam as u64) << 5)
                | ((self.drawing_visibility.columns as u64) << 6)
                | (if self.result_field == crate::plan_trace::fem::DISPLACEMENT_NORM_FIELD {
                    u64::from(self.displacement_axes) << 7
                } else {
                    0
                }),
        ]
    }
    fn screen_key(&self, rect: Rect, ppp: f32) -> Vec<u64> {
        vec![
            self.camera.center[0].to_bits(),
            self.camera.center[1].to_bits(),
            self.camera.zoom.to_bits(),
            self.camera.angle.to_bits(),
            rect.min.x.to_bits() as u64,
            rect.min.y.to_bits() as u64,
            rect.max.x.to_bits() as u64,
            rect.max.y.to_bits() as u64,
            ppp.to_bits() as u64,
        ]
    }
    pub(super) fn area_load_visible(&self, i: usize) -> bool {
        if self.spatial.enabled {
            !self.mesh_element_hidden(i)
        } else {
            self.mesh_element_on_floor(i)
        }
    }
    pub(super) fn result_range_cached(&self) -> Option<[f64; 2]> {
        let case = self.selected_result()?;
        let key = self.view_key();
        if let Some((old, range, _)) = *self.perf.range.borrow() {
            if old == key {
                return Some(range);
            }
        }
        if (17..=22).contains(&self.result_field) {
            let section_scene = if self.spatial.enabled && self.sections.active() {
                Some(self.spatial_mesh_scene()?)
            } else {
                None
            };
            let values = self.beam_field_values(section_scene);
            let extrema = values
                .iter()
                .fold([f64::INFINITY, f64::NEG_INFINITY], |a, &v| {
                    [a[0].min(v), a[1].max(v)]
                });
            let extrema = if extrema[0].is_finite() {
                extrema
            } else {
                [0., 0.]
            };
            let range = crate::plan_trace::result_scale::range(extrema.into_iter());
            *self.perf.range.borrow_mut() = Some((key, range, extrema));
            return Some(range);
        }
        let nodal = self.smoothed_efforts();
        let loads = self.load_mosaic_shapes();
        let reactions = if self.result_field == 6 && self.sum_reactions {
            Some(self.support_reaction_totals())
        } else {
            None
        };
        let mut values: Box<dyn Iterator<Item = f64> + '_> =
            if self.iso_efforts && self.result_has_element_field() {
                Box::new(
                    nodal
                        .as_ref()?
                        .iter()
                        .enumerate()
                        .filter(|(i, _)| self.spatial.enabled || self.mesh_element_on_floor(*i))
                        .flat_map(|(_, v)| v.iter().copied()),
                )
            } else if self.result_field == 7 {
                let factor = self.force_result_factor();
                Box::new(
                    case.loads_kn_m2
                        .iter()
                        .enumerate()
                        .filter(|(i, _)| self.area_load_visible(*i))
                        .map(move |(_, value)| value * factor),
                )
            } else if matches!(self.result_field, 8 | 9) {
                Box::new(loads.iter().map(|s| s.value))
            } else if self.result_field == 15
                || crate::plan_trace::fem::displacement_field(self.result_field)
            {
                Box::new(
                    (0..case.w_mm.len())
                        .filter(|&i| self.spatial.enabled || self.mesh_node_on_floor(i))
                        .map(|i| self.result_node_value(case, i)),
                )
            } else if self.result_field == 6 && self.sum_reactions {
                Box::new(reactions.as_ref()?.iter().map(|r| r.value))
            } else if self.result_field == 6 {
                Box::new(
                    self.support_node_indices()
                        .into_iter()
                        .map(|i| case.reaction_kn[i] * self.result_display_factor()),
                )
            } else {
                Box::new(
                    (0..case.moments.len())
                        .filter(|&i| self.spatial.enabled || self.mesh_element_on_floor(i))
                        .map(|i| self.result_element_value(case, i) * self.result_display_factor()),
                )
            };
        let mut extrema = [f64::INFINITY, f64::NEG_INFINITY];
        for v in &mut values {
            extrema[0] = extrema[0].min(v);
            extrema[1] = extrema[1].max(v);
        }
        if !extrema[0].is_finite() {
            extrema = [0., 0.];
        }
        let range = crate::plan_trace::result_scale::range(extrema.into_iter());
        *self.perf.range.borrow_mut() = Some((key, range, extrema));
        Some(range)
    }
    pub(super) fn result_extrema_cached(&self) -> Option<[f64; 2]> {
        if self.spatial.enabled && !matches!(self.result_field, 7..=9 | 17..=22) {
            return self.spatial_result_extrema();
        }
        self.result_range_cached()?;
        self.perf.range.borrow().as_ref().map(|c| c.2)
    }
    pub(super) fn invalidate_scene_buffers(&self) {
        self.spatial.invalidate_load_geometry();
        *self.perf.edges.borrow_mut() = None;
        *self.perf.results.borrow_mut() = None;
        *self.perf.result_world.borrow_mut() = None;
        *self.perf.range.borrow_mut() = None;
        *self.perf.smoothed.borrow_mut() = None;
        *self.perf.label_values.borrow_mut() = None;
        *self.perf.geometry.borrow_mut() = None;
        *self.perf.reactions.borrow_mut() = None;
    }
    pub(super) fn paint_cached_edges(&self, painter: &egui::Painter, rect: Rect) {
        let Some(mesh) = &self.fe_mesh else {
            return;
        };
        let ppp = painter.ctx().pixels_per_point();
        let mut key = self.screen_key(rect, ppp);
        key.push(self.mesh_generation);
        let mut slot = self.perf.edges.borrow_mut();
        if slot.as_ref().is_none_or(|c| c.key != key) {
            let mut out = egui::Mesh::default();
            let fragment_edges = self.fe_fragment_active().then(|| {
                mesh.elements
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| self.mesh_element_on_floor(*i))
                    .flat_map(|(_, e)| {
                        (0..3).map(|i| {
                            let [a, b] = [e.nodes[i], e.nodes[(i + 1) % 3]];
                            [a.min(b), a.max(b)]
                        })
                    })
                    .collect::<std::collections::BTreeSet<_>>()
            });
            for [a, b] in mesh.visible_edges() {
                if fragment_edges
                    .as_ref()
                    .is_some_and(|edges| !edges.contains(&[a, b]))
                    || !self.mesh_node_on_floor(a)
                    || !self.mesh_node_on_floor(b)
                {
                    continue;
                }
                if let Some([a, b]) = clip_line(
                    self.camera.screen64(mesh.nodes[a].xy, rect),
                    self.camera.screen64(mesh.nodes[b].xy, rect),
                    rect.expand(1.),
                ) {
                    crate::plan_trace::render_cache::line(
                        &mut out,
                        a,
                        b,
                        0.8,
                        Color32::from_rgb(45, 70, 85),
                        ppp,
                    );
                }
            }
            *slot = Some(Retained {
                key,
                mesh: Arc::new(out),
            });
        }
        if let Some(c) = slot.as_ref() {
            crate::plan_trace::render_cache::paint(painter, rect, c.mesh.clone());
        }
    }
    pub(super) fn paint_cached_results(&self, painter: &egui::Painter, rect: Rect) {
        if !self.show_results
            || self.point_displacements()
            || self.result_field == 6
            || matches!(self.result_field, 8 | 9)
        {
            return;
        }
        let (Some(mesh), Some(case), Some(auto)) = (
            &self.fe_mesh,
            self.selected_result(),
            self.result_range_cached(),
        ) else {
            return;
        };
        let scale = &self.result_scales[self.result_field];
        let nodal = self.smoothed_efforts();
        let mut world_key = self.view_key().to_vec();
        world_key.extend(scale.signature());
        let mut world = self.perf.result_world.borrow_mut();
        if world.as_ref().is_none_or(|c| c.key != world_key) {
            let mut triangles: Vec<[(Point, Color32); 3]> = vec![];
            let uses_nodal_values = self.result_field == 15
                || crate::plan_trace::fem::displacement_field(self.result_field);
            let quads = if uses_nodal_values || nodal.is_some() {
                mesh.quads
                    .iter()
                    .flat_map(|q| q.display_triangles.into_iter().map(move |id| (id, q)))
                    .collect::<BTreeMap<_, _>>()
            } else {
                BTreeMap::new()
            };
            let display_ids = if !quads.is_empty() && nodal.is_some() {
                mesh.elements
                    .iter()
                    .enumerate()
                    .map(|(i, e)| (e.id, i))
                    .collect::<BTreeMap<_, _>>()
            } else {
                BTreeMap::new()
            };
            for (i, e) in mesh.elements.iter().enumerate() {
                if !self.mesh_element_on_floor(i) {
                    continue;
                }
                let xy = e.nodes.map(|n| mesh.nodes[n].xy);
                if uses_nodal_values || nodal.is_some() {
                    if let Some(quad) = quads.get(&e.id) {
                        if e.id != quad.display_triangles[0] {
                            continue;
                        }
                        let Ok(element) = crate::plan_trace::fem::mitc4::Quad4::new(
                            quad.nodes.map(|n| mesh.nodes[n].xy),
                        ) else {
                            continue;
                        };
                        let corners = if uses_nodal_values {
                            quad.nodes.map(|n| self.result_node_value(case, n))
                        } else {
                            let Some(v) = super::super::isofields::quad_corner_values(
                                mesh,
                                quad,
                                nodal.as_ref().unwrap(),
                                &display_ids,
                            ) else {
                                continue;
                            };
                            v
                        };
                        // Graphics tessellation samples the real bilinear Q4 field.
                        // It does not create any finite elements or integration cells.
                        let steps = 4;
                        for row in 0..steps {
                            for col in 0..steps {
                                let r = -1. + 2. * col as f64 / steps as f64;
                                let s = -1. + 2. * row as f64 / steps as f64;
                                let h = 2. / steps as f64;
                                let sample =
                                    [[r, s], [r + h, s], [r + h, s + h], [r, s + h]].map(|p| {
                                        let (n, _) = crate::plan_trace::fem::mitc4::shape(p);
                                        (
                                            element.position(p),
                                            (0..4).map(|j| n[j] * corners[j]).sum::<f64>(),
                                        )
                                    });
                                for t in [[0, 1, 2], [0, 2, 3]] {
                                    for tri in scale.triangles(
                                        t.map(|j| sample[j].0),
                                        t.map(|j| sample[j].1),
                                        auto,
                                    ) {
                                        triangles.push(tri);
                                    }
                                }
                            }
                        }
                        continue;
                    }
                }
                if uses_nodal_values {
                    for tri in
                        scale.triangles(xy, e.nodes.map(|n| self.result_node_value(case, n)), auto)
                    {
                        triangles.push(tri);
                    }
                } else if let Some(nodal) = &nodal {
                    for tri in scale.triangles(xy, nodal[i], auto) {
                        triangles.push(tri);
                    }
                } else if let Some(c) = scale.color(
                    self.result_element_value(case, i) * self.result_display_factor(),
                    auto,
                ) {
                    triangles.push(xy.map(|p| (p, c)));
                }
            }
            let gpu = Arc::new(super::result_gpu::Geometry::new(&triangles));
            *world = Some(WorldResults {
                key: world_key.clone(),
                triangles,
                gpu,
            });
        }
        if super::result_gpu::paint(
            painter,
            rect,
            world.as_ref().unwrap().gpu.clone(),
            self.camera,
        ) {
            return;
        }
        // The CPU-only test/backend fallback still clips before f32 conversion.
        let mut key = self.screen_key(rect, painter.ctx().pixels_per_point());
        key.extend(world_key);
        let mut slot = self.perf.results.borrow_mut();
        if slot.as_ref().is_none_or(|c| c.key != key) {
            let mut out = egui::Mesh::default();
            for &tri in &world.as_ref().unwrap().triangles {
                append_colored(
                    &mut out,
                    tri.map(|(p, c)| (self.camera.screen64(p, rect), c)),
                    rect,
                );
            }
            *slot = Some(Retained {
                key,
                mesh: Arc::new(out),
            });
        }
        if let Some(c) = slot.as_ref() {
            crate::plan_trace::render_cache::paint(painter, rect, c.mesh.clone());
        }
    }
    pub(super) fn pace_frame(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(deadline) = self.perf.next_frame {
            while let Some(wait) = deadline.checked_duration_since(Instant::now()) {
                std::thread::sleep(wait);
            }
        }
        self.perf.next_frame = Some(Instant::now() + Duration::from_nanos(16_666_667));
    }
    pub(super) fn tick_fps(&mut self, ctx: &egui::Context) {
        let now = self.started.elapsed().as_secs_f64();
        let foreground = ctx.input(|i| {
            i.viewport().focused.unwrap_or(true) && !i.viewport().minimized.unwrap_or(false)
        });
        let active = foreground
            && (ctx.input(|i| !i.events.is_empty() || i.pointer.any_down())
                || (self.spatial.enabled
                    && ctx
                        .data(|d| d.get_temp::<fem_ui::ModalView>(egui::Id::new("modal_view")))
                        .is_some_and(|v| {
                            v.playing
                                && v.mesh_generation == self.mesh_generation
                                && v.result_generation == self.result_generation
                                && v.stiffness_variant == self.stiffness_variant
                                && v.paused_at.is_none()
                        })));
        if self.perf.sample_fps(now, active) {
            ctx.request_repaint_after(Duration::from_millis(400));
        }
    }

    pub(super) fn draw_fps(&self, painter: &egui::Painter, rect: Rect) {
        let text = if self.perf.fps > 0. {
            format!("FPS: {:.0}", self.perf.fps)
        } else {
            "FPS: 0 · покой".into()
        };
        let galley = painter.layout_no_wrap(text, egui::FontId::monospace(12.), Color32::WHITE);
        let box_ = Rect::from_min_size(
            rect.right_top() - egui::vec2(galley.size().x + 8., 0.),
            galley.size() + egui::vec2(8., 4.),
        );
        painter.rect_filled(box_, 2., Color32::from_rgba_unmultiplied(20, 24, 30, 235));
        painter.galley(box_.min + egui::vec2(4., 2.), galley, Color32::WHITE);
    }
}
// Clip before conversion to f32; interpolate vertex colors at viewport edges.
fn append_colored(out: &mut egui::Mesh, tri: [(Point, Color32); 3], rect: Rect) {
    if tri.iter().any(|(p, _)| p.iter().any(|x| !x.is_finite())) {
        return;
    }
    let mut p = tri
        .into_iter()
        .map(|(p, c)| (p, c.to_array().map(|x| x as f64)))
        .collect::<Vec<_>>();
    for (axis, edge, above) in [
        (0, rect.left() as f64, true),
        (0, rect.right() as f64, false),
        (1, rect.top() as f64, true),
        (1, rect.bottom() as f64, false),
    ] {
        if p.is_empty() {
            return;
        }
        let inside = |p: Point| {
            if above {
                p[axis] >= edge
            } else {
                p[axis] <= edge
            }
        };
        let mut next = vec![];
        let mut a = *p.last().unwrap();
        for b in p {
            if inside(a.0) != inside(b.0) {
                let t = (edge - a.0[axis]) / (b.0[axis] - a.0[axis]);
                next.push((
                    std::array::from_fn(|j| a.0[j] + t * (b.0[j] - a.0[j])),
                    std::array::from_fn(|j| a.1[j] + t * (b.1[j] - a.1[j])),
                ));
            }
            if inside(b.0) {
                next.push(b);
            }
            a = b;
        }
        p = next;
    }
    if p.len() < 3 {
        return;
    }
    let start = out.vertices.len() as u32;
    for (p, c) in &p {
        let c = c.map(|v| v.round().clamp(0., 255.) as u8);
        out.colored_vertex(
            egui::pos2(p[0] as f32, p[1] as f32),
            Color32::from_rgba_premultiplied(c[0], c[1], c[2], c[3]),
        );
    }
    for i in 1..p.len() - 1 {
        out.add_triangle(start, start + i as u32, start + i as u32 + 1);
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn load_fill_reuses_geometry_for_camera_and_invalidates_for_holes() {
        use super::*;
        let mut app = App::default();
        for (kind, a, b) in [
            (Kind::Slab, [0., 0.], [1000., 1000.]),
            (Kind::Hole, [100., 100.], [200., 200.]),
            (Kind::LoadArea, [-100., -100.], [1100., 1100.]),
        ] {
            app.begin(kind);
            app.choose_tool(Tool::Rectangle);
            app.sketch_click(a, false);
            app.sketch_click(b, false);
        }
        let load = app
            .contours
            .iter()
            .find(|c| c.kind == Kind::LoadArea)
            .unwrap()
            .clone();
        let first = app.cached_load_fill(&load);
        assert!(!first.is_empty());
        app.camera.zoom *= 2.;
        app.camera.center = [123., 456.];
        app.start_load_fill_frame();
        assert!(Arc::ptr_eq(&first, &app.cached_load_fill(&load)));
        app.contours
            .iter_mut()
            .find(|c| c.kind == Kind::Hole)
            .unwrap()
            .translate([20., 0.])
            .unwrap();
        app.start_load_fill_frame();
        let after_hole_edit = app.cached_load_fill(&load);
        assert!(!Arc::ptr_eq(&first, &after_hole_edit));

        let mut changed_load = load.clone();
        changed_load.translate([200., 0.]).unwrap();
        let after_load_edit = app.cached_load_fill(&changed_load);
        assert!(!Arc::ptr_eq(&after_hole_edit, &after_load_edit));

        app.building.enabled = true;
        app.building.floors.push(building::Floor {
            id: 2,
            name: "Этаж 2".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        app.building.contour_floors.insert(load.id, 2);
        app.start_load_fill_frame();
        assert!(app.cached_load_fill(&load).is_empty());
        let mut shifted_to_first_floor = load;
        shifted_to_first_floor.placement.slab.offset_mm = -3000.;
        assert!(!app.cached_load_fill(&shifted_to_first_floor).is_empty());
    }
    use super::*;
    #[test]
    fn reaction_range_uses_only_support_nodes_and_converts_force_units() {
        use crate::plan_trace::{
            fem::{CaseResult, Results},
            meshing::{Mesh, Node},
        };
        let mut a = App::default();
        a.fe_mesh = Some(Mesh {
            building: None,
            quads: vec![],
            constrained_nodes: Default::default(),
            constrained_edges: Default::default(),
            nodes: vec![
                Node {
                    id: 1,
                    xy: [0., 0.],
                },
                Node {
                    id: 2,
                    xy: [1., 0.],
                },
            ],
            elements: vec![],
            edges: vec![],
            support_boundary: BTreeMap::from([(9, vec![0])]),
        });
        a.analysis = Some(Results {
            nonlinear: None,
            lengths: None,
            modal: None,
            buckling: None,
            free_dofs: 1,
            cases: vec![CaseResult {
                punching: Default::default(),
                shell: None,
                id: 1,
                w_mm: vec![0., 2.],
                rotations_rad: vec![[0.; 3]; 0],
                moments: vec![],
                shears: vec![],
                reaction_kn: vec![-20., 0.],
                loads_kn_m2: vec![],
                foundation_pressure_kn_m2: vec![],
                applied_kn: 20.,
                balance_kn: 0.,
                balance_m_knm: [0.; 2],
                relative_residual: 0.,
            }],
        });
        a.result_field = 6;
        assert_eq!(a.result_extrema_cached().unwrap(), [-2., -2.]);
        a.load_units = crate::plan_trace::loads::ForceUnits::KiloNewton;
        a.sync_result_units();
        assert_eq!(a.result_extrema_cached().unwrap(), [-20., -20.]);
        assert_eq!(a.selected_result().unwrap().reaction_kn, vec![-20., 0.]);
    }
    #[test]
    fn frame_pacer_enforces_a_minimum_frame_interval_without_an_idle_timer() {
        let mut app = App::default();
        app.pace_frame();
        let first = Instant::now();
        app.pace_frame();
        assert!(first.elapsed() >= Duration::from_millis(16));
    }
    #[test]
    fn fps_idle_notification_does_not_start_a_repaint_loop() {
        let mut perf = Perf::default();
        assert!(perf.sample_fps(0., true));
        assert!(perf.sample_fps(0.016, true));
        assert!(perf.fps > 50.);
        assert!(!perf.sample_fps(0.416, false));
        assert_eq!(perf.fps, 0.);
        assert!(!perf.sample_fps(0.9, false));
        assert_eq!(perf.fps, 0.);
    }
    #[test]
    fn retained_triangle_clipping_is_finite_and_empty_safe() {
        let rect = Rect::from_min_max(egui::pos2(0., 0.), egui::pos2(100., 100.));
        let mut mesh = egui::Mesh::default();
        append_colored(&mut mesh, [([f64::NAN, 0.], Color32::WHITE); 3], rect);
        assert!(mesh.is_empty());
        append_colored(
            &mut mesh,
            [
                ([-1e9, -1e9], Color32::RED),
                ([1e9, -1e9], Color32::GREEN),
                ([0., 1e9], Color32::BLUE),
            ],
            rect,
        );
        assert!(!mesh.is_empty());
        assert!(mesh.is_valid());
        assert!(mesh
            .vertices
            .iter()
            .all(|v| v.pos.is_finite() && rect.expand(0.01).contains(v.pos)));
        let area = mesh
            .indices
            .chunks_exact(3)
            .map(|i| {
                let a = mesh.vertices[i[0] as usize].pos;
                let b = mesh.vertices[i[1] as usize].pos;
                let c = mesh.vertices[i[2] as usize].pos;
                ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)).abs() * 0.5
            })
            .sum::<f32>();
        assert!((area - 10000.).abs() < 0.1);
    }
}
