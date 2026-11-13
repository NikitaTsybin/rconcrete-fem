//! Named shared physical properties. Contours reference stable property IDs.
use super::{
    fem::element::Material,
    model::{Contour, Kind},
};
use std::collections::{BTreeMap, BTreeSet};
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Family {
    Slab,
    Wall,
    Beam,
    Column,
}
impl Family {
    pub fn section_family(self) -> Self {
        match self {
            Self::Wall => Self::Slab,
            family => family,
        }
    }

    pub fn is_plate(self) -> bool {
        self.section_family() == Self::Slab
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Slab => "Плита",
            Self::Wall => "Стена",
            Self::Beam => "Балка",
            Self::Column => "Колонна",
        }
    }
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Entry {
    pub name: String,
    pub family: Family,
    pub material: Material,
    #[serde(default)]
    pub reinforcement: super::reinforcement::Settings,
    #[serde(default = "default_beam_width")]
    pub width_mm: f64,
}
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ElasticMaterial {
    pub e_mpa: f64,
    pub nu: f64,
    pub rho: super::loads::Magnitude,
}
impl ElasticMaterial {
    fn from_section(material: Material) -> Self {
        Self {
            e_mpa: material.e_mpa,
            nu: material.nu,
            rho: material.weight,
        }
    }
    fn apply(self, material: &mut Material) {
        material.e_mpa = self.e_mpa;
        material.nu = self.nu;
        material.weight = self.rho;
    }
}
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DatabaseConcrete {
    pub class: usize,
    pub factor: f64,
    pub alternative_factor: f64,
}
impl Default for DatabaseConcrete {
    fn default() -> Self {
        Self {
            class: super::reinforcement::Settings::default().concrete,
            factor: 1.,
            alternative_factor: 1.,
        }
    }
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct NamedMaterial {
    pub name: String,
    pub primary: ElasticMaterial,
    pub alternative: ElasticMaterial,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub concrete: Option<DatabaseConcrete>,
}
impl NamedMaterial {
    fn resolve(&self, alternative: bool) -> ElasticMaterial {
        let custom = if alternative {
            self.alternative
        } else {
            self.primary
        };
        let Some(concrete) = self.concrete else {
            return custom;
        };
        let classes = &super::reinforcement::CONCRETE;
        let (_, _, initial, _, _) = classes[concrete.class.min(classes.len() - 1)];
        let factor = if alternative {
            concrete.alternative_factor
        } else {
            concrete.factor
        };
        ElasticMaterial {
            e_mpa: initial * factor,
            nu: 0.2,
            rho: super::loads::Magnitude {
                value: 2.5,
                units: super::loads::ForceUnits::TonForce,
            },
        }
    }
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Design {
    pub name: String,
    pub bar: bool,
    pub settings: super::reinforcement::Settings,
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Library {
    #[serde(default)]
    pub alternative_enabled: bool,
    #[serde(default)]
    pub alternatives: BTreeMap<u64, Entry>,
    #[serde(default)]
    pub colors: BTreeMap<u64, [u8; 3]>,
    #[serde(default)]
    pub elastic: BTreeMap<u64, NamedMaterial>,
    #[serde(default)]
    pub elastic_assignments: BTreeMap<u64, u64>,
    #[serde(default)]
    pub contour_materials: BTreeMap<u64, u64>,
    #[serde(default)]
    pub material_colors: BTreeMap<u64, [u8; 3]>,
    #[serde(skip)]
    material_variant: usize,
    #[serde(default)]
    pub designs: BTreeMap<u64, Design>,
    #[serde(default)]
    pub design_assignments: BTreeMap<u64, u64>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub contour_designs: BTreeMap<u64, u64>,
    pub entries: BTreeMap<u64, Entry>,
    pub assignments: BTreeMap<u64, u64>,
    pub next_id: u64,
}
impl Default for Library {
    fn default() -> Self {
        Self {
            alternative_enabled: false,
            alternatives: BTreeMap::new(),
            colors: BTreeMap::new(),
            elastic: BTreeMap::new(),
            elastic_assignments: BTreeMap::new(),
            contour_materials: BTreeMap::new(),
            material_colors: BTreeMap::new(),
            material_variant: 0,
            designs: BTreeMap::new(),
            design_assignments: BTreeMap::new(),
            contour_designs: BTreeMap::new(),
            entries: BTreeMap::from([
                (
                    1,
                    Entry {
                        reinforcement: Default::default(),
                        width_mm: 200.,
                        name: "Плита 1".into(),
                        family: Family::Slab,
                        material: Material::default(),
                    },
                ),
                (
                    2,
                    Entry {
                        reinforcement: Default::default(),
                        width_mm: 200.,
                        name: "Стена 1".into(),
                        family: Family::Wall,
                        material: Material::wall_default(),
                    },
                ),
                (
                    3,
                    Entry {
                        reinforcement: Default::default(),
                        name: "Балка 1".into(),
                        family: Family::Beam,
                        material: beam_material(),
                        width_mm: 200.,
                    },
                ),
                (
                    4,
                    Entry {
                        name: "Колонна 1".into(),
                        family: Family::Column,
                        width_mm: 250.,
                        material: column_material(),
                        reinforcement: Default::default(),
                    },
                ),
            ]),
            assignments: BTreeMap::new(),
            next_id: 5,
        }
    }
}
impl Library {
    pub fn split_legacy_entries(&mut self) {
        let missing = self
            .assignments
            .iter()
            .filter_map(|(&contour, &id)| {
                self.material_id(contour).is_none().then_some((contour, id))
            })
            .collect::<Vec<_>>();
        for (contour, id) in missing {
            let Some(entry) = self.entries.get(&id) else {
                continue;
            };
            let mat = if let Some(mat) = self
                .elastic_assignments
                .get(&id)
                .copied()
                .filter(|m| self.elastic.contains_key(m))
            {
                mat
            } else {
                let primary = ElasticMaterial::from_section(entry.material);
                let alternative = self
                    .alternatives
                    .get(&id)
                    .map_or(primary, |alt| ElasticMaterial::from_section(alt.material));
                let mat = self
                    .elastic
                    .iter()
                    .find(|(_, m)| {
                        m.concrete.is_none() && m.primary == primary && m.alternative == alternative
                    })
                    .map(|(&id, _)| id);
                mat.unwrap_or_else(|| {
                    let id = self.elastic.keys().next_back().copied().unwrap_or(0) + 1;
                    self.elastic.insert(
                        id,
                        NamedMaterial {
                            name: format!("Материал {id}"),
                            primary,
                            alternative,
                            concrete: None,
                        },
                    );
                    id
                })
            };
            self.contour_materials.insert(contour, mat);
        }
        for (&id, entry) in &mut self.entries {
            let bar = matches!(entry.family, Family::Beam | Family::Column);
            if !self
                .design_assignments
                .get(&id)
                .is_some_and(|d| self.designs.get(d).is_some_and(|d| d.bar == bar))
            {
                let design = self
                    .designs
                    .iter()
                    .find(|(_, d)| d.bar == bar && d.settings == entry.reinforcement)
                    .map(|(&id, _)| id);
                let design = design.unwrap_or_else(|| {
                    let id = self.designs.keys().next_back().copied().unwrap_or(0) + 1;
                    self.designs.insert(
                        id,
                        Design {
                            name: format!("Конструирование {id}"),
                            bar,
                            settings: entry.reinforcement,
                        },
                    );
                    id
                });
                self.design_assignments.insert(id, design);
            }
            entry.reinforcement = self.designs[&self.design_assignments[&id]].settings;
        }
    }
    pub fn for_variant(&self, variant: usize) -> Self {
        let mut out = self.clone();
        out.material_variant = variant;
        out
    }
    pub fn material_id(&self, contour: u64) -> Option<u64> {
        self.contour_materials
            .get(&contour)
            .copied()
            .filter(|id| self.elastic.contains_key(id))
    }
    pub fn design_id(&self, contour: u64) -> Option<u64> {
        let section = self.assignments.get(&contour)?;
        self.contour_designs
            .get(&contour)
            .or_else(|| self.design_assignments.get(section))
            .copied()
            .filter(|id| self.designs.contains_key(id))
    }
    pub fn resolved_entry(&self, contour: u64) -> Option<Entry> {
        self.resolved_entry_for(contour, self.material_variant)
    }
    pub fn resolved_entry_for(&self, contour: u64, variant: usize) -> Option<Entry> {
        let id = self.assignments.get(&contour)?;
        let mut entry = self.entries.get(id)?.clone();
        if entry.family.is_plate() {
            if let Some(design) = self.design_id(contour).and_then(|id| self.designs.get(&id)) {
                entry.reinforcement = design.settings;
            }
        }
        let alternative = variant == 1 && self.alternative_enabled;
        if let Some(m) = self
            .material_id(contour)
            .or_else(|| self.elastic_assignments.get(id).copied())
            .and_then(|m| self.elastic.get(&m))
        {
            m.resolve(alternative).apply(&mut entry.material);
            if let Some(concrete) = m.concrete {
                entry.reinforcement.concrete =
                    concrete.class.min(super::reinforcement::CONCRETE.len() - 1);
            }
        } else if alternative {
            if let Some(alt) = self.alternatives.get(id) {
                ElasticMaterial::from_section(alt.material).apply(&mut entry.material);
            }
        }
        Some(entry)
    }
    pub fn import_named_material(
        &mut self,
        material: NamedMaterial,
        color: Option<[u8; 3]>,
    ) -> u64 {
        if let Some((&id, _)) = self
            .elastic
            .iter()
            .find(|(id, m)| **m == material && self.material_colors.get(id).copied() == color)
        {
            return id;
        }
        let id = self.elastic.keys().next_back().copied().unwrap_or(0) + 1;
        self.elastic.insert(id, material);
        if let Some(color) = color {
            self.material_colors.insert(id, color);
        }
        id
    }
    pub fn remove_unused_material(&mut self, id: u64) -> bool {
        if self.contour_materials.values().any(|&m| m == id) || self.elastic.remove(&id).is_none() {
            return false;
        }
        self.elastic_assignments.retain(|_, m| *m != id);
        self.material_colors.remove(&id);
        true
    }

    pub fn add(&mut self, family: Family, material: Material) -> u64 {
        let mut number = 1;
        let name = loop {
            let name = format!("{} {number}", family.label());
            if !self.entries.values().any(|e| e.name == name) {
                break name;
            }
            number += 1;
        };
        let id = self
            .next_id
            .max(self.entries.keys().next_back().copied().unwrap_or(0) + 1);
        self.next_id = id + 1;
        self.entries.insert(
            id,
            Entry {
                reinforcement: Default::default(),
                width_mm: if family == Family::Column { 250. } else { 200. },
                name,
                family,
                material,
            },
        );
        id
    }
    pub fn reconcile(
        &mut self,
        contours: &[Contour],
        slabs: &BTreeMap<u64, Material>,
        walls: &BTreeMap<u64, Material>,
    ) {
        let column_default = self
            .entries
            .iter()
            .find(|(_, e)| e.family == Family::Column)
            .map(|(&id, _)| id)
            .unwrap_or_else(|| self.add(Family::Column, column_material()));
        let beam_default = self
            .entries
            .iter()
            .find(|(_, e)| e.family == Family::Beam)
            .map(|(&id, _)| id)
            .unwrap_or_else(|| self.add(Family::Beam, beam_material()));
        for c in contours.iter().filter(|c| {
            matches!(
                c.kind,
                Kind::Slab | Kind::Support | Kind::Beam | Kind::Column
            )
        }) {
            if self
                .assignments
                .get(&c.id)
                .is_some_and(|id| self.entries.contains_key(id))
            {
                continue;
            }
            let (family, legacy, default_id) = if c.kind == Kind::Column {
                (Family::Column, None, column_default)
            } else if c.kind == Kind::Beam {
                (Family::Beam, None, beam_default)
            } else if c.kind == Kind::Slab {
                (Family::Slab, slabs.get(&c.id), 1)
            } else {
                (Family::Wall, walls.get(&c.id), 2)
            };
            let id = if let Some(&material) = legacy {
                let existing = self
                    .entries
                    .iter()
                    .find(|(_, e)| e.family == family && e.material == material)
                    .map(|(&id, _)| id);
                existing.unwrap_or_else(|| self.add(family, material))
            } else {
                default_id
            };
            self.assignments.insert(c.id, id);
        }
    }
    pub fn resolve(&self, contours: &[Contour], kind: Kind) -> BTreeMap<u64, Material> {
        contours
            .iter()
            .filter(|c| c.kind == kind)
            .map(|c| {
                let default = if kind == Kind::Support {
                    Material::wall_default()
                } else {
                    Material::default()
                };
                let material = self.resolved_entry(c.id).map_or(default, |e| e.material);
                (c.id, material)
            })
            .collect()
    }
    pub fn snapshot(&self, contours: &[Contour]) -> Self {
        let ids = contours.iter().map(|c| c.id).collect::<BTreeSet<_>>();
        let mut out = self.clone();
        out.assignments.retain(|id, _| ids.contains(id));
        out.contour_materials.retain(|id, _| ids.contains(id));
        if !out.contour_designs.is_empty() {
            let plates = contours
                .iter()
                .filter(|c| matches!(c.kind, Kind::Slab | Kind::Support))
                .map(|c| c.id)
                .collect::<BTreeSet<_>>();
            out.contour_designs.retain(|id, _| plates.contains(id));
        }
        out
    }
    pub fn validate(&self, contours: &[Contour]) -> Result<(), String> {
        if self.entries.len() < 2
            || self.entries.len() > 10000
            || !self.entries.contains_key(&1)
            || !self.entries.contains_key(&2)
            || self.entries.contains_key(&0)
            || self.entries.contains_key(&u64::MAX)
            || self.next_id == u64::MAX
            || self.next_id <= *self.entries.keys().next_back().unwrap()
        {
            return Err("Некорректный список жёсткостей".into());
        }
        for record in self.elastic.values() {
            for elastic in [record.primary, record.alternative] {
                let mut material = Material::default();
                elastic.apply(&mut material);
                material.validate()?;
            }
        }
        for record in self.designs.values() {
            record.settings.validate_source()?;
        }
        for e in self.entries.values() {
            if e.name.trim().is_empty() || e.name.len() > 1024 {
                return Err("Введите имя жёсткости (до 1024 байт)".into());
            }
            e.material.validate()?;
            if e.family == Family::Slab {
                e.reinforcement.validate_source()?;
            }
            if matches!(e.family, Family::Beam | Family::Column)
                && (!e.width_mm.is_finite() || !(1. ..=10000.).contains(&e.width_mm))
            {
                return Err("Некорректная ширина сечения балки".into());
            }
        }
        for (&id, e) in &self.alternatives {
            let Some(base) = self.entries.get(&id) else {
                return Err("Альтернативная жёсткость без основного набора".into());
            };
            if base.family != e.family {
                return Err("Тип альтернативной жёсткости отличается от основного".into());
            }
            e.material.validate()?;
            if matches!(e.family, Family::Beam | Family::Column)
                && (!e.width_mm.is_finite() || !(1. ..=10000.).contains(&e.width_mm))
            {
                return Err("Некорректная альтернативная ширина балки".into());
            }
        }
        for (&id, &property) in &self.assignments {
            if !self.entries.contains_key(&property)
                || !contours.iter().any(|c| {
                    c.id == id
                        && matches!(
                            c.kind,
                            Kind::Slab | Kind::Support | Kind::Beam | Kind::Column
                        )
                })
            {
                return Err("Жёсткость ссылается на неизвестный объект или набор".into());
            }
        }
        if self.contour_materials.iter().any(|(id, material)| {
            !self.assignments.contains_key(id) || !self.elastic.contains_key(material)
        }) {
            return Err("Материал ссылается на неизвестный объект или набор".into());
        }
        if self.contour_designs.iter().any(|(&id, design)| {
            !self.designs.get(design).is_some_and(|d| !d.bar)
                || !contours
                    .iter()
                    .any(|c| c.id == id && matches!(c.kind, Kind::Slab | Kind::Support))
        }) {
            return Err(
                "Параметры конструирования ссылаются на неизвестную пластину или набор".into(),
            );
        }
        if contours
            .iter()
            .filter(|c| {
                matches!(
                    c.kind,
                    Kind::Slab | Kind::Support | Kind::Beam | Kind::Column
                )
            })
            .any(|c| !self.assignments.contains_key(&c.id))
        {
            return Err("Не назначена жёсткость объекта".into());
        }
        Ok(())
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn slab(id: u64) -> Contour {
        Contour {
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            id,
            kind: Kind::Slab,
            parent: None,
            points: vec![[0., 0.], [1000., 0.], [1000., 1000.], [0., 1000.]],
            curves: vec![],
            load: None,
            refinement: None,
            grid: None,
            support_open: false,
        }
    }
    #[test]
    fn concrete_database_shares_class_and_applies_only_an_alternative_modulus_factor() {
        let mut library = Library::default();
        library.assignments.insert(10, 1);
        library.split_legacy_entries();
        let id = library.material_id(10).unwrap();
        let concrete = library.elastic.get_mut(&id).unwrap();
        concrete.concrete = Some(DatabaseConcrete {
            class: 3,
            factor: 0.8,
            alternative_factor: 0.45,
        });
        library.alternative_enabled = true;
        let data = serde_json::to_vec(&library).unwrap();
        let restored: Library = serde_json::from_slice(&data).unwrap();
        let first = restored.resolved_entry_for(10, 0).unwrap().material;
        let second = restored.resolved_entry_for(10, 1).unwrap().material;
        assert_eq!(first.e_mpa, 32500. * 0.8);
        assert_eq!(second.e_mpa, 32500. * 0.45);
        assert_eq!(first.nu, second.nu);
        assert_eq!(first.weight, second.weight);
        assert_eq!(first.thickness_mm, second.thickness_mm);
    }

    #[test]
    fn defaults_shared_assignments_and_legacy_properties_are_preserved() {
        let mut lib = Library::default();
        assert_eq!(lib.entries[&2].material.weight.value, 0.);
        let a = slab(10);
        let mut wall = slab(20);
        wall.kind = Kind::Support;
        let mut m = Material::default();
        m.thickness_mm = 350.;
        let all = vec![a.clone(), slab(11), wall];
        lib.reconcile(&all, &BTreeMap::from([(10, m), (11, m)]), &BTreeMap::new());
        assert_eq!(lib.assignments[&10], lib.assignments[&11]);
        assert_eq!(lib.assignments[&20], 2);
        lib.entries
            .get_mut(&lib.assignments[&10])
            .unwrap()
            .material
            .thickness_mm = 400.;
        assert!(lib
            .resolve(&all, Kind::Slab)
            .values()
            .all(|m| m.thickness_mm == 400.));
        lib.validate(&all).unwrap();
        // Working references survive deletion for geometry undo; files omit them.
        assert!(!lib.snapshot(&[a]).assignments.contains_key(&20));
    }
}

fn default_beam_width() -> f64 {
    200.
}
pub fn beam_material() -> Material {
    Material {
        thickness_mm: 500.,
        ..Material::default()
    }
}

pub fn column_material() -> Material {
    let mut m = beam_material();
    m.thickness_mm = 750.;
    m
}
#[cfg(test)]
mod separated_library_tests {
    use super::*;
    #[test]
    fn shared_material_changes_elastic_properties_without_changing_sections() {
        let mut library = Library::default();
        let first = library.entries[&1].material;
        let second = library.add(
            Family::Wall,
            Material {
                thickness_mm: 450.,
                ..first
            },
        );
        library.assignments.extend([(10, 1), (11, second)]);
        library.split_legacy_entries();
        let material = library.material_id(10).unwrap();
        assert_eq!(library.material_id(11), Some(material));
        library.alternative_enabled = true;
        library
            .elastic
            .get_mut(&material)
            .unwrap()
            .alternative
            .e_mpa = first.e_mpa * 0.5;
        let resolved = library.for_variant(1);
        let second = resolved.resolved_entry(11).unwrap().material;
        assert_eq!(second.e_mpa, first.e_mpa * 0.5);
        assert_eq!(second.thickness_mm, 450.);
        assert_eq!(
            resolved.resolved_entry(10).unwrap().material.thickness_mm,
            first.thickness_mm
        );
    }
}
