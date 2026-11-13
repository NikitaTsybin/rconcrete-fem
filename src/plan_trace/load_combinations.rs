//! Linear postprocessing only: never changes or refactors the FE stiffness.
use super::{
    fem::{CaseResult, Results},
    loads::LoadCase,
};
use std::collections::BTreeMap;
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Combination {
    pub id: u64,
    #[serde(default)]
    pub name: String,
    pub coefficients: BTreeMap<u64, f64>,
}
impl Combination {
    pub fn new(id: u64, cases: impl IntoIterator<Item = u64>) -> Self {
        Self {
            id,
            name: id.to_string(),
            coefficients: cases.into_iter().map(|id| (id, 1.)).collect(),
        }
    }
    pub fn display_name(&self) -> String {
        if self.name.trim().is_empty() {
            self.id.to_string()
        } else {
            self.name.clone()
        }
    }
    pub fn weights(&self, cases: &[LoadCase], variant: Variant) -> Result<Vec<(u64, f64)>, String> {
        cases
            .iter()
            .map(|case| {
                variant
                    .weight(case, *self.coefficients.get(&case.id).unwrap_or(&1.))
                    .map(|w| (case.id, w))
            })
            .collect()
    }
}
#[derive(Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Variant {
    Design,
    DesignLong,
    Normative,
    NormativeLong,
}
impl Variant {
    pub fn weight(self, case: &LoadCase, coefficient: f64) -> Result<f64, String> {
        if !coefficient.is_finite() {
            return Err("Коэффициент должен быть конечным".into());
        }
        if coefficient == 0. {
            return Ok(0.);
        }
        let mut w = coefficient;
        if matches!(self, Self::Normative | Self::NormativeLong) {
            if !case.gamma_f.is_finite() || case.gamma_f <= 0. {
                return Err(format!(
                    "{}: γf должен быть больше нуля",
                    case.display_name()
                ));
            }
            w /= case.gamma_f;
        }
        if matches!(self, Self::DesignLong | Self::NormativeLong) {
            if !case.gamma_l.is_finite() || case.gamma_l < 0. {
                return Err(format!("{}: некорректный γl", case.display_name()));
            }
            w *= case.gamma_l;
        }
        if !w.is_finite() {
            return Err("Слишком большой коэффициент".into());
        }
        Ok(w)
    }
    pub const ALL: [Self; 4] = [
        Self::Design,
        Self::DesignLong,
        Self::Normative,
        Self::NormativeLong,
    ];
    pub fn label(self) -> &'static str {
        match self {
            Self::Design => "Расчетные",
            Self::DesignLong => "Расчетные длит.",
            Self::Normative => "Нормативные",
            Self::NormativeLong => "Нормативные длит.",
        }
    }
}
fn components<'a>(
    results: &'a Results,
    weights: &[(u64, f64)],
) -> Result<Vec<(&'a CaseResult, f64)>, String> {
    let mut components = Vec::new();
    let mut total_weight = 0.;
    for &(case, weight) in weights {
        if !weight.is_finite() {
            return Err("Некорректный вес сочетания".into());
        }
        if weight == 0. {
            continue;
        }
        let case_result = results
            .cases
            .iter()
            .find(|c| c.id == case)
            .ok_or_else(|| format!("Нет результатов загружения {case}; выполните расчёт"))?;
        total_weight += weight;
        components.push((case_result, weight));
    }
    if let Some(nonlinear) = &results.nonlinear {
        let weight = 1. - total_weight;
        if weight != 0. {
            components.push((&nonlinear.offset, weight));
        }
    }
    Ok(components)
}

pub(super) fn combine_area_loads(
    results: &Results,
    id: u64,
    weights: &[(u64, f64)],
) -> Result<CaseResult, String> {
    let base = results.cases.first().ok_or("Нет результатов загружений")?;
    let mut loads = vec![0.; base.loads_kn_m2.len()];
    for (case, weight) in components(results, weights)? {
        if loads.len() != case.loads_kn_m2.len() {
            return Err("Результаты относятся к разным сеткам".into());
        }
        for (load, value) in loads.iter_mut().zip(&case.loads_kn_m2) {
            *load += weight * value;
        }
    }
    Ok(CaseResult {
        id,
        loads_kn_m2: loads,
        shell: None,
        punching: Default::default(),
        w_mm: vec![],
        rotations_rad: vec![],
        moments: vec![],
        shears: vec![],
        reaction_kn: vec![],
        foundation_pressure_kn_m2: vec![],
        applied_kn: f64::NAN,
        balance_kn: f64::NAN,
        balance_m_knm: [f64::NAN; 2],
        relative_residual: f64::NAN,
    })
}

pub fn combine(results: &Results, id: u64, weights: &[(u64, f64)]) -> Result<CaseResult, String> {
    let base = results.cases.first().ok_or("Нет результатов загружений")?;
    let mut out = CaseResult {
        punching: Default::default(),
        shell: None,
        id,
        w_mm: vec![0.; base.w_mm.len()],
        rotations_rad: vec![[0.; 3]; base.rotations_rad.len()],
        moments: vec![[0.; 3]; base.moments.len()],
        shears: vec![[0.; 2]; base.shears.len()],
        reaction_kn: vec![0.; base.reaction_kn.len()],
        loads_kn_m2: vec![0.; base.loads_kn_m2.len()],
        foundation_pressure_kn_m2: base
            .foundation_pressure_kn_m2
            .iter()
            .map(|v| if v.is_finite() { 0. } else { f64::NAN })
            .collect(),
        applied_kn: 0.,
        balance_kn: 0.,
        balance_m_knm: [0.; 2],
        relative_residual: 0.,
    };
    if let Some(s) = &base.shell {
        out.shell = Some(super::fem::shell::Case {
            rotations_rad: vec![[0.; 3]; s.rotations_rad.len()],
            restrained_reactions_kn: vec![[0.; 3]; s.restrained_reactions_kn.len()],
            plate_transfer_kn: vec![0.; s.plate_transfer_kn.len()],
            surface_forces: s
                .surface_forces
                .iter()
                .map(|f| {
                    let mut f = f.clone();
                    f.moments = [0.; 3];
                    f.shears = [0.; 2];
                    f.membrane = [0.; 3];
                    f
                })
                .collect(),
            translations_mm: vec![[0.; 3]; s.translations_mm.len()],
            beam_forces: s
                .beam_forces
                .iter()
                .map(|force| {
                    let mut force = force.clone();
                    force.ends = [[0.; 6]; 2];
                    force.geometric_moments = force.geometric_moments.map(|_| [[0.; 5]; 2]);
                    force
                })
                .collect(),
            membrane: vec![[0.; 3]; s.membrane.len()],
            support_reactions_kn: s
                .support_reactions_kn
                .keys()
                .map(|&id| (id, [0.; 3]))
                .collect(),
        });
    }
    for (c, w) in components(results, weights)? {
        if c.w_mm.len() != out.w_mm.len()
            || c.moments.len() != out.moments.len()
            || c.shears.len() != out.shears.len()
            || c.loads_kn_m2.len() != out.loads_kn_m2.len()
            || c.reaction_kn.len() != out.reaction_kn.len()
        {
            return Err("Результаты относятся к разным сеткам".into());
        }
        if c.rotations_rad.len() != out.rotations_rad.len() {
            return Err("Несовместимые узловые повороты".into());
        }
        for (&id, r) in &c.punching {
            let target =
                out.punching
                    .entry(id)
                    .or_insert_with(|| super::fem::cut_forces::CutResult {
                        center: r.center,
                        method: r.method,
                        base_diagnostic: r.base_diagnostic.clone(),
                        edge_forces: r
                            .edge_forces
                            .iter()
                            .map(|e| super::fem::cut_forces::EdgeForce {
                                index: e.index,
                                ends: e.ends,
                                force: 0.,
                                mx_origin: 0.,
                                my_origin: 0.,
                                active: e.active,
                                diagnostic: e.diagnostic.clone(),
                            })
                            .collect(),
                        nodes: r.nodes,
                        elements: r.elements,
                        ..Default::default()
                    });
            if target.method != r.method
                || target.edge_forces.len() != r.edge_forces.len()
                || target
                    .edge_forces
                    .iter()
                    .zip(&r.edge_forces)
                    .any(|(a, b)| a.index != b.index || a.ends != b.ends || a.active != b.active)
            {
                return Err("Несовместимые рёбра областей суммирования нагрузок".into());
            }
            for (a, b) in target.edge_forces.iter_mut().zip(&r.edge_forces) {
                a.force += w * b.force;
                a.mx_origin += w * b.mx_origin;
                a.my_origin += w * b.my_origin;
                if !b.diagnostic.is_empty() {
                    a.diagnostic = b.diagnostic.clone();
                }
            }
            target.force += w * r.force;
            #[cfg(test)]
            {
                target.closing_force += w * r.closing_force;
                target.inner_force += w * r.inner_force;
                target.full_net_force += w * r.full_net_force;
            }
            target.mx += w * r.mx;
            target.my += w * r.my;
            if !r.diagnostic.is_empty() {
                target.diagnostic = r.diagnostic.clone();
            }
        }
        for (a, b) in out.rotations_rad.iter_mut().zip(&c.rotations_rad) {
            for j in 0..3 {
                a[j] += w * b[j];
            }
        }
        for (a, b) in out.w_mm.iter_mut().zip(&c.w_mm) {
            *a += w * b;
        }
        for (i, a) in out.foundation_pressure_kn_m2.iter_mut().enumerate() {
            *a += c
                .foundation_pressure_kn_m2
                .get(i)
                .copied()
                .unwrap_or(f64::NAN)
                * w;
        }
        for (a, b) in out.loads_kn_m2.iter_mut().zip(&c.loads_kn_m2) {
            *a += w * b;
        }
        for (a, b) in out.reaction_kn.iter_mut().zip(&c.reaction_kn) {
            *a += w * b;
        }
        for (a, b) in out.moments.iter_mut().zip(&c.moments) {
            for j in 0..3 {
                a[j] += w * b[j];
            }
        }
        for (a, b) in out.shears.iter_mut().zip(&c.shears) {
            for j in 0..2 {
                a[j] += w * b[j];
            }
        }
        out.applied_kn += w * c.applied_kn;
        out.balance_kn += w * c.balance_kn;
        for j in 0..2 {
            out.balance_m_knm[j] += w * c.balance_m_knm[j];
        }
        out.relative_residual = out.relative_residual.max(c.relative_residual);
        match (&mut out.shell, &c.shell) {
            (Some(a), Some(b)) => {
                if a.translations_mm.len() != b.translations_mm.len()
                    || a.membrane.len() != b.membrane.len()
                    || !a
                        .support_reactions_kn
                        .keys()
                        .eq(b.support_reactions_kn.keys())
                {
                    return Err("Несовместимые пространственные результаты".into());
                }
                for (a, b) in a
                    .translations_mm
                    .iter_mut()
                    .zip(&b.translations_mm)
                    .chain(a.membrane.iter_mut().zip(&b.membrane))
                {
                    for j in 0..3 {
                        a[j] += w * b[j];
                    }
                }
                if a.rotations_rad.len() != b.rotations_rad.len()
                    || a.surface_forces.len() != b.surface_forces.len()
                {
                    return Err("Несовместимые результаты поверхностей".into());
                }
                if a.plate_transfer_kn.len() != b.plate_transfer_kn.len() {
                    return Err("Несовместимые реакции плит".into());
                }
                for (a, b) in a.plate_transfer_kn.iter_mut().zip(&b.plate_transfer_kn) {
                    *a += w * b;
                }
                if a.restrained_reactions_kn.len() != b.restrained_reactions_kn.len() {
                    return Err("Несовместимые узловые реакции".into());
                }
                for (target, source) in a
                    .restrained_reactions_kn
                    .iter_mut()
                    .zip(&b.restrained_reactions_kn)
                {
                    for j in 0..3 {
                        target[j] += w * source[j];
                    }
                }
                for (target, source) in a.rotations_rad.iter_mut().zip(&b.rotations_rad) {
                    for j in 0..3 {
                        target[j] += w * source[j];
                    }
                }
                for (target, source) in a.surface_forces.iter_mut().zip(&b.surface_forces) {
                    if target.nodes != source.nodes
                        || target.support != source.support
                        || target.axes != source.axes
                        || target.material != source.material
                    {
                        return Err("Разная геометрия результатов поверхностей".into());
                    }
                    for j in 0..3 {
                        target.moments[j] += w * source.moments[j];
                        target.membrane[j] += w * source.membrane[j];
                    }
                    for j in 0..2 {
                        target.shears[j] += w * source.shears[j];
                    }
                }
                if a.beam_forces.len() != b.beam_forces.len() {
                    return Err("Несовместимые балочные результаты".into());
                }
                for (target, source) in a.beam_forces.iter_mut().zip(&b.beam_forces) {
                    if target.id != source.id || target.points != source.points {
                        return Err("Несовместимые участки балок".into());
                    }
                    if let Some(source) = &source.geometric_moments {
                        let target = target.geometric_moments.get_or_insert([[0.; 5]; 2]);
                        for i in 0..2 {
                            for j in 0..5 {
                                target[i][j] += w * source[i][j];
                            }
                        }
                    }
                    for end in 0..2 {
                        for component in 0..6 {
                            target.ends[end][component] += w * source.ends[end][component];
                        }
                    }
                }
                for (id, r) in &b.support_reactions_kn {
                    let target = a.support_reactions_kn.get_mut(id).unwrap();
                    for j in 0..3 {
                        target[j] += w * r[j];
                    }
                }
            }
            (None, None) => {}
            _ => return Err("Нельзя смешивать пространственные и плоские расчёты".into()),
        }
    }
    if out
        .w_mm
        .iter()
        .chain(out.moments.iter().flatten())
        .chain(out.shears.iter().flatten())
        .chain(&out.reaction_kn)
        .chain(&out.loads_kn_m2)
        .chain([&out.applied_kn, &out.balance_kn])
        .chain(&out.balance_m_knm)
        .any(|v| !v.is_finite())
    {
        return Err("Переполнение результатов сочетания".into());
    }
    if out.shell.as_ref().is_some_and(|s| {
        s.translations_mm
            .iter()
            .flatten()
            .chain(s.membrane.iter().flatten())
            .chain(s.support_reactions_kn.values().flatten())
            .chain(s.restrained_reactions_kn.iter().flatten())
            .chain(s.plate_transfer_kn.iter())
            .chain(s.beam_forces.iter().flat_map(|f| f.ends.iter().flatten()))
            .any(|v| !v.is_finite())
    }) {
        return Err("Переполнение пространственных результатов сочетания".into());
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn result(id: u64, v: f64) -> CaseResult {
        CaseResult {
            punching: Default::default(),
            shell: None,
            id,
            w_mm: vec![v, 2. * v],
            rotations_rad: vec![[0.; 3]; 2],
            moments: vec![[v, 2. * v, 3. * v]],
            shears: vec![[4. * v, 5. * v]],
            reaction_kn: vec![-v],
            loads_kn_m2: vec![6. * v],
            foundation_pressure_kn_m2: vec![],
            applied_kn: v,
            balance_kn: 0.,
            balance_m_knm: [0.; 2],
            relative_residual: 1e-12,
        }
    }
    #[test]
    fn four_variants_use_per_load_factors_and_combination_coefficients() {
        let mut c = LoadCase::new(7);
        c.gamma_f = 1.5;
        c.gamma_l = 0.4;
        let mut combo = Combination::new(1, [7]);
        combo.coefficients.insert(7, 2.);
        for (v, w) in [
            (Variant::Design, 2.),
            (Variant::DesignLong, 0.8),
            (Variant::Normative, 2. / 1.5),
            (Variant::NormativeLong, 0.8 / 1.5),
        ] {
            assert!((combo.weights(std::slice::from_ref(&c), v).unwrap()[0].1 - w).abs() < 1e-12);
        }
        c.gamma_f = 0.;
        assert!(combo
            .weights(std::slice::from_ref(&c), Variant::Normative)
            .is_err());
        assert!(combo
            .weights(std::slice::from_ref(&c), Variant::Design)
            .is_ok());
    }
    #[test]
    fn signed_superposition_includes_shears_reactions_and_keeps_base_results() {
        let r = Results {
            nonlinear: None,
            lengths: None,
            modal: None,
            buckling: None,
            cases: vec![result(7, 3.), result(2, 5.)],
            free_dofs: 9,
        };
        let a = combine(&r, 1, &[(2, 2.), (7, -1.)]).unwrap();
        assert_eq!(a.w_mm, vec![7., 14.]);
        assert_eq!(a.moments, vec![[7., 14., 21.]]);
        assert_eq!(a.shears, vec![[28., 35.]]);
        assert_eq!(a.reaction_kn, vec![-7.]);
        assert_eq!(a.loads_kn_m2, vec![42.]);
        assert_eq!(a.applied_kn, 7.);
        assert_eq!(r.cases[0].w_mm, vec![3., 6.]);
        assert!(combine(&r, 2, &[(99, 1.)]).is_err());
        assert!(combine(&r, 2, &[(99, 0.)]).is_ok());
        let z = combine(&r, 2, &[(7, 1.), (7, -1.)]).unwrap();
        assert_eq!(z.w_mm, vec![0., 0.]);
    }
    #[test]
    fn nonlinear_offset_is_counted_once_for_every_target_combination() {
        let r = Results {
            nonlinear: Some(super::super::fem::NonlinearResult {
                caption: "base".into(),
                weights: vec![],
                steps: 4,
                iterations: 8,
                offset: Box::new(result(0, 10.)),
            }),
            lengths: None,
            modal: None,
            buckling: None,
            cases: vec![result(1, 12.), result(2, 13.)],
            free_dofs: 9,
        };
        let target = combine(&r, 3, &[(1, 2.), (2, 3.)]).unwrap();
        assert_eq!(target.w_mm, vec![23., 46.]);
        let area = combine_area_loads(&r, 3, &[(1, 2.), (2, 3.)]).unwrap();
        assert_eq!(area.loads_kn_m2, target.loads_kn_m2);
        let unloaded = combine(&r, 3, &[]).unwrap();
        assert_eq!(unloaded.w_mm, vec![10., 20.]);
        assert_eq!(
            combine_area_loads(&r, 3, &[]).unwrap().loads_kn_m2,
            unloaded.loads_kn_m2
        );
    }
}
