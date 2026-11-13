//! Exact multipoint condensation q=T*q_reduced, with rigid-arm offsets in metres.
use std::collections::{BTreeMap, BTreeSet};
type Row = Vec<(usize, f64)>;
pub fn projection(
    n: usize,
    xyz: &[[f64; 3]],
    rotations: &[[Row; 3]],
    groups: &[crate::plan_trace::columns::Rigid],
    fixed: &BTreeSet<usize>,
    fixed_rotations: &BTreeSet<usize>,
) -> Result<Vec<Row>, String> {
    let mut t = (0..n).map(|i| vec![(i, 1.)]).collect::<Vec<_>>();
    let mut assigned = BTreeSet::new();
    for group in groups {
        let m = group.master;
        for &node in &group.nodes {
            if node == m {
                continue;
            }
            if !assigned.insert(node) {
                return Err("Узел принадлежит нескольким АЖТ".into());
            }
            let d = std::array::from_fn::<_, 3, _>(|a| (xyz[node][a] - xyz[m][a]) / 1000.);
            let cross = [[0., d[2], -d[1]], [-d[2], 0., d[0]], [d[1], -d[0], 0.]];
            for a in 0..3 {
                let mut row = BTreeMap::from([(3 * m + a, 1.)]);
                for axis in 0..3 {
                    for &(q, w) in &rotations[m][axis] {
                        *row.entry(q).or_default() += cross[a][axis] * w;
                    }
                }
                t[3 * node + a] = row.into_iter().filter(|(_, w)| w.abs() > 1e-14).collect();
            }
            let slave = rotations[node]
                .iter()
                .flatten()
                .map(|(q, _)| *q)
                .collect::<BTreeSet<_>>();
            for q in slave {
                let mut row = BTreeMap::new();
                for axis in 0..3 {
                    let c = rotations[node][axis]
                        .iter()
                        .filter(|(i, _)| *i == q)
                        .map(|(_, w)| *w)
                        .sum::<f64>();
                    for &(p, w) in &rotations[m][axis] {
                        *row.entry(p).or_insert(0.) += c * w;
                    }
                }
                t[q] = row.into_iter().filter(|(_, w)| w.abs() > 1e-14).collect();
            }
        }
    }
    // Homogeneous boundary conditions at slave nodes become exact equations on masters.
    let mut equations = Vec::<(usize, BTreeMap<usize, f64>)>::new();
    let mut constraints = fixed.iter().map(|&d| t[d].clone()).collect::<Vec<_>>();
    for &d in fixed_rotations {
        let row = rotations
            .get(d / 3)
            .ok_or("Нет узла закрепления поворота")?;
        constraints.push(
            row[d % 3]
                .iter()
                .flat_map(|&(i, c)| t[i].iter().map(move |&(q, w)| (q, c * w)))
                .collect(),
        );
    }
    for constraint in constraints {
        let mut row = BTreeMap::<usize, f64>::new();
        for (q, w) in constraint {
            if !fixed.contains(&q) {
                *row.entry(q).or_default() += w;
            }
        }
        for (pivot, eq) in &equations {
            if let Some(c) = row.remove(pivot) {
                for (&p, &w) in eq {
                    if p != *pivot {
                        *row.entry(p).or_default() -= c * w;
                    }
                }
            }
        }
        row.retain(|_, w| w.abs() > 1e-12);
        if let Some((&pivot, &value)) = row.iter().max_by(|a, b| a.1.abs().total_cmp(&b.1.abs())) {
            for w in row.values_mut() {
                *w /= value;
            }
            equations.push((pivot, row));
        }
    }
    for (pivot, row) in equations.iter().rev() {
        for expr in &mut t {
            let c = expr
                .iter()
                .filter(|(i, _)| i == pivot)
                .map(|(_, w)| *w)
                .sum::<f64>();
            if c == 0. {
                continue;
            }
            let mut values = expr
                .iter()
                .filter(|(q, _)| q != pivot)
                .copied()
                .collect::<BTreeMap<_, _>>();
            for (&q, &w) in row {
                if q != *pivot {
                    *values.entry(q).or_default() -= c * w;
                }
            }
            *expr = values
                .into_iter()
                .filter(|(_, w)| w.abs() > 1e-12)
                .collect();
        }
    }
    for row in &mut t {
        row.retain(|(q, _)| !fixed.contains(q));
    }
    Ok(t)
}
pub fn gather(t: &[Row], f: &[f64]) -> Vec<f64> {
    let mut out = vec![0.; t.len()];
    for (row, &v) in t.iter().zip(f) {
        for &(q, w) in row {
            out[q] += v * w;
        }
    }
    out
}
