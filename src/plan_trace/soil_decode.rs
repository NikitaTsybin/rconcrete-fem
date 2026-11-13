use mini_fem::soil::{Borehole, Layer};
use std::collections::BTreeMap;
pub(super) fn decode(bytes: &[u8]) -> Result<Vec<Borehole>, String> {
    let data: serde_json::Value = serde_json::from_slice(bytes).map_err(|e| e.to_string())?;
    let mut materials = BTreeMap::new();
    for s in data["soils"].as_array().ok_or("soils")? {
        let p = &s["E_nu_gamma_c_phi_R0"];
        materials.insert(
            s["id"].as_u64().ok_or("IGE")?,
            (p[0].as_f64().ok_or("E")?, p[2].as_f64().ok_or("gamma")?),
        );
    }
    let mut holes = vec![];
    for h in data["drills"].as_array().ok_or("drills")? {
        let a = &h["xy_mouth_depth"];
        let mut layers = vec![];
        for l in h["strata"].as_array().ok_or("strata")? {
            let id = l[0].as_u64().ok_or("layer id")?;
            let &(e, gamma) = materials.get(&id).ok_or("IGE reference")?;
            layers.push(Layer {
                material_id: u32::try_from(id).ok(),
                bottom_z: l[1].as_f64().ok_or("bottom")?,
                e,
                gamma,
            });
        }
        holes.push(Borehole {
            id: h["id"].as_u64().ok_or("id")? as u32,
            xy: [a[0].as_f64().ok_or("x")?, a[1].as_f64().ok_or("y")?],
            mouth_z: a[2].as_f64().ok_or("z")?,
            layers,
        });
    }
    Ok(holes)
}
/// The binary layout verified against the original S1-2 S11 SLD.
#[cfg(not(target_arch = "wasm32"))]
pub(super) fn decode_sld(bytes: &[u8]) -> Result<Vec<Borehole>, String> {
    struct Reader<'a>(&'a [u8]);
    impl<'a> Reader<'a> {
        fn take(&mut self, len: usize) -> Result<&'a [u8], String> {
            let (head, tail) = self.0.split_at_checked(len).ok_or("Неполная запись SLD")?;
            self.0 = tail;
            Ok(head)
        }
        fn byte(&mut self) -> Result<usize, String> {
            Ok(self.take(1)?[0] as usize)
        }
        fn word(&mut self) -> Result<u32, String> {
            Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
        }
        fn real(&mut self) -> Result<f64, String> {
            Ok(f32::from_bits(self.word()?) as f64)
        }
        fn count(&mut self) -> Result<usize, String> {
            let n = self.word()? as usize;
            if n > 1_000_000 {
                return Err("Некорректное количество записей SLD".into());
            }
            Ok(n)
        }
        fn string(&mut self) -> Result<&'a [u8], String> {
            let n = self.byte()?;
            if n != 255 {
                return self.take(n);
            }
            let low = self.byte()?;
            if low == 254 {
                let mut n = self.byte()?;
                if n == 255 {
                    n = self.byte()?;
                }
                self.take(n * 2)
            } else {
                let high = self.byte()?;
                self.take(low | high << 8)
            }
        }
    }
    let mut r = Reader(bytes);
    let header = r.string()?;
    let version = r.word()?;
    if header.is_empty() || version != 0x02000002 {
        return Err(format!("Неподдерживаемый формат SLD, версия {version:#x}"));
    }
    if version > 0x1000022 {
        r.take(12)?;
    }
    r.take(26)?;
    let mut drills = Vec::new();
    for _ in 0..r.count()? {
        let id = r.word()?;
        let xy = [r.real()?, r.real()?];
        let mouth_z = r.real()?;
        r.take(20)?;
        if r.byte()? != 0 {
            r.take(16)?;
        }
        for _ in 0..r.count()? {
            r.take(16)?;
            let n = r.count()?;
            r.take(n * 4)?;
        }
        let mut layers = Vec::new();
        for _ in 0..r.count()? {
            layers.push((r.word()?, r.real()?));
            r.take(8)?;
        }
        r.take(4)?;
        drills.push((id, xy, mouth_z, layers));
    }
    for _ in 0..r.count()? {
        r.take(42)?;
        if version > 0x1000004 {
            r.take(4)?;
        }
        if version > 0x1000032 {
            r.take(8)?;
        }
        r.take(16)?;
        let n = r.count()?;
        r.take(n * 8)?;
        let n = r.count()?;
        r.take(n * 4)?;
    }
    let mut soils = BTreeMap::new();
    for _ in 0..r.count()? {
        let id = r.word()?;
        r.string()?;
        let raw = r.take(56)?;
        let real = |offset| f32::from_le_bytes(raw[offset..offset + 4].try_into().unwrap()) as f64;
        soils.insert(id, (real(28), real(36)));
    }
    drills
        .into_iter()
        .map(|(id, xy, mouth_z, strata)| {
            let layers = strata
                .into_iter()
                .map(|(soil, bottom_z)| {
                    let &(e, gamma) = soils.get(&soil).ok_or("Не найден ИГЭ скважины")?;
                    Ok(Layer {
                        material_id: Some(soil),
                        bottom_z,
                        e,
                        gamma,
                    })
                })
                .collect::<Result<_, String>>()?;
            Ok(Borehole {
                id,
                xy,
                mouth_z,
                layers,
            })
        })
        .collect()
}
#[cfg(not(target_arch = "wasm32"))]
pub(super) fn read_ground(path: &std::path::Path) -> Result<Vec<Borehole>, String> {
    let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
    let holes = if path
        .extension()
        .is_some_and(|ext| ext.eq_ignore_ascii_case("sld"))
    {
        decode_sld(&bytes)?
    } else {
        decode(&bytes)?
    };
    for h in &holes {
        let mut top = h.mouth_z;
        if !top.is_finite() || h.xy.iter().any(|v| !v.is_finite()) || h.layers.is_empty() {
            return Err(format!("Некорректная скважина {}", h.id));
        }
        for l in &h.layers {
            if !l.bottom_z.is_finite()
                || l.bottom_z >= top
                || !l.e.is_finite()
                || l.e <= 0.
                || !l.gamma.is_finite()
                || l.gamma <= 0.
            {
                return Err(format!("Некорректный слой скважины {}", h.id));
            }
            top = l.bottom_z;
        }
    }
    Ok(holes)
}
