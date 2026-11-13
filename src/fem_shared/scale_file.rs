//! Portable, versioned JSON scale preset. No model geometry/results are included.
use serde::{Deserialize, Serialize};
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Preset {
    pub format: String,
    pub version: u32,
    pub kind: String,
    pub edges: Vec<f64>,
    pub labels: Vec<String>,
    pub colors: Vec<[u8; 3]>,
    pub enabled: Vec<bool>,
}
impl Preset {
    pub fn validate(&self) -> Result<(), String> {
        let n = self.colors.len();
        if self.format != "mini-fem-scale" || self.version != 1 {
            return Err("Unsupported scale file format/version".into());
        }
        if n == 0
            || n > 4096
            || self.labels.len() != n
            || self.enabled.len() != n
            || self.labels.iter().any(|s| s.len() > 1024)
        {
            return Err("Invalid scale row count or labels".into());
        }
        match self.kind.as_str() {
            "numeric" | "deficit"
                if n <= 32
                    && self.edges.len() == n + 1
                    && self.edges.iter().all(|v| v.is_finite())
                    && self.edges.windows(2).all(|p| p[0] <= p[1])
                    && self.edges[0] < self.edges[n] =>
            {
                Ok(())
            }
            "dxf-colors" if self.edges.is_empty() => Ok(()),
            _ => Err("Invalid scale kind or boundaries".into()),
        }
    }
    pub fn encode(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string_pretty(self).map_err(|e| e.to_string())
    }
    pub fn decode(text: &str) -> Result<Self, String> {
        if text.len() > 4 * 1024 * 1024 {
            return Err("Scale file too large".into());
        }
        let preset: Self = serde_json::from_str(text).map_err(|e| e.to_string())?;
        preset.validate()?;
        Ok(preset)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn roundtrip_preserves_expression_colors_and_visibility_rejects_bad_files() {
        let mut p = Preset {
            format: "mini-fem-scale".into(),
            version: 1,
            kind: "numeric".into(),
            edges: vec![0., 4.52, 9.05],
            labels: vec!["s250d12".into(), "s250d12+s250d12".into()],
            colors: vec![[10, 20, 30], [40, 50, 60]],
            enabled: vec![true, false],
        };
        let q = Preset::decode(&p.encode().unwrap()).unwrap();
        assert_eq!(q.edges, p.edges);
        assert_eq!(q.labels, p.labels);
        assert_eq!(q.colors, p.colors);
        assert_eq!(q.enabled, p.enabled);
        p.edges[1] = 10.;
        assert!(p.encode().is_err());
        p.edges[1] = 4.52;
        p.version = 2;
        assert!(p.encode().is_err());
        assert!(Preset::decode("{}").is_err());
    }
}
