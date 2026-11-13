#[derive(serde::Serialize, serde::Deserialize, Clone, Copy, PartialEq, Eq)]
pub enum LoadType {
    Permanent,
    SelfWeight,
    LongTerm,
    Temporary,
}
impl LoadType {
    pub const ALL: [Self; 4] = [
        Self::Permanent,
        Self::SelfWeight,
        Self::LongTerm,
        Self::Temporary,
    ];
    pub fn label(self) -> &'static str {
        match self {
            Self::Permanent => "Постоянное",
            Self::SelfWeight => "Собств. вес",
            Self::LongTerm => "Длительное",
            Self::Temporary => "Временное",
        }
    }
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct LoadCase {
    pub id: u64,
    pub name: String,
    pub kind: LoadType,
    pub gamma_f: f64,
    pub gamma_l: f64,
}
#[derive(serde::Serialize, serde::Deserialize, Clone, Copy, PartialEq, Debug)]
pub enum ForceUnits {
    TonForce,
    KiloNewton,
}
impl ForceUnits {
    pub fn label(self) -> &'static str {
        match self {
            Self::TonForce => "тс",
            Self::KiloNewton => "кН",
        }
    }
    pub fn intensity_label(self, dimension: u8) -> String {
        match dimension {
            0 => self.label().into(),
            1 => format!("{}/м", self.label()),
            3 => format!("{}·м", self.label()),
            _ => format!("{}/м²", self.label()),
        }
    }
}
#[derive(serde::Serialize, serde::Deserialize, Clone, Copy, PartialEq, Debug)]
pub struct Magnitude {
    pub value: f64,
    pub units: ForceUnits,
}
impl Magnitude {
    pub fn in_units(self, units: ForceUnits, g: f64) -> f64 {
        if self.units == units {
            return self.value;
        }
        match self.units {
            ForceUnits::TonForce => self.value * g,
            ForceUnits::KiloNewton => self.value / g,
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn gravity_is_applied_to_original_ton_force_values_without_unit_switch_drift() {
        let tons = Magnitude {
            value: 2.,
            units: ForceUnits::TonForce,
        };
        assert_eq!(tons.in_units(ForceUnits::KiloNewton, 10.), 20.);
        assert!((tons.in_units(ForceUnits::KiloNewton, 9.81) - 19.62).abs() < 1e-10);
        let force = Magnitude {
            value: 20.,
            units: ForceUnits::KiloNewton,
        };
        assert_eq!(force.in_units(ForceUnits::TonForce, 10.), 2.);
        assert_eq!(tons.in_units(ForceUnits::TonForce, 9.81), 2.);
        assert_eq!(force.in_units(ForceUnits::KiloNewton, 9.81), 20.);
    }
}
pub(super) const MESH_CONSTRAINT_LABEL: &str = "Учитывать в триангуляции";
pub(super) const MESH_CONSTRAINT_HELP: &str = "Учитывать границы нагрузки при триангуляции. Для площадной — также границы её отверстий, для сосредоточенной — узел. Сетка общая для всех загружений.";

#[derive(serde::Serialize, serde::Deserialize, Clone, Copy, Default, PartialEq, Debug)]
pub enum Direction {
    X,
    Y,
    #[default]
    Z,
}
impl Direction {
    pub fn index(self) -> usize {
        self as usize
    }
    pub fn label(self, moment: bool) -> &'static str {
        if moment {
            ["Mx", "My", "Mz"][self.index()]
        } else {
            ["X", "Y", "Z"][self.index()]
        }
    }
    pub fn force(self, value: f64) -> [f64; 3] {
        let mut vector = [0.; 3];
        vector[self.index()] = if self == Self::Z { -value } else { value };
        vector
    }
}
#[derive(serde::Serialize, serde::Deserialize, Clone, PartialEq, Debug)]
pub struct LoadData {
    #[serde(default)]
    pub direction: Direction,
    #[serde(default)]
    pub moment: bool,
    #[serde(default)]
    pub additive: bool,
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub mesh_constraint: bool,
    pub case_id: u64,
    pub magnitude: Magnitude,
    pub closed: bool,
}
impl LoadCase {
    pub fn new(id: u64) -> Self {
        Self {
            id,
            name: format!("Загружение {id}"),
            kind: LoadType::Permanent,
            gamma_f: 1.,
            gamma_l: 1.,
        }
    }
    pub fn display_name(&self) -> String {
        if self.name.trim().is_empty() {
            format!("Загружение {}", self.id)
        } else {
            self.name.clone()
        }
    }
}
