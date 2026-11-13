# Native FEM solver notices

The native plate solver depends on unmodified faer 0.24.4, pinned in Cargo.lock.
Upstream: https://github.com/sarah-quinones/faer-rs
Package: https://crates.io/crates/faer/0.24.4
The package manifest declares MIT. The package also carries notices for code
derived from Eigen (MPL-2.0), LAPACK (BSD), SuiteSparse AMD and COLAMD (BSD).
Those upstream notices are reproduced here without modification. Do not strip
them from a binary distribution containing this solver.

`faer-0.24.4.crate` is the complete unmodified upstream source package (gzip tar),
included for the source-availability requirements of any MPL-covered portions.
The application adds no changes to faer sources. This source directory must be
included alongside the executable when a new FEM-enabled release is packaged.
Cargo.lock records transitive dependency versions; retain their notices when
preparing the distribution. No solver-enabled binary has been packaged yet.

The DKT formulation in src/plan_trace/fem/element.rs was implemented directly
from the discrete Kirchhoff interpolation, independently of faer and third-party
finite-element implementation code.
