# Mixed triangle/quad correction (2026-10-29)

Current mixed::solve uses MITC3 triangles and MITC4 quads in the same
Reissner–Mindlin theory. Shell::build makes the same choice for all triangles
when its scene contains any quad (including walls); pure triangular scenes
retain DKT. Both stiffness and M/Q recovery use the selected kernel. No
smoothing, fictitious midside nodes, or extra global DOFs were introduced.
MITC3 covariant shear is a+c*s, b-c*r with three edge-midpoint ties, linear
rotations and three-point integration. See Lee/Bathe 2004, Fig8, DOI
10.1016/j.compstruc.2004.02.004. Plain MITC3 can lock for very thin plates;
it is not the improved MITC3+ nor the separate P2/CR formulation.
Source-based and mixed-patch tests plus current Project2 interface diagnostics
passed. Production EXE still requires an explicitly requested rebuild.

The historical DKT pilot description below only describes that kernel;
its exclusions do not describe the current application as a whole.

Load visualization is split by type. CaseResult.loads_kn_m2 now contains ONLY
area-load intensity per element; it is not an equivalent pressure for point/line
forces. The assembled RHS, applied force and moments are unchanged. Line/point
views use original load geometry and current case/combination/variant weights,
in force/m and force respectively; they are not represented as element pressures.

# DKT pilot: conventions and verification

Native-only module for Slab Contours. It is not connected to Matrix Grid/Web.
DOFs at each vertex are `[w, dw/dx, dw/dy]` in global XY. These slopes are
equivalent to the two bending rotations (with the appropriate sign convention).
Three rigid modes are `1, x, y`. Geometry and renderer IDs are not changed.

Input geometry/h: mm, E: MPa, ν: dimensionless. Kernel: m and kN.
Positive w and positive transverse load act downward. Moments are `-D κ`, with
κ = `[w,xx, w,yy, 2w,xy]`; sagging is positive. Results: w in mm; Mx, My, Mxy
in kN m/m, at element centroids in GLOBAL XY, independent of node ordering.
No transverse shear deformation, membrane DOFs, self-weight, combinations,
geometric/material nonlinearity, or reinforcement design is included.

The quadratic slope field is defined by vertex slopes and midside slopes.
At an edge midpoint its tangential component is the cubic-Hermite derivative
`1.5 (wj-wi)/L - .25 (ti+tj)`; its normal component is the average of endpoint
normal slopes. Three-point triangle integration integrates BᵀDB exactly for
constant isotropic material. Per-slab default h=200 mm, E=30000 MPa, ν=.2
is editable and must be replaced with the actual project values.

Slab boundaries are retained as mesh constraints, permitting adjacent plates
with different h/E/ν. Overlapping plates of different materials are rejected;
overlapping identical plates provide one layer of stiffness, not a sum.
Support-perimeter nodes have w=0; rotations are free, with no artificial springs.
Each connected component must have non-collinear restraints on w.

Assembly uses upper-triangular sparse CSC, eliminates fixed DOFs, and applies
diagonal scaling. Native faer sparse Cholesky LLᵀ factors once for all cases.
Only local 9×9 matrices and global vectors are dense. Singular/indefinite
factorisation and excessive scaled residual terminate the calculation.

Loads are nominal per load case; γf/γl are not applied. Tons-force convert to kN
using the entered g. Point loads use linear barycentric transverse interpolation,
without equivalent nodal rotation moments. This preserves force and first
moments. Area loads apply their FULL intensity over the ENTIRE element whenever the
zone and element interiors intersect. Boundary-only contact does not activate
a load. No overlap-area fraction is computed; overlapping zones sum. Line loads
remain analytic in the source model; derived segments get midpoint point loads
with spacing no greater than the smallest mesh-scale estimate. Curves use their
fine derived display polyline as the integration geometry, so curved loads are
an approximation, not exact analytic integration. Point/line samples outside
concrete cause an error. Area loads act only on existing concrete elements. A spatial hash
limits candidate triangle searches. Reactions and force/first-moment balance
are recovered by element matrix/vector products, independently of CSC storage.

UI runs the solve on a worker; edits invalidate pending/completed results.
Select the active load case at the bottom and the result field in the right
expander. Current release changes after the first pilot: w colors are now
interpolated from vertex values instead of constant element averages; moments
remain constant per element. Hover w inside an element uses linear barycentric
interpolation. Optional labels show nodal w / centroid moments, suppressing label
overlap; cursor annotations remain active without persistent labels. Hovering
a node shows its w; for moment views, a containing adjacent element's moments
are identified explicitly (no invented nodal moment averaging).
Supports may be open or closed paths; only actual path segments constrain w.
Open support segments are clipped to the concrete domain without closing chords
or artificial support segments along hole edges. No result export/save yet.

Current combinations/Q extensions: nominal per-case solutions
remain unchanged. Postprocessing superposes w/M/Q, reactions and force/first-
moment balances. Table coefficients c become c, c·γl, c/γf or c·γl/γf for design,
design-long, normative or normative-long views. Load-case IDs are stable keys.
Only the combined vector cache changes when coefficients or γ change; no new
matrix assembly/factorisation/solve. New accepted base results clear that cache.
Combined diagnostics show maximum participating base relative residual, not a
new combined relative residual. Nonzero missing cases/nonfinite values/overflow
are errors; zero-weight missing cases are ignored.

Qx = ∂Mx/∂x + ∂Mxy/∂y; Qy = ∂My/∂y + ∂Mxy/∂x in global XY, kN/m.
The exact derivatives use the underlying linear DKT moment field at barycentric
vertices, not differences of displayed centroid moments. Q is constant within
each triangle; no interelement equilibrium recovery or nodal averaging is done.
Source tests compare Q with boundary moment flux and reversed orientation;
the new Q results still need convergence benchmarks.
Reference: [Code_Aster R3.07.03](https://code-aster.org/doc/v11/fr/man_r/r3/r3.07.03.pdf)
describes equilibrium recovery of DKT transverse forces.

Source tests include rigid/quadratic patches with reversed triangle order,
point/area/line force and first-moment preservation, missing supports, linear
scaling between cases, and a simply-supported 6m square against Navier.
On 2026-10-27 `cargo fmt --all --check` and `cargo test --bin slab_contours`
passed: 69 tests, including the FEM/combination/scale verification tests listed above.
Release packaging and graphical smoke status are recorded in HANDOFF.md.
For subsequent changes run:

```
cargo test --bin slab_contours
cargo build --bin slab_contours
cargo run --bin slab_contours -- --smoke-test
```

Then add/refine convergence studies, rotated/skew meshes, holes, material jumps,
and large-model timing before treating this as a validated engineering solver.

## Element selector and Mindlin P2/CR

Default `ElementType::Dkt` keeps the original kernel. DKT omits transverse shear,
so it already does not exhibit transverse-shear locking. The optional independent
Reissner–Mindlin kernel uses continuous quadratic w (vertices + globally shared
edge midpoints) and nonconforming linear CR rotations (two globally shared
components at each edge midpoint). Local order: w at vertices 0..3, w at edges
(01,12,20) 3..6, and two rotations per edge 6..12. Thus 12 local DOFs and
N_vertices + 3 N_edges global DOFs. Mesh/model IDs do not change.

Energy is bending with the same D tensor plus κGh |grad(w)-β|², κ=5/6.
P2 functions are Li(2Li-1), 4LiLj; CR functions are 1-2L_opposite_edge.
Three-point quadrature integrates the quadratic shear energy exactly; no
reduced integration, fake stiffness or thickness-dependent penalty is added.
Area loading uses full-element triangle quadrature of P2 shapes for every
element with interior overlap with the zone, matching the DKT activation rule.
Point/line sample forces use P2 interpolation, preserving force and first moments.
Negative equivalent vertex forces for some interior points are legitimate P2
shape weights. Edge w is fixed only if the edge lies on an actual support line;
two fixed endpoints alone must not constrain an interior diagonal. Rotations free.
Midpoint support reactions are represented at original endpoint nodes by halves,
preserving resultant and first moments. Global sparse scaling/Cholesky/case reuse
and residual diagnostics remain as in the DKT solver.

M = -D curvature(β) is constant per triangle. Q = κGh (grad(w)-β) is evaluated
at the centroid; it is not recovered from derivatives of constant centroid M.
The internal solve uses all P2/CR DOFs, but CaseResult currently returns w only
at original mesh vertices: plot/hover interpolation remains linear there, and
displayed nodal min/max does not include internal midpoint extrema. Full P2
visualisation/output is not implemented. Material defaults and units unchanged.

Switching type invalidates accepted/pending analysis and combinations, preserving
geometry/triangulation. Type is part of the async input snapshot. Source tests
cover zero-shear rigid/quadratic patches, reversed orientation, P2 force/point
preservation, exact full-area loads, and Navier/thickness scaling (200/20/2 mm).
On 2026-10-27 all these tests passed in the native suite: 78 passed, plus
the separately executed retained-GPU test passed (79 distinct tests total).
Release build and graphical smoke passed. Larger convergence studies, distorted
meshes, thick-plate benchmarks and combined support/local-size cases remain
additional validation work; the current Navier/thin-limit test is not exhaustive.

Method reference: [Jeremy Bleyer's P2/CR plate example](https://comet-fenics.readthedocs.io/en/latest/demo/reissner_mindlin_crouzeix_raviart/reissner_mindlin_CR.html).
This Rust implementation was written independently from the variational form;
no FEniCS or third-party implementation source was copied or added as a dependency.
