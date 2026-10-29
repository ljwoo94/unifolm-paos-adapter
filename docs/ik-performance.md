# PlaCo backend and IK performance

Adapter0.3.0 uses PlaCo0.10.2 with Pinocchio3.8.0 by default. The custom DLS implementation is retained only in historical commit29dfbf3, not as a runtime fallback. Both measured lower15/waist and the floating base are fixed; only arm14 is optimized. The original G1 URDF and wrist-local+x105mm frame are unchanged.

The solver keeps joint position limits, at most45QP steps, at most0.15rad per step, and3mm/.03rad residual thresholds. Position/orientation soft-task weights are1/.12² and regularization is.015². It does not add a physical velocity-limit or collision-planning contract. Frozen lower15 keep exact measured values; only their in-memory position bounds expand if a measurement lies outside nominal URDF bounds, so these non-decision variables cannot make the arm QP infeasible. Arm bounds remain unchanged. `backend=placo` and `qp_solves` identify diagnostics; each arm residual's `iterations` is the shared dual-arm QP count. Infeasible targets remain flagged and bounded under the existing experimental projection policy; there is no silent DLS fallback.

A separate Python3.12.10/Mac ARM64 comparison used30cases×10repeats, identical fixtures/tool frames, warmup, alternating solver order and single-thread BLAS:

| Corpus | Historical DLS mean / p95 | PlaCo mean / p95 | Converged per solver |
|---|---:|---:|---:|
| Reachable |0.929 /1.015ms|0.419 /0.459ms|300/300|
| Recorded difficult targets |14.680 /15.685ms|7.951 /8.217ms|0/300|

PlaCo solves both arms in one QP; DLS solves each arm separately. Their weights/regularization are not numerically equivalent. The hard corpus is not a successful task, and these measurements exclude model/network/constructor time and establish neither WLA timing nor a100Hz worst-case bound. Raw comparison: PAOS experiments/ik-placo at report commit a13b743.

A separate replay of the actual0.3.0 production backend (same fixtures,30cases×10repeats, single-thread BLAS, same host/Python) measured reachable mean0.387ms/p950.459ms/max0.709ms and hard mean5.180ms/p955.490ms/max5.794ms. All30reachable cases converged, zero of30hard cases converged. This sequential production replay is not a new paired speedup comparison; the paired comparison above remains the basis for the DLS/PlaCo ratio. Raw JSON is in the task delivery artifacts, placo-production-reachable.json and placo-production-hard.json.

## Historical DLS optimization (0.2.2)

2026-10-29. Offline Mac ARM64 replay; no WLA model, remote installation, camera or robot was run. Baseline21610ce uses full yourdfpy/trimesh scene updates at each FK evaluation. The numeric chain is derived from the same unchanged G1 URDF, with measured waist and the same wrist-local105mm tool point. DLS weights, damping,45iteration cap, line search, joint clamp and3mm/.03rad convergence criteria are unchanged.

| Identical input corpus (30cases×3repeats) | Before mean | After mean | After p95 / max | Converged cases |
|---|---:|---:|---:|---:|
| Actual saved model EE targets from fixed recorded seed |105.403ms|13.893ms|15.607 /16.171ms|0/30 in both|
| Reachable URDF-derived near targets, perturbed seed |6.164ms|1.093ms|1.330 /1.546ms|30/30 in both|

These are offline comparison corpora, not a new live banana trial. The first corpus is deliberately difficult and not solved by either implementation. Faster nonconvergence is not a successful task result. Neither corpus establishes WLA/Orin timing or a100Hz worst-case bound.

The baseline cProfile pass took5.311s: update_cfg3.050s, get_transform1.296s. Those generic graph paths dominate. The new solver uses only URDF numeric chain transforms, caches the unchanged prefix per measured waist, evaluates FK/joint axes in one pass, and batches the geometric Jacobian assembly. Original URDF FK and independent finite-difference Jacobians agree in randomized tests. Maximum before/after solved joint difference is2.62e-13rad (hard) and1.34e-15rad (reachable); convergence flags match.

Reproduce with the same machine/runtime:

```sh
PYTHONPATH=src python benchmarks/benchmark_ik.py --fixtures benchmarks/fixtures/model-targets.npz --output hard.json
PYTHONPATH=src python benchmarks/benchmark_ik.py --fixtures benchmarks/fixtures/reachable.npz --output reachable.json
```

## Existing libraries

- [Pinocchio](https://github.com/stack-of-tasks/pinocchio): C++ kinematics/dynamics with Python bindings. Its [IK example](https://gepetto.github.io/doc/pinocchio/doxygen-html/md_doc_b-examples_d-inverse-kinematics.html) uses damped iterative solving and describes preallocation. Pinocchio supplies efficient FK/Jacobians; choosing it alone does not remove the need to define the IK optimization and convergence policy.
- [Pink](https://github.com/pink-kinematics/pink): differential task-space IK over Pinocchio and QP solvers, including configuration/velocity limits. It returns velocities to integrate; this is a different output/time contract from solving an absolute pose and immediately dispatching a joint goal.
- [PlaCo](https://placo.readthedocs.io/en/latest/): C++ QP-based IK/dynamics with Python bindings and APIs for frames, masked joints and joint/velocity limits. It is a direct candidate when the solver as well as FK should come from an existing library.

For this Adapter, compare an arm-only PlaCo or Pinocchio/Pink configuration using the original URDF, locked measured lower-body/waist, pelvis-relative tool targets and the same constraints/input corpus. Keep the existing WBC responsible for balance/locomotion; do not accidentally introduce a second full-body actuator controller. A library's C++ implementation does not prove lower total latency or successful handling of unreachable poses. That paragraph describes the historical0.2.2 investigation. The0.3.0 PlaCo selection and independent comparison are recorded above.
