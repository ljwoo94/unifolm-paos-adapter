# IK profiling and library alternatives

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

For this Adapter, compare an arm-only PlaCo or Pinocchio/Pink configuration using the original URDF, locked measured lower-body/waist, pelvis-relative tool targets and the same constraints/input corpus. Keep the existing WBC responsible for balance/locomotion; do not accidentally introduce a second full-body actuator controller. A library's C++ implementation does not prove lower total latency or successful handling of unreachable poses. No external IK library was installed or benchmarked in this change, and no production backend switch is claimed.
