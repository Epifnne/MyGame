# Changelog

All notable changes to this project will be documented in this file.

Format:

- Unreleased
  - Added (2026-10-08): island-level sleep with displacement/orientation probes, retained sleeping contact pairs, whole-island wake-up on external activity and confirmed collisions, and awake/sleeping body telemetry
  - Added: analytic sphere/sphere and sphere/box queries, 15-axis box SAT, clipped convex contact patches with physical surface witnesses and contact topology, and separate SAT/primitive query counters
  - Changed: persistent contact reuse validates generation-relative poses and tangential anchor drift; impulse matching uses a 1 cm local-anchor threshold and preserves tangent/spin caches
  - Changed: contact solver uses manifold-centroid tangent and spin friction, recomputed surface-anchor position correction, force-compensated restitution, and one warm start per fixed step across TOI substeps
  - Changed: CCD uses swept-BVH candidates and conservative advancement with rotational bounds; its motion threshold is independent of speculative contacts, and separated speculative pairs remain eligible for CCD
  - Added: sleep and stack-pipeline regressions, including wake-up behavior, worker-count determinism, contact witnesses, non-face stacking, and speculative/CCD interaction
  - Added: standalone MetalCubeSample with pause/resume controls and rendered-frame smoke test; BoxStackSample, FieldRainBenchmark, and optional JoltBoxFillSample for visualization and comparison
  - Changed: all samples now have independent directories and CMake configurations; rendering samples store GLSL in shaders subdirectories and embed shader sources at configure time
  - Changed: PhysicsCollisionSample uses the five regular Platonic convex hulls and provides a deterministic headless verification mode; the main game no longer renders the rotating cube, while its ball and ground reuse the sample's metal shaders
  - Added: release-o3 configure/build/test presets (-O3 -DNDEBUG), benchmark sleep controls and SAT/primitive/sleep telemetry, and a Windows minidump inspection script
  - Removed: obsolete rule, investigation, and staged physics planning documents
  - Added: initial project scaffolding
  - Added: physics midphase with persistent pair pool (PairKey/generational handle slots), persistent contact manifolds with local anchors and accumulated impulse caches, deterministic anchor matching, and Enter/Stay/Exit contact events published per (fixedStepId, queryEpoch, PairKey)
  - Added: Phase 4 contact solver pipeline (Prepare -> WarmStart -> accumulated-incremental velocity iterations -> ResolvePosition -> CommitSolvedImpulses) with substep-temporary PreparedContactConstraint, friction-disc clamping, dt-scaled warm start with tangent reprojection, one-shot restitution bias, and fixed-step total impulse summary fields (fixedStepNormalImpulse / fixedStepTangentImpulse); legacy iterative-detection path removed
  - Changed: CollisionDetector split into broad-phase / midphase / narrow-phase stages with main-thread commit; PhysicsWorld publishes a PairKey-keyed per-fixed-step contact summary (replacing the O(n^2) merge), normalImpulse now reports the accumulated impulse of the last touching sub-step, and external pose writes (teleport) are distinguished from internal integrator/solver writes via revision counters
  - Removed: legacy bounce boost and non-physical angular scaling in one-sided contacts (intentional behavior correction; restitution now enters as a single Prepare-stage velocity bias)

## [1.0.0] - YYYY-MM-DD
- Initial release
