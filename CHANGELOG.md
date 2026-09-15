# Changelog

All notable changes to this project will be documented in this file.

Format:

- Unreleased
  - Added: initial project scaffolding
  - Added: physics midphase with persistent pair pool (PairKey/generational handle slots), persistent contact manifolds with local anchors and accumulated impulse caches, deterministic anchor matching, and Enter/Stay/Exit contact events published per (fixedStepId, queryEpoch, PairKey)
  - Added: Phase 4 contact solver pipeline (Prepare -> WarmStart -> accumulated-incremental velocity iterations -> ResolvePosition -> CommitSolvedImpulses) with substep-temporary PreparedContactConstraint, friction-disc clamping, dt-scaled warm start with tangent reprojection, one-shot restitution bias, and fixed-step total impulse summary fields (fixedStepNormalImpulse / fixedStepTangentImpulse); legacy iterative-detection path removed
  - Changed: CollisionDetector split into broad-phase / midphase / narrow-phase stages with main-thread commit; PhysicsWorld publishes a PairKey-keyed per-fixed-step contact summary (replacing the O(n^2) merge), normalImpulse now reports the accumulated impulse of the last touching sub-step, and external pose writes (teleport) are distinguished from internal integrator/solver writes via revision counters
  - Removed: legacy bounce boost and non-physical angular scaling in one-sided contacts (intentional behavior correction; restitution now enters as a single Prepare-stage velocity bias)

## [1.0.0] - YYYY-MM-DD
- Initial release
