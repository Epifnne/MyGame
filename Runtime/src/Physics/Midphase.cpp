#include "Physics/Midphase.h"

#include <utility>

#include <glm/gtc/quaternion.hpp>

namespace Runtime {
namespace Physics {

void Midphase::BeginQuery(
    uint64_t queryEpoch,
    uint64_t fixedStepId,
    BroadPhaseQueryCoverage coverage) {
    m_queryEpoch = queryEpoch;
    m_fixedStepId = fixedStepId;
    m_coverage = coverage;
    m_workList.clear();
    m_stats = {};
    // Synchronization point: slots removed by the previous query's sweep may
    // only be recycled now; handles from before this point are invalid.
    RecyclePendingRemovals();
}

uint32_t Midphase::RegisterCandidate(
    const PairKey& key,
    const Collider& colliderA,
    const RigidBody& bodyA,
    const Collider& colliderB,
    const RigidBody& bodyB) {
    uint32_t slotIndex = PairHandle::kInvalidIndex;
    const auto it = m_pairToSlot.find(key);
    if (it == m_pairToSlot.end()) {
        slotIndex = AllocateSlot();
        Slot& slot = m_slots[slotIndex];
        slot.pair = MidphasePair{};
        slot.pair.key = key;
        slot.pair.state = PairLifecycleState::New;
        slot.pair.createdFixedStepId = m_fixedStepId;
        slot.pair.manifold.bodyA = key.bodyA;
        slot.pair.manifold.bodyB = key.bodyB;
        slot.pair.binding = CaptureBinding(colliderA, bodyA, colliderB, bodyB);
        m_pairToSlot.emplace(key, PairHandle{slotIndex, slot.generation});
        ++m_stats.newPairCount;
    } else {
        slotIndex = it->second.index;
        MidphasePair& pair = m_slots[slotIndex].pair;
        pair.state = PairLifecycleState::Persisting;

        // Frozen invalidation matrix: collider identity/revision (replacement,
        // shape, trigger, one-sided, layer/mask), material value, external pose
        // (teleport) and mass/inertia/static changes all clear the contact
        // caches without ending the touching state by themselves.
        const PairBindingSnapshot binding = CaptureBinding(colliderA, bodyA, colliderB, bodyB);
        pair.cacheClearedThisQuery = (binding != pair.binding);
        if (pair.cacheClearedThisQuery) {
            ClearContactCaches(pair);
            pair.binding = binding;
        }
    }

    MidphasePair& pair = m_slots[slotIndex].pair;
    pair.lastSeenQueryEpoch = m_queryEpoch;
    pair.needsNarrowphase = true;
    m_workList.push_back(slotIndex);
    return slotIndex;
}

void Midphase::FinishQuery(std::vector<ContactEvent>& events) {
    if (m_coverage == BroadPhaseQueryCoverage::FullScene) {
        for (auto it = m_pairToSlot.begin(); it != m_pairToSlot.end();) {
            MidphasePair& pair = m_slots[it->second.index].pair;
            if (pair.lastSeenQueryEpoch == m_queryEpoch) {
                ++it;
                continue;
            }
            // The pair belonged to this query's full re-check coverage and was
            // not returned: the candidate is removed. Touching pairs publish
            // Exit with the stable PairKey before the slot is queued for
            // recycling at the next synchronization point.
            if (pair.hadContact) {
                PushEvent(events, ContactEventType::Exit, pair.key);
            }
            pair.hadContact = false;
            pair.needsNarrowphase = false;
            pair.state = PairLifecycleState::Removed;
            m_pendingRemoval.push_back(it->second.index);
            ++m_stats.removedPairCount;
            it = m_pairToSlot.erase(it);
        }
    }

    m_stats.activePairCount = m_pairToSlot.size();
    m_stats.workListSize = m_workList.size();
}

void Midphase::CommitContact(
    uint32_t slotIndex,
    const RigidBody& bodyA,
    const RigidBody& bodyB,
    bool isTrigger,
    ContactManifold freshManifold,
    bool isToiImpact,
    std::vector<ContactEvent>& events) {
    MidphasePair& pair = m_slots[slotIndex].pair;
    const ContactManifold oldManifold = pair.manifold;

    freshManifold.bodyA = pair.key.bodyA;
    freshManifold.bodyB = pair.key.bodyB;
    freshManifold.isTrigger = isTrigger;

    const glm::quat invOrientationA = glm::conjugate(bodyA.Orientation());
    const glm::quat invOrientationB = glm::conjugate(bodyB.Orientation());
    for (std::size_t index = 0; index < freshManifold.pointCount; ++index) {
        ContactPoint& point = freshManifold.Point(index);
        point.localPointA = invOrientationA * (point.position - bodyA.Position());
        point.localPointB = invOrientationB * (point.position - bodyB.Position());
    }

    // Transfer accumulated impulse caches from the matched previous points,
    // unless this query's binding check already cleared them. A significant
    // normal flip also transfers nothing (handled inside the matcher).
    if (!pair.cacheClearedThisQuery) {
        MatchPersistentContactPoints(oldManifold, freshManifold);
    }

    for (std::size_t index = 0; index < freshManifold.pointCount; ++index) {
        ContactPoint& point = freshManifold.Point(index);
        // The current sub-step's TOI flag is fresh; the cache interpretation
        // data (old tangent basis, cachedDt, TOI origin) was transferred by
        // the matcher above and is re-stamped by the solver when it stores
        // the solved cache at sub-step end.
        point.isToiImpact = isToiImpact;
        // Per-sub-step accumulation base for the solver-visible field.
        point.normalImpulse = 0.0f;
    }

    pair.manifold = std::move(freshManifold);
    pair.cacheClearedThisQuery = false;

    // Fixed-step impulse totals restart at the first touch of a new fixed
    // step; sub-steps of the same fixed step accumulate into them.
    if (pair.lastTouchFixedStepId != m_fixedStepId) {
        pair.fixedStepNormalImpulse = 0.0f;
        pair.fixedStepTangentImpulse = glm::vec3(0.0f);
    }

    // Touching transitions: Enter on the first touch, Stay at most once per
    // fixed step while the contact persists.
    if (!pair.hadContact) {
        PushEvent(events, ContactEventType::Enter, pair.key);
    } else if (pair.lastStayFixedStepId != m_fixedStepId) {
        PushEvent(events, ContactEventType::Stay, pair.key);
        pair.lastStayFixedStepId = m_fixedStepId;
    }
    pair.hadContact = true;
    pair.lastTouchFixedStepId = m_fixedStepId;
}

void Midphase::CommitSeparation(uint32_t slotIndex, std::vector<ContactEvent>& events) {
    MidphasePair& pair = m_slots[slotIndex].pair;
    // Normal separation: the candidate pair is kept (fat AABBs still overlap),
    // the touching state ends and the contact caches are cleared.
    if (pair.hadContact) {
        PushEvent(events, ContactEventType::Exit, pair.key);
    }
    pair.hadContact = false;
    pair.manifold.ClearPoints();
    ClearContactCaches(pair);
}

void Midphase::CommitQueryFailure(uint32_t slotIndex) {
    // Deliberately no state change: an algorithm failure is counted by the
    // narrow-phase stats and never poses as a normal separation, so no Exit is
    // published and the previous touching state and caches are preserved.
    (void)slotIndex;
}

const MidphasePair* Midphase::FindPair(const PairKey& key) const {
    const auto it = m_pairToSlot.find(key);
    if (it == m_pairToSlot.end()) {
        return nullptr;
    }
    return &m_slots[it->second.index].pair;
}

MidphasePair* Midphase::FindPair(const PairKey& key) {
    const auto it = m_pairToSlot.find(key);
    if (it == m_pairToSlot.end()) {
        return nullptr;
    }
    return &m_slots[it->second.index].pair;
}

bool Midphase::IsHandleValid(const PairHandle& handle) const {
    if (!handle.IsValid() || handle.index >= m_slots.size()) {
        return false;
    }
    const Slot& slot = m_slots[handle.index];
    return slot.occupied && slot.generation == handle.generation;
}

uint32_t Midphase::AllocateSlot() {
    if (!m_freeSlots.empty()) {
        const uint32_t index = m_freeSlots.back();
        m_freeSlots.pop_back();
        m_slots[index].occupied = true;
        return index;
    }
    Slot slot;
    slot.occupied = true;
    m_slots.push_back(slot);
    return static_cast<uint32_t>(m_slots.size() - 1);
}

void Midphase::RecyclePendingRemovals() {
    for (const uint32_t index : m_pendingRemoval) {
        Slot& slot = m_slots[index];
        slot.occupied = false;
        slot.pair = MidphasePair{};
        // Generation bump invalidates every handle captured before the sync
        // point, so a recycled slot can never be addressed by a stale handle.
        ++slot.generation;
        m_freeSlots.push_back(index);
    }
    m_pendingRemoval.clear();
}

PairBindingSnapshot Midphase::CaptureBinding(
    const Collider& colliderA,
    const RigidBody& bodyA,
    const Collider& colliderB,
    const RigidBody& bodyB) {
    PairBindingSnapshot binding;
    binding.colliderIdentityA = colliderA.Identity();
    binding.colliderIdentityB = colliderB.Identity();
    binding.colliderRevisionA = colliderA.Revision();
    binding.colliderRevisionB = colliderB.Revision();
    binding.poseRevisionA = bodyA.PoseRevision();
    binding.poseRevisionB = bodyB.PoseRevision();
    binding.structureRevisionA = bodyA.StructureRevision();
    binding.structureRevisionB = bodyB.StructureRevision();
    binding.materialA = colliderA.Material();
    binding.materialB = colliderB.Material();
    return binding;
}

void Midphase::ClearContactCaches(MidphasePair& pair) {
    for (std::size_t index = 0; index < pair.manifold.pointCount; ++index) {
        ContactPoint& point = pair.manifold.Point(index);
        point.accumulatedNormalImpulse = 0.0f;
        point.accumulatedTangentImpulse = glm::vec2(0.0f);
        point.cachedTangent1 = glm::vec3(0.0f);
        point.cachedTangent2 = glm::vec3(0.0f);
        point.cachedDt = 0.0f;
        point.isToiImpact = false;
        point.cacheFromToiImpact = false;
    }
}

void Midphase::PushEvent(
    std::vector<ContactEvent>& events,
    ContactEventType type,
    const PairKey& key) const {
    ContactEvent event;
    event.type = type;
    event.pair = key;
    event.fixedStepId = m_fixedStepId;
    event.queryEpoch = m_queryEpoch;
    events.push_back(event);
}

} // namespace Physics
} // namespace Runtime
