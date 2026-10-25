#ifndef KINETIX_PHYSICS_SOFT_BODY_H
#define KINETIX_PHYSICS_SOFT_BODY_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include "collision/CollisionFilter.h"
#include "collision/Narrowphase.h"

#include <ct/vector.hpp>

namespace kinetix::Physics
{

class PhysicsWorld;
 
class SoftBody
{
public:
    struct Particle
    {
        Math::vec3 position{0.0f};

        Math::vec3 previousPosition{0.0f};
        Math::vec3 velocity{0.0f};
        // Velocity before updateVelocities() rewrote it from the positions;
        // restitution is decided on this one.
        Math::vec3 previousVelocity{0.0f};

        f32 invMass = 0.0f;
    };

    struct DistanceConstraint
    {
        u32 a = 0;
        u32 b = 0;
        f32 restLength = 0.0f;
        f32 compliance = 0.0f;
    };

    // Two triangles sharing the edge a-b, with c and d the opposite vertices;
    // holds the angle between their normals at its rest value. A distance
    // across the diagonal turns near-rigid when a fold is pressed flat.
    struct DihedralBendConstraint
    {
        u32 a = 0;
        u32 b = 0;
        u32 c = 0;
        u32 d = 0;
        f32 compliance = 0.0f;
        f32 initialAngle = 0.0f;
    };

 
    struct LongRangeAttachment
    {
        u32 anchor = 0;
        u32 vertex = 0;
        f32 maxDistance = 0.0f;
    };

    // Whether a particle is resting against something as of the last step(),
    // and against what: determineContactPlanes()'s per-particle result.
    struct Contact
    {
        bool active = false;
        Math::vec3 normal{0.0f, 1.0f, 0.0f};
        u32 bodyId = 0xFFFFFFFFu;
    };
    Contact contact(u32 index) const;

    void clear();

    // Particles at `positions`, sharing `totalMass` equally. Any previous
    // topology is dropped.
    void setParticles(const Math::vec3* positions, u32 count, f32 totalMass);

    // Redistributes `totalMass` evenly across every particle that is not
    // pinned (invMass 0, see setPinned()); positions, velocities and
    // constraints are left alone. Same formula as setParticles(), but live.
    void setTotalMass(f32 totalMass);

    // How bending resistance is built. Distance (default): a distance
    // constraint between the two vertices opposite each shared edge -
    // approximate, but stable in a crumpled pile. Dihedral holds the true
    // angle between the triangles, but without self collision a fully folded
    // pile traps it in degenerate configurations that never settle.
    enum class BendType : u8
    {
        None,
        Distance,
        Dihedral
    };

    // Structural constraints along every triangle edge, and a bending
    // constraint across each edge two triangles share. Rest lengths and rest
    // angles come from the positions as they stand, so the mesh is its own
    // rest pose. A negative bend compliance skips bending entirely.
    // Replaces every constraint a previous build made. A triangle with an
    // index past particleCount() or a repeated index is skipped and
    // reported.
    void buildFromMesh(const u32* indices, u32 indexCount, f32 structuralCompliance,
                       f32 bendCompliance, BendType bendType = BendType::Distance);

    void addDistanceConstraint(u32 a, u32 b, f32 compliance);

    // invMass 0. Pin before building attachments: they measure from whatever
    // is pinned at that moment.
    void setPinned(u32 index, bool pinned);
    bool pinned(u32 index) const;

    // One attachment per free particle, to the pinned particle nearest it in
    // the current pose. Does nothing when nothing is pinned.
    void buildAttachments(f32 maxDistanceMultiplier = 1.0f);

    // Splits `dt` into `substeps` slices, each predicted, projected once and
    // read back. Sub-stepping, not iterating: see step()'s own note.
    void step(f32 dt, u32 substeps);

    void setGravity(const Math::vec3& gravity)
    {
        mGravity = gravity;
    }
    const Math::vec3& gravity() const
    {
        return mGravity;
    }
    // Per-second exponential velocity damping: velocity *= pow(damping, dt).
    // 1 disables it.
    void setDamping(f32 damping)
    {
        mDamping = damping;
    }
    f32 damping() const
    {
        return mDamping;
    }
    // Maximum particle speed (safety limit).
    void setMaxLinearVelocity(f32 velocity)
    {
        mMaxLinearVelocity = Math::max(velocity, 0.0f);
    }
    f32 maxLinearVelocity() const
    {
        return mMaxLinearVelocity;
    }
    void setWind(const Math::vec3& wind)
    {
        mWind = wind;
    }
    // Gas pressure inside a closed mesh: the coefficient is n*R*T, so the
    // force on a triangle is that over the enclosed volume times the
    // triangle's area, along its normal. Keeps a ball a ball: edges alone let
    // a closed shell fold in on itself. 0 is off; an open mesh has no volume.
    void setPressure(f32 pressure)
    {
        mPressure = Math::max(pressure, 0.0f);
    }
    f32 pressure() const
    {
        return mPressure;
    }
    // Six times the volume the faces enclose, signed by their winding.
    f32 volumeTimesSix() const;
    // Non-owning world used for particle contacts. Shapes, transforms,
    // velocities and collision filtering come from the engine's bodies.
    void setCollisionWorld(const PhysicsWorld* world)
    {
        mCollisionWorld = world;
    }
    void setCollisionFilter(const CollisionFilter& filter)
    {
        mCollisionQuery.collision = filter;
    }
    // Excluded from every collision query this body makes, e.g. the body a
    // splash of particles came out of.
    void setIgnoredBody(u32 bodyId)
    {
        mCollisionQuery.ignoredBody = bodyId;
    }
    // Kept off the surface by this much, so a sheet does not shimmer with its
    // own thickness against the collider it rests on.
    void setCollisionMargin(f32 margin)
    {
        mCollisionMargin = Math::max(margin, 0.0f);
    }
    u32 particleCount() const
    {
        return static_cast<u32>(mParticles.size());
    }
    const Particle& particle(u32 index) const
    {
        return mParticles[index];
    }
    Particle& particle(u32 index)
    {
        return mParticles[index];
    }
    const ct::Vector<Particle>& particles() const
    {
        return mParticles;
    }
    u32 constraintCount() const
    {
        return static_cast<u32>(mConstraints.size());
    }
    u32 bendConstraintCount() const
    {
        return static_cast<u32>(mBendConstraints.size());
    }
    u32 faceCount() const
    {
        return static_cast<u32>(mFaces.size() / 3);
    }
    u32 attachmentCount() const
    {
        return static_cast<u32>(mAttachments.size());
    }

    // Longest constraint as a fraction of its rest length, over the whole
    // body. 1 is inextensible; a sheet that reads 1.4 is stretched 40% and
    // looks like rubber, which is the number LRA exists to hold down.
    f32 worstStretch() const;

private:
    // One collision plane per particle, held fixed across every substep of a
    // step: contacts are determined once per update, not with a fresh normal
    // each substep.
    struct ContactPlane
    {
        Math::vec3 normal{0.0f, 1.0f, 0.0f};
        f32 offset = 0.0f;
        u32 bodyId = 0;
        f32 friction = 0.0f;
        f32 restitution = 0.0f;
        bool active = false;
    };

    void applyPressure(f32 dt);
    void predict(f32 dt);
    void projectDistanceConstraints(f32 dt);
    void projectDihedralBendConstraints(f32 dt);
    void projectAttachments();
    void determineContactPlanes(f32 dt);
    void applyContactPlanes(f32 dt);
    void updateVelocities(f32 dt);

    ct::Vector<Particle> mParticles;
    ct::Vector<u32> mFaces;
    f32 mPressure = 0.0f;
    ct::Vector<DistanceConstraint> mConstraints;
    ct::Vector<DihedralBendConstraint> mBendConstraints;
    ct::Vector<LongRangeAttachment> mAttachments;
    ct::Vector<ContactPlane> mContactPlanes;
    const PhysicsWorld* mCollisionWorld = nullptr;
    QueryFilter mCollisionQuery;
    ct::Vector<u32> mCollisionCandidates;
    ct::Vector<ContactManifold> mCollisionManifolds;
    Math::vec3 mGravity{0.0f, -9.81f, 0.0f};
    Math::vec3 mWind{0.0f};
    // Linear damping 0.1 / second: exp(-0.1) retained over one second.
    f32 mDamping = 0.9048374f;
    f32 mMaxLinearVelocity = 500.0f;
    f32 mCollisionMargin = 0.01f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_SOFT_BODY_H
