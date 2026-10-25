#ifndef KINETIX_PHYSICS_MARINE_PRISMBUOYANCY_H
#define KINETIX_PHYSICS_MARINE_PRISMBUOYANCY_H

#include "marine/HullMesh.h"
#include "marine/WaterEffects.h"

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::Physics
{

class RigidBody;
class WaterSurface;

// Buoyancy and water drag from the triangles of a hull mesh, each treated
// as a face that the water presses on and drags.
//
// Every triangle is split into six around its centre and the midpoints of
// its edges, and every piece is cut at the waterline, so only what is under
// the surface counts. On a submerged piece the pressure force is the volume
// of the prism that stands on the piece and reaches up to the surface - the
// depth of each corner times rho * g, laid out along the piece's normal -
// which is the integral of the hydrostatic pressure over the piece; it acts
// along the normal. The drag is rho / 2 * area * v^2 times how squarely the
// piece faces the flow, along the normal against the flow, so a bottom
// trimmed bow-up lifts as it runs and a transom moving astern pushes back.
// The force of each piece goes to the vertex it hangs from and is applied
// to the body there, in the world; a vertex's velocity is the body's at
// that point.
class PrismBuoyancy
{
public:
    // What a triangle contributes: pressure and drag, drag alone, or
    // pressure alone.
    enum class Type : u8
    {
        Full,
        DragOnly,
        Dragless
    };

    PrismBuoyancy() = default;

    void setBody(RigidBody* body)
    {
        mBody = body;
    }
    RigidBody* body() const
    {
        return mBody;
    }
    void setHull(const HullMesh& hull);
    const HullMesh& hull() const
    {
        return mHull;
    }
    void setWater(const WaterSurface* water)
    {
        mWater = water;
    }
    const WaterSurface* water() const
    {
        return mWater;
    }
    void setEffects(WaterEffects* effects)
    {
        mEffects = effects;
    }
    void setType(Type type)
    {
        mType = type;
    }
    Type type() const
    {
        return mType;
    }

    // Sinking: the pressure is left out and only the drag acts.
    void setSink(bool sink)
    {
        mSink = sink;
    }
    bool sink() const
    {
        return mSink;
    }
    // Whether the next applyForces() sends its splash, ripple and spray to
    // the effects. Once a frame is enough; applyForces() clears it.
    void setEmitEffects(bool emit)
    {
        mEmitEffects = emit;
    }
    bool emitEffects() const
    {
        return mEmitEffects;
    }

    // One pass over the hull, and with the effects on, the splash and
    // ripple of every vertex just under the surface and moving.
    void applyForces();

    // What the last pass did.
    const Math::vec3& force() const
    {
        return mForce;
    }
    const Math::vec3& torque() const
    {
        return mTorque;
    }
    // How deep the deepest vertex is under the surface; 0 clear of the water.
    f32 draft() const
    {
        return mDraft;
    }
    u32 wetTriangles() const
    {
        return mWetTriangles;
    }

private:
    f32 tetrahedronVolume(const Math::vec3& o, const Math::vec3& a, const Math::vec3& b,
                          const Math::vec3& c) const;
    Math::vec3 submergedForce(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c,
                              Math::vec3 velocity, Type type);
    Math::vec3 pieceForce(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c,
                          const Math::vec3& velocity, Type type);
    void accumulateTriangle(u32 a, u32 b, u32 c, Type type);
    f32 surfaceHeight(const Math::vec3& at) const;
    Math::vec3 surfaceVelocity(const Math::vec3& at) const;

    RigidBody* mBody = nullptr;
    HullMesh mHull;
    const WaterSurface* mWater = nullptr;
    WaterEffects* mEffects = nullptr;
    Type mType = Type::Full;
    bool mSink = false;
    bool mEmitEffects = false;

    // The vertices this pass: where each is, how fast it moves, what it
    // collected.
    ct::Vector<Math::vec3> mPositions;
    ct::Vector<Math::vec3> mVelocities;
    ct::Vector<Math::vec3> mForces;
    f32 mPressurePerMetre = 9810.0f; // rho * g
    f32 mHalfDensity = 500.0f;       // rho / 2
    Math::vec3 mForce{0.0f};
    Math::vec3 mTorque{0.0f};
    f32 mDraft = 0.0f;
    u32 mWetTriangles = 0;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MARINE_PRISMBUOYANCY_H
