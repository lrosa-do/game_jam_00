#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "marine/Hydrofoil.h"

#include "dynamics/RigidBody.h"
#include "marine/WaterSurface.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr u32 kImmersionSamples = 8;
// Half-width of the blend across the surface, in chords.
constexpr f32 kSurfaceBlend = 0.1f;

Math::vec3 waterFlow(const Math::vec3& worldPoint, const void* userData)
{
    const WaterSurface* water = static_cast<const WaterSurface*>(userData);
    return water->flowVelocity(worldPoint);
}
} // namespace

Hydrofoil::Hydrofoil()
{
    mBase = mWing.description();
    rebuild();
}

Hydrofoil::Hydrofoil(const Wing::Description& description, const Airfoil& airfoil)
    : mBase(description)
{
    mWing.setAirfoil(airfoil);
    rebuild();
}

void Hydrofoil::setDescription(const Wing::Description& description)
{
    mBase = description;
    rebuild();
}

void Hydrofoil::setMaxDeflection(f32 angle)
{
    mMaxDeflection = isfinite(angle) ? Clamp(fabsf(angle), 0.0f, 1.5f) : 0.0f;
    setDeflection(mDeflection);
}

void Hydrofoil::setDeflection(f32 angle)
{
    const f32 clamped = isfinite(angle) ? Clamp(angle, -mMaxDeflection, mMaxDeflection) : 0.0f;
    if (clamped == mDeflection)
        return;
    mDeflection = clamped;
    rebuild();
}

void Hydrofoil::rebuild()
{
    Wing::Description d = mBase;
    d.rootIncidence += mDeflection;
    d.tipIncidence += mDeflection;
    if (d.elementCount * (d.mirrored ? 2u : 1u) > kMaxElements)
        d.elementCount = kMaxElements / (d.mirrored ? 2u : 1u);
    mWing.setDescription(d);
}

void Hydrofoil::measureImmersion(const RigidBody& body, const WaterSurface& water)
{
    const u32 count = mWing.elementCount();
    f32 wetArea = 0.0f;
    f32 totalArea = 0.0f;
    for (u32 i = 0; i < count && i < kMaxElements; ++i)
    {
        const Wing::Element& element = mWing.element(i);
        const f32 halfWidth = 0.5f * element.area / Max(element.chord, 1.0e-6f);
        const Math::vec3 centre = body.pointToWorld(element.position);
        const Math::vec3 span = body.directionToWorld(element.pitchAxis) * halfWidth;
        const Math::vec3 a = centre - span;
        const Math::vec3 b = centre + span;
        const f32 depthA = water.depth(a);
        const f32 depthB = water.depth(b);
        const f32 blend = Max(kSurfaceBlend * element.chord, 1.0e-4f);
        f32 wet = 0.0f;
        for (u32 s = 0; s < kImmersionSamples; ++s)
        {
            const f32 t = (static_cast<f32>(s) + 0.5f) / static_cast<f32>(kImmersionSamples);
            const f32 depth = Math::mix(depthA, depthB, t);
            wet += Clamp(0.5f + 0.5f * depth / blend, 0.0f, 1.0f);
        }
        mScales[i] = wet / static_cast<f32>(kImmersionSamples);
        wetArea += mScales[i] * element.area;
        totalArea += element.area;
    }
    mImmersion = totalArea > 0.0f ? wetArea / totalArea : 0.0f;
}

const WingState& Hydrofoil::computeForces(const RigidBody& body, const WaterSurface& water)
{
    measureImmersion(body, water);
    return mWing.computeForces(body, water.density(), &waterFlow, &water, mScales);
}

const WingState& Hydrofoil::apply(RigidBody& body, const WaterSurface& water)
{
    measureImmersion(body, water);
    return mWing.apply(body, water.density(), &waterFlow, &water, mScales);
}

} // namespace kinetix::Physics
