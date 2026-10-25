#pragma once

#include "shapes.h"

namespace kx
{

struct Shape;

// Linear convex shape cast. This is a direct C++ adaptation of Box2D's
// GJK shape-cast (Erin Catto, MIT), used here for character motion.
// Shape B moves by translationB while shape A stays fixed.
struct ShapeCastInput
{
    const Shape* shapeA = nullptr;
    Transform transformA;
    const Shape* shapeB = nullptr;
    Transform transformB;
    vec2 translationB = vec2(0.0f, 0.0f);
};

struct ShapeCastOutput
{
    vec2 point = vec2(0.0f, 0.0f);
    vec2 normal = vec2(0.0f, 0.0f);
    float fraction = 1.0f;
    int iterations = 0;
    // Set when the cast returned false because the shapes already overlap
    // (within their skin) at the start; point and normal are not filled then.
    bool initialOverlap = false;
};

// Returns false on a miss or when the shapes start overlapped (output.initialOverlap).
// Rotations are fixed during the cast.
bool ShapeCast(const ShapeCastInput& input, ShapeCastOutput& output);

} // namespace kx
