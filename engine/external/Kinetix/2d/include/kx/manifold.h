#pragma once

#include "math.h"
#include <stdint.h>

#include "common.h"

namespace kx
{

    constexpr int32_t kMaxManifoldPoints = 2;

    struct ContactFeature
    {
        enum Type
        {
            kVertex = 0,
            kFace = 1
        };

        uint8_t indexA;
        uint8_t indexB;
        uint8_t typeA;
        uint8_t typeB;
    };

    union ContactID
    {
        ContactFeature cf;
        uint32_t key;
    };

    // Zero-initialised so a manifold that is declared and only partially
    // filled by a collide routine never carries garbage impulses into the
    // warm start.
    struct ManifoldPoint
    {
        vec2 localPoint = vec2(0.0f, 0.0f);
        float normalImpulse = 0.0f;
        float tangentImpulse = 0.0f;
        ContactID id = {};
    };

    struct Manifold
    {
        enum Type
        {
            kCircles,
            kFaceA,
            kFaceB
        };

        ManifoldPoint points[kMaxManifoldPoints];
        vec2 localNormal = vec2(0.0f, 0.0f);
        vec2 localPoint = vec2(0.0f, 0.0f);
        Type type = kCircles;
        int32_t pointCount = 0;
    };

    struct WorldManifold
    {
        void Initialize(const Manifold *manifold,
                        const Transform &xfA, float radiusA,
                        const Transform &xfB, float radiusB);

        vec2 normal;
        vec2 points[kMaxManifoldPoints];
        float separations[kMaxManifoldPoints];
    };

} 