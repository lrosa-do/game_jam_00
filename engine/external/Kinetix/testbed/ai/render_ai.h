#pragma once

// render_ai.h - agents drawn as a triangle along forward inside their radius circle.

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace testbed
{

struct AgentStyle
{
    float r = 0.4f;
    float g = 0.75f;
    float b = 1.0f;
    float radius = 0.5f;
};

void setupLightingAI();

// Drawn in the XZ plane.
void drawAgent(const kinetix::Math::vec3& position, const kinetix::Math::vec3& forward,
               const AgentStyle& style);

void drawVelocity(const kinetix::Math::vec3& position, const kinetix::Math::vec3& velocity,
                  float scale = 0.35f);

void drawPath(const ct::Vector<kinetix::Math::vec3>& corners, float r, float g, float b);

void drawNavMeshSolid(const ct::Vector<kinetix::Math::vec3>& triangles);

void drawGoalMarker(const kinetix::Math::vec3& position, float time);

} // namespace testbed
