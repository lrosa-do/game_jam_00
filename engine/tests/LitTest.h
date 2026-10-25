#pragma once

#include "prisma3d/render/RenderingServer.h"

inline prisma3d::EnvironmentSettings whiteAmbient()
{
    prisma3d::EnvironmentSettings environment;
    environment.ambientSource = prisma3d::AmbientSource::Color;
    environment.ambientColor = Math::Vec3(1.0f, 1.0f, 1.0f);
    environment.reflectionSource = prisma3d::ReflectionSource::Disabled;
    return environment;
}
