#pragma once

#include <ct/string.hpp>

namespace prisma3d
{

ct::String capitalizePropertyName(const ct::String& name);
const char* propertyLabelOverride(const ct::String& name);

} // namespace prisma3d
