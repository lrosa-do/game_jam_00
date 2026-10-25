#include "prisma3d/scene/ScriptServer.h"

namespace prisma3d
{

ScriptServer* ScriptServer::sSingleton = nullptr;

ScriptServer::ScriptServer()
{
    if (!sSingleton) sSingleton = this;
}

ScriptServer::~ScriptServer()
{
    if (sSingleton == this) sSingleton = nullptr;
}

ScriptServer* ScriptServer::singleton()
{
    return sSingleton;
}

} // namespace prisma3d
