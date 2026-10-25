#pragma once

#include <ct/string.hpp>

namespace prisma3d
{

class Node;

class ScriptServer
{
public:
    virtual ~ScriptServer();
    ScriptServer(const ScriptServer&) = delete;
    ScriptServer& operator=(const ScriptServer&) = delete;

    static ScriptServer* singleton();

    virtual void attachScript(Node* node, const ct::String& path) = 0;
    virtual void update(double delta) = 0;

protected:
    ScriptServer();

private:
    static ScriptServer* sSingleton;
};

} // namespace prisma3d
