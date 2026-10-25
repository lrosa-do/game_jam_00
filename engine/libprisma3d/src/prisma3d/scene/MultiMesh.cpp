#include "prisma3d/scene/MultiMesh.h"

#include "platform.h"

namespace prisma3d
{

MultiMesh::MultiMesh()
        : mId(),
          mMesh(),
          mUseColors(false),
          mUseCustomData(false),
          mDissolve(false),
          mInstanceCount(0),
          mVisibleInstanceCount(-1),
          mCustomAabb()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server) mId = server->multimeshCreate();
}

MultiMesh::~MultiMesh()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server && mId.valid()) server->multimeshFree(mId);
}

MultiMeshId MultiMesh::getId() const
{
    return mId;
}

void MultiMesh::setMesh(MeshId mesh)
{
    mMesh = mesh;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->multimeshSetMesh(mId, mesh);
}

MeshId MultiMesh::getMesh() const
{
    return mMesh;
}

bool MultiMesh::setUseColors(bool enable)
{
    if (mInstanceCount > 0)
    {
        log_error("multimesh: the instance count must be 0 to toggle whether colors are used");
        return false;
    }
    mUseColors = enable;
    return true;
}

bool MultiMesh::getUseColors() const
{
    return mUseColors;
}

bool MultiMesh::setUseCustomData(bool enable)
{
    if (mInstanceCount > 0)
    {
        log_error("multimesh: the instance count must be 0 to toggle whether custom data is used");
        return false;
    }
    mUseCustomData = enable;
    return true;
}

bool MultiMesh::getUseCustomData() const
{
    return mUseCustomData;
}

void MultiMesh::setInstanceCount(int count)
{
    if (count < 0) return;

    RenderingServer* server = RenderingServer::singleton();
    if (server)
        server->multimeshAllocateData(mId, static_cast<std::uint32_t>(count), mUseColors,
                mUseCustomData);
    mInstanceCount = count;
    if (mVisibleInstanceCount > count) mVisibleInstanceCount = count;
}

int MultiMesh::getInstanceCount() const
{
    return mInstanceCount;
}

bool MultiMesh::setVisibleInstanceCount(int count)
{
    if (count < -1 || count > mInstanceCount) return false;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->multimeshSetVisibleInstances(mId, count);
    mVisibleInstanceCount = count;
    return true;
}

int MultiMesh::getVisibleInstanceCount() const
{
    return mVisibleInstanceCount;
}

void MultiMesh::setDissolve(bool enable)
{
    mDissolve = enable;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->multimeshSetDissolve(mId, enable);
}

bool MultiMesh::getDissolve() const
{
    return mDissolve;
}

void MultiMesh::setInstanceTransform(int instance, const Math::Mat4& transform)
{
    if (instance < 0) return;

    RenderingServer* server = RenderingServer::singleton();
    if (server)
        server->multimeshInstanceSetTransform(mId, static_cast<std::uint32_t>(instance), transform);
}

Math::Mat4 MultiMesh::getInstanceTransform(int instance) const
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server || instance < 0) return Math::Mat4::Identity();
    return server->multimeshInstanceGetTransform(mId, static_cast<std::uint32_t>(instance));
}

void MultiMesh::setInstanceColor(int instance, const Math::Vec4& color)
{
    if (instance < 0) return;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->multimeshInstanceSetColor(mId, static_cast<std::uint32_t>(instance), color);
}

Math::Vec4 MultiMesh::getInstanceColor(int instance) const
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server || instance < 0) return Math::Vec4();
    return server->multimeshInstanceGetColor(mId, static_cast<std::uint32_t>(instance));
}

void MultiMesh::setInstanceCustomData(int instance, const Math::Vec4& data)
{
    if (instance < 0) return;

    RenderingServer* server = RenderingServer::singleton();
    if (server)
        server->multimeshInstanceSetCustomData(mId, static_cast<std::uint32_t>(instance), data);
}

Math::Vec4 MultiMesh::getInstanceCustomData(int instance) const
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server || instance < 0) return Math::Vec4();
    return server->multimeshInstanceGetCustomData(mId, static_cast<std::uint32_t>(instance));
}

bool MultiMesh::setBuffer(const float* data, std::size_t count)
{
    if (mInstanceCount == 0) return false;

    RenderingServer* server = RenderingServer::singleton();
    return server && server->multimeshSetBuffer(mId, data, count);
}

bool MultiMesh::getBuffer(ct::Vector<float>& out) const
{
    RenderingServer* server = RenderingServer::singleton();
    out.clear();
    return server && server->multimeshGetBuffer(mId, out);
}

void MultiMesh::setCustomAabb(const Math::Box& aabb)
{
    mCustomAabb = aabb;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->multimeshSetCustomAabb(mId, aabb);
}

Math::Box MultiMesh::getCustomAabb() const
{
    return mCustomAabb;
}

Math::Box MultiMesh::getAabb() const
{
    RenderingServer* server = RenderingServer::singleton();
    return server ? server->multimeshGetAabb(mId) : Math::Box();
}

} // namespace prisma3d
