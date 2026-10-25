#pragma once

#include "prisma3d/render/RenderingServer.h"

#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

class MultiMesh
{
public:
    MultiMesh();
    ~MultiMesh();
    MultiMesh(const MultiMesh&) = delete;
    MultiMesh& operator=(const MultiMesh&) = delete;

    MultiMeshId getId() const;

    void setMesh(MeshId mesh);
    MeshId getMesh() const;
    bool setUseColors(bool enable);
    bool getUseColors() const;
    bool setUseCustomData(bool enable);
    bool getUseCustomData() const;
    void setInstanceCount(int count);
    int getInstanceCount() const;
    bool setVisibleInstanceCount(int count);
    int getVisibleInstanceCount() const;
    void setDissolve(bool enable);
    bool getDissolve() const;
    void setInstanceTransform(int instance, const Math::Mat4& transform);
    Math::Mat4 getInstanceTransform(int instance) const;
    void setInstanceColor(int instance, const Math::Vec4& color);
    Math::Vec4 getInstanceColor(int instance) const;
    void setInstanceCustomData(int instance, const Math::Vec4& data);
    Math::Vec4 getInstanceCustomData(int instance) const;
    bool setBuffer(const float* data, std::size_t count);
    bool getBuffer(ct::Vector<float>& out) const;
    void setCustomAabb(const Math::Box& aabb);
    Math::Box getCustomAabb() const;
    Math::Box getAabb() const;

private:
    MultiMeshId mId;
    MeshId mMesh;
    bool mUseColors;
    bool mUseCustomData;
    bool mDissolve;
    int mInstanceCount;
    int mVisibleInstanceCount;
    Math::Box mCustomAabb;
};

} // namespace prisma3d
