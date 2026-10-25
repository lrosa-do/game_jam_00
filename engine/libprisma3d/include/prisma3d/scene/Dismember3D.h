#pragma once

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/Ragdoll3D.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

struct DismemberSegment;

class Dismember3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(Dismember3D, Node3D);
    static void bindProperties();

    Dismember3D();
    ~Dismember3D() override;

    void setRagdoll(Ragdoll3D* ragdoll);
    Ragdoll3D* getRagdoll() const;
    void setRagdollPath(const ct::String& path);
    ct::String getRagdollPath() const;

    void setRadius(float radius);
    float getRadius() const;
    void setForce(float force);
    float getForce() const;

    bool detach(PhysicsRagdollPart part);
    bool isDetached(PhysicsRagdollPart part) const;
    int getDetachedCount() const;
    bool explode(const Math::Vec3& origin);
    bool isMeshCut() const;

protected:
    void notificationHandler(int what) override;

private:
    struct Cut
    {
        ct::String path;
        MeshId original;
        MeshId cut;
    };

    void resolveRagdoll();
    bool ensureActive();
    void cutMeshes(const bool* separated);
    void restoreMeshes();
    void destroyCuts();
    MeshId cutMesh(MeshId source, const ct::Vector<int>& partOfBone, const int* partBones,
            const ct::Vector<DismemberSegment>& segments) const;

    Ragdoll3D* mRagdoll;
    ct::Vector<Cut> mCuts;
    bool mMeshCut;
    ct::String mRagdollPath;
    float mRadius;
    float mForce;
};

} // namespace prisma3d
