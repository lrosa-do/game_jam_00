#include "prisma3d/editor/ClassIcons.h"

#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/NodeClassDB.h"

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

struct DefaultEntry
{
    const char* className;
    const char* iconName;
};

const DefaultEntry kDefaults[] = {
    { "Node", "Node" },
    { "MissingNode", "MissingNode" },
    { "Node3D", "Node3D" },
    { "VisualInstance3D", "VisualInstance3D" },
    { "MeshInstance3D", "MeshInstance3D" },
    { "SoftBody3D", "SoftBody3D" },
    { "MultiMeshInstance3D", "MultiMeshInstance3D" },
    { "ManualMesh3D", "MeshInstance3D" },
    { "ParticleSystem3D", "GPUParticles3D" },
    { "Camera3D", "Camera3D" },
    { "Light3D", "Light3D" },
    { "DirectionalLight3D", "DirectionalLight3D" },
    { "OmniLight3D", "OmniLight3D" },
    { "SpotLight3D", "SpotLight3D" },
    { "Skeleton3D", "Skeleton3D" },
    { "BoneAttachment3D", "BoneAttachment3D" },
    { "Ragdoll3D", "PhysicalBoneSimulator3D" },
    { "Dismember3D", "PhysicalBoneSimulator3D" },
    { "Decal3D", "Decal" },
    { "ReflectionProbe3D", "ReflectionProbe" },
    { "AudioListener3D", "AudioListener3D" },
    { "AudioStreamPlayer", "AudioStreamPlayer" },
    { "AudioStreamPlayer3D", "AudioStreamPlayer3D" },
    { "Path3D", "Path3D" },
    { "PathFollow3D", "PathFollow3D" },
    { "ParticleCollision3D", "GPUParticlesCollisionBox3D" },
    { "ParticleCollisionSphere3D", "GPUParticlesCollisionSphere3D" },
    { "ParticleCollisionBox3D", "GPUParticlesCollisionBox3D" },
    { "CollisionShape3D", "CollisionShape3D" },
    { "PhysicsBody3D", "PhysicsBody3D" },
    { "StaticBody3D", "StaticBody3D" },
    { "AnimatableBody3D", "AnimatableBody3D" },
    { "RigidBody3D", "RigidBody3D" },
    { "CharacterBody3D", "CharacterBody3D" },
    { "CharacterRigidBody3D", "CharacterBody3D" },
    { "VehicleBody3D", "VehicleBody3D" },
    { "GroundVehicle3D", "VehicleBody3D" },
    { "CarDamage3D", "Node3D" },
    { "SkidMarks3D", "Node3D" },
    { "ImpactFx3D", "Node3D" },
    { "VehicleDriver3D", "Node" },
    { "BoostJet3D", "Node3D" },
    { "Traffic3D", "Node3D" },
    { "VehicleWheel3D", "VehicleWheel3D" },
    { "Joint3D", "Generic6DOFJoint3D" },
    { "PinJoint3D", "PinJoint3D" },
    { "HingeJoint3D", "HingeJoint3D" },
    { "SliderJoint3D", "SliderJoint3D" },
    { "ConeTwistJoint3D", "ConeTwistJoint3D" },
    { "Viewport", "SubViewport" },
    { "Window", "Window" },
    { "WorldEnvironment", "WorldEnvironment" },
    { "AnimationPlayer", "AnimationPlayer" },
};

const char* const kDisabledName = "Node";

const char* findDefault(const ct::String& className)
{
    for (const DefaultEntry& entry: kDefaults)
        if (className == entry.className) return entry.iconName;
    return nullptr;
}

} // namespace

ClassIcons::ClassIcons()
        : mDisabled(rt::IconId::None),
          mDisabledSet(false),
          mGeneration(EditorIcons::generation())
{
}

void ClassIcons::refresh() const
{
    const unsigned int generation = EditorIcons::generation();
    if (generation == mGeneration) return;
    mGeneration = generation;
    mCache.clear();
}

const char* ClassIcons::defaultIconName(const char* className)
{
    return className ? findDefault(ct::String(className)) : nullptr;
}

void ClassIcons::set(const char* className, rt::IconId icon)
{
    if (!className || !*className) return;
    mOverrides.put(ct::String(className), icon);
    mCache.clear();
}

bool ClassIcons::hasEntry(const ct::String& className) const
{
    return mOverrides.contains(className) || findDefault(className) != nullptr;
}

void ClassIcons::setDisabled(rt::IconId icon)
{
    mDisabled = icon;
    mDisabledSet = true;
    mCache.clear();
}

rt::IconId ClassIcons::disabled() const
{
    return mDisabledSet ? mDisabled : EditorIcons::get(kDisabledName);
}

rt::IconId ClassIcons::icon(const ct::String& className) const
{
    if (className.empty()) return rt::IconId::None;
    refresh();
    if (const rt::IconId* cached = mCache.find(className)) return *cached;
    const rt::IconId found = lookup(className, false);
    mCache.put(className, found);
    return found;
}

rt::IconId ClassIcons::icon(const Node* node) const
{
    return node ? icon(ct::String(node->getClassName())) : rt::IconId::None;
}

rt::IconId ClassIcons::lookup(const ct::String& className, bool skipDisabled) const
{
    if (const rt::IconId* entry = mOverrides.find(className)) return *entry;
    if (const char* name = findDefault(className)) return EditorIcons::get(name);

    if (NodeClassDB::classExists(className))
    {
        if (!skipDisabled && !NodeClassDB::canInstantiate(className)) return disabled();
        const ct::String parent = NodeClassDB::getParentClass(className);
        if (!parent.empty()) return lookup(parent, true);
    }
    return rt::IconId::None;
}

} // namespace prisma3d
