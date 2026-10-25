#include "prisma3d/scene/SceneTypes.h"

#include "prisma3d/scene/AnimationPlayer.h"
#include "prisma3d/scene/AudioListener3D.h"
#include "prisma3d/scene/AudioStreamPlayer.h"
#include "prisma3d/scene/AudioStreamPlayer3D.h"
#include "prisma3d/scene/BoneAttachment3D.h"
#include "prisma3d/scene/BoostJet3D.h"
#include "prisma3d/scene/Button.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/CarDamage3D.h"
#include "prisma3d/scene/CharacterBody3D.h"
#include "prisma3d/scene/CharacterRigidBody3D.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/ColorRect.h"
#include "prisma3d/scene/Control.h"
#include "prisma3d/scene/Decal3D.h"
#include "prisma3d/scene/Destructible3D.h"
#include "prisma3d/scene/Forest3D.h"
#include "prisma3d/scene/Grass3D.h"
#include "prisma3d/scene/GroundVehicle3D.h"
#include "prisma3d/scene/Hair3D.h"
#include "prisma3d/scene/ImpactFx3D.h"
#include "prisma3d/scene/Joint3D.h"
#include "prisma3d/scene/Label.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/LightmapGI.h"
#include "prisma3d/scene/LineEdit.h"
#include "prisma3d/scene/ManualMesh3D.h"
#include "prisma3d/scene/MissingNode.h"
#include "prisma3d/scene/MultiMeshInstance3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Ocean3D.h"
#include "prisma3d/scene/ParticleCollision3D.h"
#include "prisma3d/scene/ParticleSystem3D.h"
#include "prisma3d/scene/Path3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/PlanarReflector3D.h"
#include "prisma3d/scene/Ragdoll3D.h"
#include "prisma3d/scene/Dismember3D.h"
#include "prisma3d/scene/NavigationRegion3D.h"
#include "prisma3d/scene/ReflectionProbe3D.h"
#include "prisma3d/scene/RibbonTrail3D.h"
#include "prisma3d/scene/Road3D.h"
#include "prisma3d/scene/Skeleton3D.h"
#include "prisma3d/scene/SkeletonModifier3D.h"
#include "prisma3d/scene/SkidMarks3D.h"
#include "prisma3d/scene/SoftBody3D.h"
#include "prisma3d/scene/SpringBoneCollision3D.h"
#include "prisma3d/scene/SpringBoneSimulator3D.h"
#include "prisma3d/scene/Terrain3D.h"
#include "prisma3d/scene/TextureRect.h"
#include "prisma3d/scene/Traffic3D.h"
#include "prisma3d/scene/Tree3D.h"
#include "prisma3d/scene/Vehicle3D.h"
#include "prisma3d/scene/VehicleDriver3D.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/Water3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

namespace prisma3d
{

void registerSceneTypes()
{
    static bool registered = false;
    if (registered) return;
    registered = true;

    NodeClassDB::registerClass<Node>();
    NodeClassDB::registerAbstractClass<MissingNode>();
    NodeClassDB::registerClass<Node3D>();
    NodeClassDB::registerAbstractClass<VisualInstance3D>();
    NodeClassDB::registerClass<MeshInstance3D>();
    NodeClassDB::registerClass<SoftBody3D>();
    NodeClassDB::registerClass<DebrisMesh3D>();
    NodeClassDB::registerClass<ManualMesh3D>();
    NodeClassDB::registerClass<MultiMeshInstance3D>();
    NodeClassDB::registerClass<ParticleSystem3D>();
    NodeClassDB::registerClass<RibbonTrail3D>();
    NodeClassDB::registerClass<Hair3D>();
    NodeClassDB::registerClass<Tree3D>();
    NodeClassDB::registerClass<Road3D>();
    NodeClassDB::registerClass<Ocean3D>();
    NodeClassDB::registerClass<Water3D>();
    NodeClassDB::registerClass<Camera3D>();
    NodeClassDB::registerAbstractClass<Light3D>();
    NodeClassDB::registerClass<DirectionalLight3D>();
    NodeClassDB::registerClass<OmniLight3D>();
    NodeClassDB::registerClass<SpotLight3D>();
    NodeClassDB::registerClass<Skeleton3D>();
    NodeClassDB::registerClass<BoneAttachment3D>();
    NodeClassDB::registerAbstractClass<SkeletonModifier3D>();
    NodeClassDB::registerClass<SpringBoneSimulator3D>();
    NodeClassDB::registerAbstractClass<SpringBoneCollision3D>();
    NodeClassDB::registerClass<SpringBoneCollisionSphere3D>();
    NodeClassDB::registerClass<SpringBoneCollisionCapsule3D>();
    NodeClassDB::registerClass<SpringBoneCollisionPlane3D>();
    NodeClassDB::registerClass<Ragdoll3D>();
    NodeClassDB::registerClass<Dismember3D>();
    NodeClassDB::registerClass<NavigationRegion3D>();
    NodeClassDB::registerClass<Decal3D>();
    NodeClassDB::registerClass<ReflectionProbe3D>();
    NodeClassDB::registerClass<LightmapGI>();
    NodeClassDB::registerClass<PlanarReflector3D>();
    NodeClassDB::registerClass<AudioListener3D>();
    NodeClassDB::registerClass<Control>();
    NodeClassDB::registerClass<Label>();
    NodeClassDB::registerClass<ColorRect>();
    NodeClassDB::registerClass<TextureRect>();
    NodeClassDB::registerClass<Button>();
    NodeClassDB::registerClass<LineEdit>();
    NodeClassDB::registerClass<AudioStreamPlayer3D>();
    NodeClassDB::registerClass<Path3D>();
    NodeClassDB::registerClass<PathFollow3D>();
    NodeClassDB::registerClass<Terrain3D>();
    NodeClassDB::registerClass<Grass3D>();
    NodeClassDB::registerClass<Forest3D>();
    NodeClassDB::registerAbstractClass<ParticleCollision3D>();
    NodeClassDB::registerClass<ParticleCollisionSphere3D>();
    NodeClassDB::registerClass<ParticleCollisionBox3D>();
    NodeClassDB::registerClass<CollisionShape3D>();
    NodeClassDB::registerAbstractClass<PhysicsBody3D>();
    NodeClassDB::registerClass<StaticBody3D>();
    NodeClassDB::registerClass<AnimatableBody3D>();
    NodeClassDB::registerClass<RigidBody3D>();
    NodeClassDB::registerClass<CharacterBody3D>();
    NodeClassDB::registerClass<CharacterRigidBody3D>();
    NodeClassDB::registerClass<VehicleBody3D>();
    NodeClassDB::registerClass<GroundVehicle3D>();
    NodeClassDB::registerClass<CarDamage3D>();
    NodeClassDB::registerClass<SkidMarks3D>();
    NodeClassDB::registerClass<ImpactFx3D>();
    NodeClassDB::registerClass<VehicleDriver3D>();
    NodeClassDB::registerClass<BoostJet3D>();
    NodeClassDB::registerClass<Traffic3D>();
    NodeClassDB::registerClass<DestructibleBody3D>();
    NodeClassDB::registerClass<DebrisBody3D>();
    NodeClassDB::registerClass<VehicleWheel3D>();
    NodeClassDB::registerAbstractClass<Joint3D>();
    NodeClassDB::registerClass<PinJoint3D>();
    NodeClassDB::registerClass<HingeJoint3D>();
    NodeClassDB::registerClass<SliderJoint3D>();
    NodeClassDB::registerClass<DistanceJoint3D>();
    NodeClassDB::registerClass<FixedJoint3D>();
    NodeClassDB::registerClass<PistonJoint3D>();
    NodeClassDB::registerClass<UniversalJoint3D>();
    NodeClassDB::registerClass<WheelJoint3D>();
    NodeClassDB::registerClass<ConeTwistJoint3D>();
    NodeClassDB::registerClass<MouseJoint3D>();
    NodeClassDB::registerClass<Viewport>();
    NodeClassDB::registerClass<Window>();
    NodeClassDB::registerClass<WorldEnvironment>();
    NodeClassDB::registerClass<AnimationPlayer>();
    NodeClassDB::registerClass<AudioStreamPlayer>();
}

} // namespace prisma3d
