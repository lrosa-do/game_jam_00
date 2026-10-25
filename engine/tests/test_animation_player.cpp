#include "Check.h"

#include "prisma3d/scene/AnimationPlayer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Skeleton3D.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const float kTolerance = 1e-4f;
const float kHalfPi = 1.57079632679f;
const double kStep = 0.25;

bool near(float a, float b)
{
    return fabsf(a - b) <= kTolerance;
}

bool nearVec3(const Math::Vec3& a, float x, float y, float z)
{
    return near(a.x, x) && near(a.y, y) && near(a.z, z);
}

bool nearAngleZ(const Math::Quaternion& q, float degrees)
{
    const Math::Quaternion expected = Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 0.0f, 1.0f),
            degrees * kHalfPi / 90.0f);
    const float dot = fabsf(Math::Quaternion::Dot(q.Normalized(), expected));
    return dot >= 1.0f - kTolerance;
}

Math::Mat4 translation(float x, float y, float z)
{
    return Math::Mat4::Translation(Math::Vec3(x, y, z));
}

Skeleton chain()
{
    Skeleton skeleton;
    skeleton.addBone("root", -1, translation(0.0f, 0.0f, 0.0f), translation(0.0f, 0.0f, 0.0f));
    skeleton.addBone("spine", 0, translation(0.0f, 1.0f, 0.0f), translation(0.0f, -1.0f, 0.0f));
    skeleton.addBone("head", 1, translation(0.0f, 1.0f, 0.0f), translation(0.0f, -2.0f, 0.0f));
    skeleton.finalize();
    return skeleton;
}

BoneTrack track(int bone)
{
    BoneTrack result;
    result.bone = bone;
    return result;
}

void key(BoneTrack& track, float time, const Math::Vec3& position, float degreesZ)
{
    track.times.push_back(time);
    track.positions.push_back(position);
    track.rotations.push_back(Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 0.0f, 1.0f),
            degreesZ * kHalfPi / 90.0f));
    track.scales.push_back(Math::Vec3(1.0f, 1.0f, 1.0f));
}

AnimationClip walkClip()
{
    AnimationClip clip;
    clip.setName("walk");
    clip.setDuration(1.0f);
    clip.setLoopMode(LoopMode::Linear);
    BoneTrack spine = track(1);
    key(spine, 0.0f, Math::Vec3(0.0f, 1.0f, 0.0f), 0.0f);
    key(spine, 1.0f, Math::Vec3(0.0f, 1.0f, 0.0f), 90.0f);
    clip.tracks().push_back(spine);
    return clip;
}

AnimationClip liftClip(LoopMode mode)
{
    AnimationClip clip;
    clip.setName("lift");
    clip.setDuration(1.0f);
    clip.setLoopMode(mode);
    BoneTrack spine = track(1);
    key(spine, 0.0f, Math::Vec3(0.0f, 1.0f, 0.0f), 0.0f);
    key(spine, 1.0f, Math::Vec3(0.0f, 3.0f, 0.0f), 0.0f);
    clip.tracks().push_back(spine);
    return clip;
}

AnimationClip nodClip()
{
    AnimationClip clip;
    clip.setName("nod");
    clip.setDuration(0.5f);
    BoneTrack head = track(2);
    key(head, 0.0f, Math::Vec3(0.0f, 1.0f, 0.0f), 0.0f);
    key(head, 0.5f, Math::Vec3(0.0f, 1.0f, 0.0f), 45.0f);
    clip.tracks().push_back(head);
    return clip;
}

struct Scene
{
    SceneTree tree;
    Skeleton3D* skeleton;
    AnimationPlayer* player;

    Scene()
            : tree(),
              skeleton(nullptr),
              player(nullptr)
    {
        tree.initialize();
        Node3D holder;
        skeleton = holder.createChild<Skeleton3D>();
        skeleton->setName("Skel");
        skeleton->setSkeleton(chain());
        skeleton->reparent(tree.getRoot());
        player = skeleton->createChild<AnimationPlayer>();
    }

    void step(double seconds)
    {
        tree.process(seconds);
    }

    void start(const char* name, double customBlend = -1.0)
    {
        player->play(ct::String(name), customBlend);
        tree.process(kStep);
    }
};

void testSkeletonNode()
{
    Skeleton3D skeleton;
    CHECK(skeleton.isClass("Skeleton3D"));
    CHECK(skeleton.isClass(NodeClass::Node3D));
    CHECK(skeleton.getBoneCount() == 0);
    skeleton.setSkeleton(chain());
    CHECK(skeleton.getBoneCount() == 3);
    CHECK(skeleton.findBone("head") == 2);
    CHECK(skeleton.getBoneName(1) == "spine");
    CHECK(skeleton.getBoneName(7).empty());
    CHECK(skeleton.getBoneParent(2) == 1);
    CHECK(skeleton.getBoneParent(0) == -1);
    CHECK(skeleton.getBoneParent(9) == -1);
    CHECK(nearVec3(skeleton.getBoneRest(2).GetTranslation(), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(skeleton.getBoneGlobalRest(2).GetTranslation(), 0.0f, 2.0f, 0.0f));
    CHECK(nearVec3(skeleton.getBoneRestPose(1).position, 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(skeleton.getBoneGlobalPose(2).GetTranslation(), 0.0f, 2.0f, 0.0f));
    CHECK(skeleton.isDirty());

    const ct::Vector<Math::Mat4>& palette = skeleton.getBonePalette();
    CHECK(palette.size() == 3);
    for (std::size_t i = 0; i < palette.size(); ++i) CHECK(palette[i] == Math::Mat4::Identity());

    skeleton.setBonePosePosition(1, Math::Vec3(0.0f, 2.0f, 0.0f));
    CHECK(nearVec3(skeleton.getBonePosePosition(1), 0.0f, 2.0f, 0.0f));
    CHECK(nearVec3(skeleton.getBoneGlobalPose(2).GetTranslation(), 0.0f, 3.0f, 0.0f));
    CHECK(nearVec3(skeleton.getBonePalette()[2].GetTranslation(), 0.0f, 1.0f, 0.0f));

    skeleton.setBoneGlobalPose(2, translation(5.0f, 5.0f, 5.0f));
    CHECK(nearVec3(skeleton.getBonePosePosition(2), 5.0f, 3.0f, 5.0f));
    CHECK(nearVec3(skeleton.getBoneGlobalPose(2).GetTranslation(), 5.0f, 5.0f, 5.0f));

    skeleton.setBonePoseRotation(0,
            Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 0.0f, 1.0f), kHalfPi));
    CHECK(nearVec3(skeleton.getBoneGlobalPose(1).GetTranslation(), -2.0f, 0.0f, 0.0f));
    skeleton.setBonePoseScale(0, Math::Vec3(2.0f, 2.0f, 2.0f));
    CHECK(nearVec3(skeleton.getBoneGlobalPose(1).GetTranslation(), -4.0f, 0.0f, 0.0f));

    skeleton.resetBonePose(0);
    CHECK(nearVec3(skeleton.getBoneGlobalPose(1).GetTranslation(), 0.0f, 2.0f, 0.0f));
    skeleton.resetBonePoses();
    CHECK(nearVec3(skeleton.getBoneGlobalPose(2).GetTranslation(), 0.0f, 2.0f, 0.0f));
    CHECK(nearVec3(skeleton.getBonePosePosition(2), 0.0f, 1.0f, 0.0f));

    LocalPose pose;
    pose.position = Math::Vec3(1.0f, 1.0f, 1.0f);
    skeleton.setBonePose(1, pose);
    CHECK(nearVec3(skeleton.getBonePose(1).position, 1.0f, 1.0f, 1.0f));
    skeleton.setBonePose(5, pose);
    CHECK(nearVec3(skeleton.getBonePose(5).position, 0.0f, 0.0f, 0.0f));

    skeleton.clearBones();
    CHECK(skeleton.getBoneCount() == 0);
    CHECK(skeleton.getBonePalette().empty());
}

void testDeferredUpdate()
{
    SceneTree tree;
    tree.initialize();
    Node3D holder;
    Skeleton3D* skeleton = holder.createChild<Skeleton3D>();
    skeleton->setSkeleton(chain());
    int updates = 0;
    skeleton->poseUpdated = [&updates]() { ++updates; };

    CHECK(skeleton->getPoseVersion() == 0);
    skeleton->reparent(tree.getRoot());
    CHECK(skeleton->isDirty());
    tree.process(kStep);
    CHECK(!skeleton->isDirty());
    CHECK(skeleton->getPoseVersion() == 1);
    CHECK(updates == 1);

    tree.process(kStep);
    CHECK(skeleton->getPoseVersion() == 1);

    skeleton->setBonePosePosition(1, Math::Vec3(0.0f, 2.0f, 0.0f));
    skeleton->setBonePosePosition(2, Math::Vec3(0.0f, 2.0f, 0.0f));
    CHECK(skeleton->isDirty());
    tree.process(kStep);
    CHECK(skeleton->getPoseVersion() == 2);
    CHECK(updates == 2);
    CHECK(nearVec3(skeleton->getBoneGlobalPose(2).GetTranslation(), 0.0f, 4.0f, 0.0f));

    skeleton->setBonePosePosition(1, Math::Vec3(0.0f, 1.0f, 0.0f));
    skeleton->reparent(&holder);
    tree.process(kStep);
    CHECK(skeleton->isDirty());
    CHECK(skeleton->getPoseVersion() == 2);
    skeleton->updateSkeleton();
    CHECK(!skeleton->isDirty());
    CHECK(skeleton->getPoseVersion() == 3);

    skeleton->reparent(tree.getRoot());
    skeleton->setBonePosePosition(1, Math::Vec3(0.0f, 5.0f, 0.0f));
    skeleton->queueFree();
    tree.process(kStep);
}

void testLibrary()
{
    AnimationPlayer player;
    const AnimationClip walk = walkClip();
    const AnimationClip lift = liftClip(LoopMode::None);
    CHECK(player.isClass("AnimationPlayer"));
    CHECK(player.getAnimationCount() == 0);
    CHECK(!player.addAnimation("", &walk));
    CHECK(!player.addAnimation("walk", nullptr));
    CHECK(player.addAnimation("walk", &walk));
    CHECK(player.addAnimation("lift", &lift));
    CHECK(player.addAnimation("walk", &walk));
    CHECK(player.getAnimationCount() == 2);
    CHECK(player.hasAnimation("lift"));
    CHECK(!player.hasAnimation("run"));
    CHECK(player.getAnimation("walk") == &walk);
    CHECK(player.getAnimation("run") == nullptr);
    CHECK(player.getAnimationName(1) == "lift");
    CHECK(player.getAnimationName(2).empty());
    CHECK(player.removeAnimation("lift"));
    CHECK(!player.removeAnimation("lift"));
    CHECK(player.getAnimationCount() == 1);
    CHECK(player.getSkeletonPath() == "..");
    CHECK(player.getSkeleton() == nullptr);
    CHECK(!player.isPlaying());
    CHECK(player.getCurrentAnimation().empty());
    CHECK(player.getCallbackModeProcess() == AnimationPlayer::CallbackModeProcess::Idle);
    CHECK(player.isActive());

    player.setBlendTime("walk", "lift", 0.5);
    CHECK(near(static_cast<float>(player.getBlendTime("walk", "lift")), 0.5f));
    player.setBlendTime("walk", "lift", 0.0);
    CHECK(player.getBlendTime("walk", "lift") == 0.0);
    player.setBlendTime("walk", "lift", -1.0);
    CHECK(player.getBlendTime("walk", "lift") == 0.0);

    player.animationSetNext("walk", "lift");
    CHECK(player.animationGetNext("walk") == "lift");
    player.animationSetNext("run", "walk");
    CHECK(player.animationGetNext("run").empty());

    player.play("run");
    CHECK(!player.isPlaying());
}

void testPlayback()
{
    Scene scene;
    const AnimationClip walk = walkClip();
    const AnimationClip lift = liftClip(LoopMode::None);
    scene.player->addAnimation("walk", &walk);
    scene.player->addAnimation("lift", &lift);
    CHECK(scene.player->getSkeleton() == scene.skeleton);

    int started = 0;
    ct::String startedName;
    scene.player->animationStarted = [&](const ct::String& name)
    {
        ++started;
        startedName = name;
    };

    scene.player->play("walk");
    CHECK(scene.player->isPlaying());
    CHECK(scene.player->getCurrentAnimation() == "walk");
    CHECK(scene.player->getAssignedAnimation() == "walk");
    CHECK(started == 1 && startedName == "walk");
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationLength()), 1.0f));
    CHECK(near(scene.player->getPlayingSpeed(), 1.0f));

    scene.step(kStep);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.0f));
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(1), 0.0f));
    CHECK(!scene.skeleton->isDirty());

    scene.step(kStep);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.25f));
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(1), 22.5f));
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(scene.skeleton->getBoneGlobalPose(2).GetTranslation(), -0.36810f, 1.92979f,
            0.0f));
    CHECK(!scene.skeleton->isDirty());

    scene.step(0.9);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.15f));
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(1), 13.5f));

    scene.player->play("walk");
    CHECK(started == 1);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.15f));

    scene.player->setSpeedScale(2.0f);
    scene.step(0.1);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.35f));
    CHECK(near(scene.player->getPlayingSpeed(), 2.0f));
    scene.player->setSpeedScale(1.0f);

    scene.player->pause();
    CHECK(!scene.player->isPlaying());
    CHECK(near(scene.player->getPlayingSpeed(), 0.0f));
    scene.step(kStep);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.35f));
    CHECK(scene.player->getAssignedAnimation() == "walk");
    CHECK(scene.player->getCurrentAnimation().empty());

    scene.player->play();
    CHECK(scene.player->isPlaying());
    CHECK(started == 2);
    scene.step(kStep);
    scene.step(0.05);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.4f));

    scene.player->seek(0.5, true);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.5f));
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(1), 45.0f));
    scene.player->seek(5.0);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 1.0f));

    scene.player->stop();
    CHECK(!scene.player->isPlaying());
    CHECK(scene.player->getCurrentAnimationLength() == 0.0);
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(1), 0.0f));
    CHECK(scene.player->getAssignedAnimation() == "walk");

    scene.start("walk");
    scene.step(0.5);
    scene.player->stop(true);
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(1), 45.0f));
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.0f));
}

// The bones the library animates are worked out once and again when it changes: a clip added
// after the player already posed the skeleton still moves its own bone.
void testLibraryChangedWhilePlaying()
{
    Scene scene;
    const AnimationClip walk = walkClip();
    const AnimationClip nod = nodClip();
    scene.player->addAnimation("walk", &walk);
    scene.start("walk", 0.0);
    scene.step(kStep);
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(1), 22.5f));
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(2), 0.0f));

    scene.player->addAnimation("nod", &nod);
    scene.start("nod", 0.0);
    scene.step(kStep);
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(2), 22.5f));

    // Replaced under the same name by a clip of another bone (stopped first: playing the
    // name that already plays would carry on from where it was).
    scene.player->addAnimation("nod", &walk);
    scene.player->stop();
    scene.start("nod", 0.0);
    scene.step(kStep);
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(1), 22.5f));
}

void testFinishAndQueue()
{
    Scene scene;
    const AnimationClip walk = walkClip();
    const AnimationClip lift = liftClip(LoopMode::None);
    const AnimationClip nod = nodClip();
    scene.player->addAnimation("walk", &walk);
    scene.player->addAnimation("lift", &lift);
    scene.player->addAnimation("nod", &nod);

    int finished = 0;
    ct::String finishedName;
    int changed = 0;
    ct::String changedFrom;
    ct::String changedTo;
    scene.player->animationFinished = [&](const ct::String& name)
    {
        ++finished;
        finishedName = name;
    };
    scene.player->animationChanged = [&](const ct::String& from, const ct::String& to)
    {
        ++changed;
        changedFrom = from;
        changedTo = to;
    };

    scene.start("lift");
    scene.step(0.5);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 2.0f, 0.0f));
    CHECK(finished == 0);
    scene.step(0.6);
    CHECK(finished == 1 && finishedName == "lift");
    CHECK(!scene.player->isPlaying());
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 1.0f));
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 3.0f, 0.0f));
    scene.step(kStep);
    CHECK(finished == 1);

    scene.player->play("lift");
    CHECK(scene.player->isPlaying());
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.0f));

    scene.player->queue("nod");
    CHECK(scene.player->getQueueCount() == 1);
    CHECK(scene.player->getQueued(0) == "nod");
    scene.step(kStep);
    scene.step(1.0);
    CHECK(finished == 2 && finishedName == "lift");
    CHECK(changed == 1 && changedFrom == "lift" && changedTo == "nod");
    CHECK(scene.player->isPlaying());
    CHECK(scene.player->getCurrentAnimation() == "nod");
    CHECK(scene.player->getQueueCount() == 0);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 3.0f, 0.0f));
    scene.step(kStep);
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(2), 0.0f));
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 3.0f, 0.0f));
    scene.step(kStep);
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(2), 22.5f));
    scene.step(0.5);
    CHECK(finished == 3 && finishedName == "nod");
    CHECK(!scene.player->isPlaying());

    scene.player->animationSetNext("lift", "nod");
    scene.player->play("lift");
    CHECK(scene.player->getQueueCount() == 1);
    scene.player->clearQueue();
    CHECK(scene.player->getQueueCount() == 0);
    scene.player->queue("walk");
    scene.player->play("nod");
    CHECK(scene.player->getQueueCount() == 0);
    scene.player->stop();
}

void testBlend()
{
    Scene scene;
    const AnimationClip walk = walkClip();
    const AnimationClip lift = liftClip(LoopMode::None);
    scene.player->addAnimation("walk", &walk);
    scene.player->addAnimation("lift", &lift);

    scene.start("lift");
    scene.step(1.0);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 3.0f, 0.0f));

    scene.start("walk", 0.5);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 3.0f, 0.0f));
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(1), 0.0f));
    scene.step(kStep);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 1.0f, 0.0f));
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(1), 22.5f));

    scene.step(0.3);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 1.0f, 0.0f));
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(1), 49.5f));

    scene.player->setDefaultBlendTime(1.0);
    scene.start("lift");
    scene.step(0.5);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 1.5f, 0.0f));
    scene.player->stop();

    scene.player->setDefaultBlendTime(0.0);
    scene.player->setBlendTime("*", "walk", 0.5);
    scene.start("lift");
    scene.step(1.0);
    scene.start("walk");
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 3.0f, 0.0f));
    scene.step(kStep);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 1.0f, 0.0f));
    scene.player->stop();

    scene.start("lift");
    scene.step(1.0);
    scene.start("walk", 0.0);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 1.0f, 0.0f));
}

void testDeterministic()
{
    Scene scene;
    const AnimationClip walk = walkClip();
    const AnimationClip lift = liftClip(LoopMode::None);
    const AnimationClip nod = nodClip();
    scene.player->addAnimation("walk", &walk);
    scene.player->addAnimation("lift", &lift);
    scene.player->addAnimation("nod", &nod);
    CHECK(!scene.player->isDeterministic());

    scene.skeleton->setBonePosePosition(2, Math::Vec3(0.0f, 5.0f, 0.0f));
    scene.start("walk");
    scene.step(kStep);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(2), 0.0f, 5.0f, 0.0f));
    scene.player->stop();

    scene.player->setDeterministic(true);
    CHECK(scene.player->isDeterministic());
    scene.skeleton->setBonePosePosition(2, Math::Vec3(0.0f, 5.0f, 0.0f));
    scene.start("walk");
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(2), 0.0f, 1.0f, 0.0f));
    scene.step(kStep);
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(1), 22.5f));
    scene.player->stop();

    scene.start("lift");
    scene.step(1.0);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 3.0f, 0.0f));
    scene.start("nod", 1.0);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 2.5f, 0.0f));
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(2), 0.0f));
    scene.step(kStep);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 2.0f, 0.0f));
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(2), 5.625f));
    scene.step(kStep);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 1.0f, 0.0f));
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(2), 45.0f));

    AnimationPlayer* top = scene.skeleton->createChild<AnimationPlayer>();
    top->setDeterministic(true);
    top->addAnimation("nod", &nod);
    scene.player->removeAnimation("nod");
    scene.player->play("walk");
    top->play("nod");
    scene.step(kStep);
    scene.step(kStep);
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(1), 22.5f));
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(2), 22.5f));
    scene.skeleton->setBonePosePosition(0, Math::Vec3(4.0f, 0.0f, 0.0f));
    scene.step(kStep);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(0), 4.0f, 0.0f, 0.0f));
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(1), 45.0f));
    CHECK(nearAngleZ(scene.skeleton->getBonePoseRotation(2), 45.0f));
}

void testNodeTracks()
{
    const Math::Quaternion quarter =
            Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 0.0f, 1.0f), kHalfPi);

    AnimationClip open;
    open.setName("open");
    CHECK(open.addNodeTrack("") == -1);
    const int door = open.addNodeTrack("Door");
    CHECK(door == 0);
    CHECK(open.addNodeTrack("Door") == 0);
    CHECK(open.findNodeTrack("Door") == 0);
    CHECK(open.findNodeTrack("Window") == -1);
    CHECK(open.positionTrackInsertKey(door, 1.0f, Math::Vec3(2.0f, 0.0f, 0.0f)) == 0);
    CHECK(open.positionTrackInsertKey(door, 0.0f, Math::Vec3(0.0f, 0.0f, 0.0f)) == 0);
    CHECK(open.positionTrackInsertKey(door, 1.0f, Math::Vec3(4.0f, 0.0f, 0.0f)) == 1);
    CHECK(open.positionTrackInsertKey(7, 0.0f, Math::Vec3(0.0f, 0.0f, 0.0f)) == -1);
    CHECK(open.rotationTrackInsertKey(door, 0.0f, Math::Quaternion::Identity()) == 0);
    CHECK(open.rotationTrackInsertKey(door, 1.0f, quarter) == 1);
    CHECK(near(open.duration(), 1.0f));
    const int camera = open.addNodeTrack("Rig/Camera");
    CHECK(camera == 1);
    CHECK(open.scaleTrackInsertKey(camera, 0.0f, Math::Vec3(1.0f, 1.0f, 1.0f)) == 0);
    CHECK(open.scaleTrackInsertKey(camera, 1.0f, Math::Vec3(3.0f, 3.0f, 3.0f)) == 1);
    open.addNodeTrack("Missing");
    open.positionTrackInsertKey(2, 0.0f, Math::Vec3(9.0f, 9.0f, 9.0f));

    const NodeTrack& doorTrack = open.nodeTracks()[0];
    CHECK(doorTrack.positions.size() == 2);
    Math::Vec3 sampled;
    CHECK(doorTrack.samplePosition(0.5f, sampled) && nearVec3(sampled, 2.0f, 0.0f, 0.0f));
    CHECK(doorTrack.samplePosition(-1.0f, sampled) && nearVec3(sampled, 0.0f, 0.0f, 0.0f));
    CHECK(doorTrack.samplePosition(9.0f, sampled) && nearVec3(sampled, 4.0f, 0.0f, 0.0f));
    CHECK(!doorTrack.sampleScale(0.5f, sampled));
    Math::Quaternion turned;
    CHECK(doorTrack.sampleRotation(0.5f, turned) && nearAngleZ(turned, 45.0f));
    CHECK(!open.nodeTracks()[1].sampleRotation(0.5f, turned));

    AnimationClip close;
    close.setName("close");
    close.setDuration(1.0f);
    CHECK(close.positionTrackInsertKey(close.addNodeTrack("Door"), 0.0f,
                  Math::Vec3(0.0f, 0.0f, 0.0f)) == 0);

    SceneTree tree;
    tree.initialize();
    Node3D* stage = tree.getRoot()->createChild<Node3D>();
    stage->setName("Stage");
    Node3D* doorNode = stage->createChild<Node3D>();
    doorNode->setName("Door");
    doorNode->setScale(Math::Vec3(2.0f, 2.0f, 2.0f));
    Node3D* rig = stage->createChild<Node3D>();
    rig->setName("Rig");
    Node3D* cameraNode = rig->createChild<Node3D>();
    cameraNode->setName("Camera");
    cameraNode->setPosition(Math::Vec3(7.0f, 7.0f, 7.0f));
    AnimationPlayer* player = stage->createChild<AnimationPlayer>();
    CHECK(strcmp(player->getRootNode().c_str(), "..") == 0);
    CHECK(player->getSkeleton() == nullptr);
    player->addAnimation("open", &open);
    player->addAnimation("close", &close);

    player->play("open");
    tree.process(kStep);
    CHECK(nearVec3(doorNode->getPosition(), 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(doorNode->getScale(), 2.0f, 2.0f, 2.0f));
    tree.process(0.5);
    CHECK(nearVec3(doorNode->getPosition(), 2.0f, 0.0f, 0.0f));
    CHECK(nearAngleZ(doorNode->getQuaternion(), 45.0f));
    CHECK(nearVec3(doorNode->getScale(), 2.0f, 2.0f, 2.0f));
    CHECK(nearVec3(cameraNode->getScale(), 2.0f, 2.0f, 2.0f));
    CHECK(nearVec3(cameraNode->getPosition(), 7.0f, 7.0f, 7.0f));
    tree.process(0.5);
    CHECK(nearVec3(doorNode->getPosition(), 4.0f, 0.0f, 0.0f));
    CHECK(nearAngleZ(doorNode->getQuaternion(), 90.0f));
    CHECK(nearVec3(cameraNode->getScale(), 3.0f, 3.0f, 3.0f));
    CHECK(!player->isPlaying());

    player->play("close", 1.0);
    tree.process(kStep);
    CHECK(nearVec3(doorNode->getPosition(), 4.0f, 0.0f, 0.0f));
    tree.process(kStep);
    CHECK(nearVec3(doorNode->getPosition(), 8.0f / 3.0f, 0.0f, 0.0f));
    CHECK(nearAngleZ(doorNode->getQuaternion(), 60.0f));
    player->stop();

    player->setRootNode("../Rig");
    doorNode->setPosition(Math::Vec3(1.0f, 1.0f, 1.0f));
    player->play("open");
    tree.process(kStep);
    tree.process(kStep);
    CHECK(nearVec3(doorNode->getPosition(), 1.0f, 1.0f, 1.0f));
    player->stop();
    player->setRootNode("..");

    player->setDeterministic(true);
    cameraNode->setScale(Math::Vec3(5.0f, 5.0f, 5.0f));
    doorNode->setQuaternion(quarter);
    player->play("close");
    tree.process(kStep);
    CHECK(nearVec3(doorNode->getPosition(), 0.0f, 0.0f, 0.0f));
    CHECK(nearAngleZ(doorNode->getQuaternion(), 0.0f));
    CHECK(nearVec3(cameraNode->getScale(), 1.0f, 1.0f, 1.0f));
    CHECK(nearVec3(cameraNode->getPosition(), 7.0f, 7.0f, 7.0f));

    doorNode->queueFree();
    tree.process(kStep);
    tree.process(kStep);
    player->stop();
}

void testRecordAndExtract()
{
    SceneTree tree;
    tree.initialize();
    Node3D* stage = tree.getRoot()->createChild<Node3D>();
    stage->setName("Stage");
    Node3D* rig = stage->createChild<Node3D>();
    rig->setName("Rig");
    Node3D* camera = rig->createChild<Node3D>();
    camera->setName("Camera");
    Node3D* outside = tree.getRoot()->createChild<Node3D>();
    outside->setName("Outside");
    AnimationPlayer* player = stage->createChild<AnimationPlayer>();

    AnimationClip shot;
    shot.setName("shot");
    CHECK(player->recordKey(shot, nullptr, 0.0f) == -1);
    CHECK(player->recordKey(shot, outside, 0.0f) == -1);
    camera->setPosition(Math::Vec3(0.0f, 1.0f, 0.0f));
    CHECK(player->recordKey(shot, camera, 0.0f) == 0);
    camera->setPosition(Math::Vec3(4.0f, 1.0f, 0.0f));
    camera->setScale(Math::Vec3(9.0f, 9.0f, 9.0f));
    CHECK(player->recordKey(shot, camera, 2.0f, true, false, false) == 0);
    camera->setPosition(Math::Vec3(4.0f, 5.0f, 0.0f));
    CHECK(player->recordKey(shot, camera, 4.0f, true, false, false) == 0);
    rig->setPosition(Math::Vec3(0.0f, 0.0f, 3.0f));
    CHECK(player->recordKey(shot, rig, 4.0f, true, false, false) == 1);
    CHECK(player->recordKey(shot, stage, 1.0f, false, false, true) == 2);
    shot.addEvent(3.0f, "cut");
    shot.addEvent(0.5f, "start");

    CHECK(shot.nodeTracks().size() == 3);
    CHECK(strcmp(shot.nodeTracks()[0].path.c_str(), "Rig/Camera") == 0);
    CHECK(strcmp(shot.nodeTracks()[1].path.c_str(), "Rig") == 0);
    CHECK(strcmp(shot.nodeTracks()[2].path.c_str(), ".") == 0);
    CHECK(shot.nodeTracks()[0].positions.size() == 3);
    CHECK(shot.nodeTracks()[0].rotations.size() == 1);
    CHECK(shot.nodeTracks()[0].scales.size() == 1);
    CHECK(nearVec3(shot.nodeTracks()[0].scales[0], 1.0f, 1.0f, 1.0f));
    CHECK(near(shot.duration(), 4.0f));

    const AnimationClip second = shot.extract(2.0f, 4.0f);
    CHECK(near(second.duration(), 2.0f));
    CHECK(strcmp(second.name().c_str(), "shot") == 0);
    CHECK(second.nodeTracks().size() == 2);
    const NodeTrack& cut = second.nodeTracks()[0];
    CHECK(cut.positions.size() == 2 && cut.rotations.empty() && cut.scales.empty());
    CHECK(near(cut.positionTimes[0], 0.0f) && near(cut.positionTimes[1], 2.0f));
    CHECK(nearVec3(cut.positions[0], 4.0f, 1.0f, 0.0f));
    CHECK(nearVec3(cut.positions[1], 4.0f, 5.0f, 0.0f));
    CHECK(strcmp(second.nodeTracks()[1].path.c_str(), "Rig") == 0);
    CHECK(near(second.nodeTracks()[1].positionTimes[0], 2.0f));
    CHECK(second.events().size() == 1 && near(second.events()[0].time, 1.0f));
    CHECK(shot.extract(3.0f, 1.0f).nodeTracks().empty());

    AnimationClip bones = walkClip();
    const AnimationClip half = bones.extract(0.5f, 1.0f);
    CHECK(half.tracks().size() == 1 && half.tracks()[0].times.size() == 1);
    CHECK(near(half.tracks()[0].times[0], 0.5f));
    CHECK(half.loopMode() == LoopMode::Linear);

    camera->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    rig->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    player->addAnimation("second", &second);
    player->play("second");
    tree.process(kStep);
    CHECK(nearVec3(camera->getPosition(), 4.0f, 1.0f, 0.0f));
    tree.process(1.0);
    CHECK(nearVec3(camera->getPosition(), 4.0f, 3.0f, 0.0f));
    CHECK(nearVec3(rig->getPosition(), 0.0f, 0.0f, 3.0f));
    CHECK(nearVec3(camera->getScale(), 9.0f, 9.0f, 9.0f));
    player->stop();
}

bool called(const ct::Vector<ct::String>& calls, std::size_t index, const char* name)
{
    return index < calls.size() && strcmp(calls[index].c_str(), name) == 0;
}

void testMethodValueAndAnimationTracks()
{
    AnimationClip cut;
    cut.setName("cut");
    cut.setDuration(1.0f);
    CHECK(cut.addMethodTrack("") == -1);
    const int self = cut.addMethodTrack(".");
    CHECK(self == 0 && cut.addMethodTrack(".") == 0);
    CHECK(cut.methodTrackInsertKey(self, 0.5f, "half") == 0);
    CHECK(cut.methodTrackInsertKey(self, 0.0f, "start") == 0);
    CHECK(cut.methodTrackInsertKey(self, 1.0f, "end") == 2);
    CHECK(cut.methodTrackInsertKey(9, 1.0f, "end") == -1);
    cut.methodTrackInsertKey(cut.addMethodTrack("Door"), 0.75f, "slam");
    cut.methodTrackInsertKey(cut.addMethodTrack("Missing"), 0.25f, "never");
    cut.addEvent(0.25f, "event");
    CHECK(cut.addValueTrack("Light", "") == -1);
    const int energy = cut.addValueTrack("Light", "energy");
    CHECK(cut.valueTrackInsertKey(energy, 0.0f, 1.0f) == 0);
    CHECK(cut.valueTrackInsertKey(energy, 1.0f, 3.0f) == 1);

    ct::Vector<int> keys;
    cut.keyIndicesInRange(cut.methodTracks()[0].times, 0.5, 0.5, LoopedFlag::None, keys);
    CHECK(keys.size() == 1 && keys[0] == 1);
    keys.clear();
    cut.keyIndicesInRange(cut.methodTracks()[0].times, 0.0, -1.0, LoopedFlag::None, keys);
    CHECK(keys.size() == 2 && keys[0] == 1 && keys[1] == 0);
    keys.clear();
    cut.keyIndicesInRange(cut.methodTracks()[0].times, 0.5, 0.0, LoopedFlag::None, keys);
    CHECK(keys.empty());
    CHECK(AnimationClip::findKey(cut.methodTracks()[0].times, 0.7, false) == 1);
    CHECK(AnimationClip::findKey(cut.methodTracks()[0].times, 0.7, true) == -1);
    CHECK(AnimationClip::findKey(cut.methodTracks()[0].times, 0.5, true) == 1);

    SceneTree tree;
    tree.initialize();
    Node3D* stage = tree.getRoot()->createChild<Node3D>();
    stage->setName("Stage");
    Node3D* door = stage->createChild<Node3D>();
    door->setName("Door");
    Node3D* light = stage->createChild<Node3D>();
    light->setName("Light");
    AnimationPlayer* player = stage->createChild<AnimationPlayer>();
    player->setName("Director");

    ct::Vector<ct::String> calls;
    ct::Vector<Node*> targets;
    player->methodCalled = [&calls, &targets](Node* node, const ct::String& method)
    {
        calls.push_back(method);
        targets.push_back(node);
    };
    float lightEnergy = -1.0f;
    Node* lightTarget = nullptr;
    player->valueChanged =
            [&lightEnergy, &lightTarget](Node* node, const ct::String& property, float value)
    {
        if (strcmp(property.c_str(), "energy") != 0) return;
        lightEnergy = value;
        lightTarget = node;
    };
    player->addAnimation("cut", &cut);

    player->play("cut");
    tree.process(kStep);
    CHECK(calls.size() == 1 && called(calls, 0, "start") && targets[0] == stage);
    CHECK(near(lightEnergy, 1.0f) && lightTarget == light);
    tree.process(kStep);
    CHECK(calls.size() == 2 && called(calls, 1, "event") && targets[1] == stage);
    CHECK(near(lightEnergy, 1.5f));
    tree.process(kStep);
    CHECK(calls.size() == 3 && called(calls, 2, "half"));
    tree.process(kStep);
    CHECK(calls.size() == 4 && called(calls, 3, "slam") && targets[3] == door);
    tree.process(kStep);
    CHECK(calls.size() == 5 && called(calls, 4, "end"));
    CHECK(near(lightEnergy, 3.0f));
    CHECK(!player->isPlaying());

    calls.clear();
    player->play("cut");
    tree.process(kStep);
    CHECK(calls.size() == 1 && called(calls, 0, "start"));
    tree.process(1.0);
    CHECK(calls.size() == 5);
    CHECK(called(calls, 1, "half") && called(calls, 2, "end"));
    CHECK(called(calls, 3, "slam") && called(calls, 4, "event"));

    calls.clear();
    player->playBackwards("cut");
    tree.process(kStep);
    CHECK(calls.size() == 1 && called(calls, 0, "end"));
    tree.process(kStep);
    CHECK(calls.size() == 2 && called(calls, 1, "slam"));
    tree.process(0.5);
    CHECK(calls.size() == 4 && called(calls, 2, "half") && called(calls, 3, "event"));
    tree.process(kStep);
    CHECK(calls.size() == 5 && called(calls, 4, "start"));

    calls.clear();
    player->play("cut");
    tree.process(kStep);
    calls.clear();
    player->seek(0.6, true);
    CHECK(calls.empty());
    CHECK(near(lightEnergy, 2.2f));
    player->stop();

    AnimationClip loop;
    loop.setName("loop");
    loop.setDuration(1.0f);
    loop.setLoopMode(LoopMode::Linear);
    const int beat = loop.addMethodTrack(".");
    loop.methodTrackInsertKey(beat, 0.0f, "zero");
    loop.methodTrackInsertKey(beat, 0.9f, "nine");
    player->addAnimation("loop", &loop);
    calls.clear();
    player->play("loop");
    tree.process(kStep);
    CHECK(calls.size() == 1 && called(calls, 0, "zero"));
    tree.process(0.5);
    CHECK(calls.size() == 1);
    tree.process(0.5);
    CHECK(calls.size() == 3 && called(calls, 1, "nine") && called(calls, 2, "zero"));
    player->stop();

    AnimationClip wave;
    wave.setName("wave");
    wave.setDuration(2.0f);
    AnimationPlayer* actor = stage->createChild<AnimationPlayer>();
    actor->setName("Actor");
    actor->addAnimation("wave", &wave);

    AnimationClip scene;
    scene.setName("scene");
    scene.setDuration(1.0f);
    CHECK(scene.addAnimationTrack("") == -1);
    const int cue = scene.addAnimationTrack("Actor");
    CHECK(scene.animationTrackInsertKey(cue, 0.25f, "wave") == 0);
    CHECK(scene.animationTrackInsertKey(cue, 0.75f, "[stop]") == 1);
    scene.animationTrackInsertKey(scene.addAnimationTrack("Door"), 0.25f, "wave");
    player->addAnimation("scene", &scene);
    player->play("scene");
    tree.process(kStep);
    CHECK(!actor->isPlaying());
    tree.process(kStep);
    CHECK(actor->isPlaying());
    CHECK(strcmp(actor->getCurrentAnimation().c_str(), "wave") == 0);
    tree.process(kStep);
    CHECK(actor->isPlaying());
    CHECK(near(static_cast<float>(actor->getCurrentAnimationPosition()), 0.25f));
    tree.process(kStep);
    CHECK(!actor->isPlaying());
    player->stop();

    AnimationClip opening;
    opening.setName("opening");
    opening.setDuration(1.0f);
    opening.animationTrackInsertKey(opening.addAnimationTrack("Actor"), 0.0f, "wave");
    player->addAnimation("opening", &opening);
    player->play("opening");
    tree.process(kStep);
    CHECK(actor->isPlaying());

    const AnimationClip part = cut.extract(0.5f, 1.0f);
    CHECK(part.methodTracks().size() == 2);
    CHECK(part.methodTracks()[0].times.size() == 2 && near(part.methodTracks()[0].times[0], 0.0f));
    CHECK(part.valueTracks().size() == 1 && near(part.valueTracks()[0].values[0], 3.0f));
}

void testLoopModesAndDirection()
{
    Scene scene;
    const AnimationClip pingPong = liftClip(LoopMode::PingPong);
    const AnimationClip lift = liftClip(LoopMode::None);
    scene.player->addAnimation("pingpong", &pingPong);
    scene.player->addAnimation("lift", &lift);

    scene.start("pingpong");
    scene.step(0.75);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.75f));
    scene.step(0.5);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.75f));
    CHECK(near(scene.player->getPlayingSpeed(), -1.0f));
    scene.step(0.5);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.25f));
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 1.5f, 0.0f));
    CHECK(scene.player->isPlaying());

    int finished = 0;
    scene.player->animationFinished = [&finished](const ct::String&) { ++finished; };
    scene.player->playBackwards("lift");
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 1.0f));
    scene.step(kStep);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 3.0f, 0.0f));
    scene.step(0.5);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.5f));
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 2.0f, 0.0f));
    scene.step(0.6);
    CHECK(finished == 1);
    CHECK(!scene.player->isPlaying());
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 1.0f, 0.0f));
}

void testModesAndAutoplay()
{
    Scene scene;
    const AnimationClip lift = liftClip(LoopMode::None);
    scene.player->addAnimation("lift", &lift);

    scene.player->setCallbackModeProcess(AnimationPlayer::CallbackModeProcess::Manual);
    scene.player->play("lift");
    scene.step(0.5);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.0f));
    scene.player->advance(0.0);
    scene.player->advance(0.5);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.5f));
    scene.tree.flushSkeletonUpdates();
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 2.0f, 0.0f));

    scene.player->setCallbackModeProcess(AnimationPlayer::CallbackModeProcess::Physics);
    scene.step(0.25);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.5f));
    scene.tree.physicsProcess(0.25);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.75f));
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 2.5f, 0.0f));

    scene.player->setCallbackModeProcess(AnimationPlayer::CallbackModeProcess::Idle);
    scene.player->setActive(false);
    scene.step(0.1);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.75f));
    scene.player->setActive(true);
    scene.step(0.1);
    CHECK(near(static_cast<float>(scene.player->getCurrentAnimationPosition()), 0.85f));
    scene.player->stop();

    Node3D secondHolder;
    AnimationPlayer* second = secondHolder.createChild<AnimationPlayer>();
    second->addAnimation("lift", &lift);
    second->setAutoplay("lift");
    second->setSkeletonPath("../Skel");
    CHECK(second->getAutoplay() == "lift");
    CHECK(second->reparent(scene.tree.getRoot()));
    CHECK(second->isPlaying());
    CHECK(second->getSkeleton() == scene.skeleton);
    scene.step(0.5);
    CHECK(near(static_cast<float>(second->getCurrentAnimationPosition()), 0.5f));
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 2.0f, 0.0f));
    second->setSkeletonPath("../Nowhere");
    CHECK(second->getSkeleton() == nullptr);
    scene.step(0.1);
    CHECK(nearVec3(scene.skeleton->getBonePosePosition(1), 0.0f, 2.0f, 0.0f));
}

} // namespace

int main()
{
    testSkeletonNode();
    testDeferredUpdate();
    testLibrary();
    testPlayback();
    testLibraryChangedWhilePlaying();
    testFinishAndQueue();
    testBlend();
    testDeterministic();
    testNodeTracks();
    testRecordAndExtract();
    testMethodValueAndAnimationTracks();
    testLoopModesAndDirection();
    testModesAndAutoplay();

    if (failures) printf("%d checks failed\n", failures);
    else
        printf("ok\n");
    return failures ? 1 : 0;
}
