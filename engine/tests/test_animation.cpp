#include "Check.h"

#include "prisma3d/scene/Animation.h"

#include <math.h>

using namespace prisma3d;

namespace
{

const float kTolerance = 1e-4f;
const float kHalfPi = 1.57079632679f;

bool near(float a, float b)
{
    return fabsf(a - b) <= kTolerance;
}

bool nearVec3(const Math::Vec3& a, float x, float y, float z)
{
    return near(a.x, x) && near(a.y, y) && near(a.z, z);
}

bool nearQuat(const Math::Quaternion& a, const Math::Quaternion& b)
{
    return near(a.x, b.x) && near(a.y, b.y) && near(a.z, b.z) && near(a.w, b.w);
}

bool nearMat4(const Math::Mat4& a, const Math::Mat4& b)
{
    return near(a.col0.x, b.col0.x) && near(a.col0.y, b.col0.y) && near(a.col0.z, b.col0.z) &&
           near(a.col0.w, b.col0.w) && near(a.col1.x, b.col1.x) && near(a.col1.y, b.col1.y) &&
           near(a.col1.z, b.col1.z) && near(a.col1.w, b.col1.w) && near(a.col2.x, b.col2.x) &&
           near(a.col2.y, b.col2.y) && near(a.col2.z, b.col2.z) && near(a.col2.w, b.col2.w) &&
           near(a.col3.x, b.col3.x) && near(a.col3.y, b.col3.y) && near(a.col3.z, b.col3.z) &&
           near(a.col3.w, b.col3.w);
}

Math::Mat4 translation(float x, float y, float z)
{
    Math::Mat4 result = Math::Mat4::Identity();
    result.col3 = Math::Vec4(x, y, z, 1.0f);
    return result;
}

Skeleton chain()
{
    Skeleton skeleton;
    CHECK(skeleton.addBone("root", -1, translation(0.0f, 0.0f, 0.0f),
            translation(0.0f, 0.0f, 0.0f)));
    CHECK(skeleton.addBone("spine", 0, translation(0.0f, 1.0f, 0.0f),
            translation(0.0f, -1.0f, 0.0f)));
    CHECK(skeleton.addBone("head", 1, translation(0.0f, 1.0f, 0.0f),
            translation(0.0f, -2.0f, 0.0f)));
    return skeleton;
}

void test_skeleton()
{
    Skeleton empty;
    CHECK(empty.empty());
    CHECK(empty.boneCount() == 0);
    CHECK(empty.findBone("root") == -1);

    Skeleton skeleton = chain();
    CHECK(!skeleton.empty());
    CHECK(skeleton.boneCount() == 3);
    CHECK(!skeleton.finalized());
    CHECK(skeleton.findBone("root") == 0);
    CHECK(skeleton.findBone("head") == 2);
    CHECK(skeleton.findBone("tail") == -1);
    CHECK(skeleton.findBone(nullptr) == -1);
    CHECK(skeleton.bone(1).name == "spine");
    CHECK(skeleton.bone(1).parent == 0);

    CHECK(!skeleton.addBone("", -1, Math::Mat4::Identity(), Math::Mat4::Identity()));
    CHECK(!skeleton.addBone("root", -1, Math::Mat4::Identity(), Math::Mat4::Identity()));
    CHECK(!skeleton.addBone("bad", -2, Math::Mat4::Identity(), Math::Mat4::Identity()));
    CHECK(skeleton.boneCount() == 3);

    CHECK(skeleton.finalize());
    CHECK(skeleton.finalized());
    CHECK(skeleton.order().size() == 3);
    CHECK(skeleton.order()[0] == 0 && skeleton.order()[1] == 1 && skeleton.order()[2] == 2);

    Skeleton shuffled;
    CHECK(shuffled.addBone("head", 1, translation(0.0f, 1.0f, 0.0f), Math::Mat4::Identity()));
    CHECK(shuffled.addBone("spine", 2, translation(0.0f, 1.0f, 0.0f), Math::Mat4::Identity()));
    CHECK(shuffled.addBone("root", -1, Math::Mat4::Identity(), Math::Mat4::Identity()));
    CHECK(shuffled.finalize());
    CHECK(shuffled.order().size() == 3);
    CHECK(shuffled.order()[0] == 2 && shuffled.order()[1] == 1 && shuffled.order()[2] == 0);
    CHECK(shuffled.addBone("extra", -1, Math::Mat4::Identity(), Math::Mat4::Identity()));
    CHECK(!shuffled.finalized());
    CHECK(shuffled.finalize());
    CHECK(shuffled.finalized());

    Skeleton cycle;
    CHECK(cycle.addBone("a", 1, Math::Mat4::Identity(), Math::Mat4::Identity()));
    CHECK(cycle.addBone("b", 0, Math::Mat4::Identity(), Math::Mat4::Identity()));
    CHECK(!cycle.finalize());
    CHECK(!cycle.finalized());

    Skeleton self;
    CHECK(self.addBone("a", 0, Math::Mat4::Identity(), Math::Mat4::Identity()));
    CHECK(!self.finalize());

    Skeleton missing;
    CHECK(missing.addBone("a", 5, Math::Mat4::Identity(), Math::Mat4::Identity()));
    CHECK(!missing.finalize());

    Skeleton none;
    CHECK(none.finalize());
    CHECK(none.finalized());
}

void test_pose()
{
    LocalPose pose;
    pose.position = Math::Vec3(1.0f, -2.0f, 3.0f);
    pose.rotation = Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 0.0f, 1.0f), 0.7f);
    pose.scale = Math::Vec3(2.0f, 3.0f, 0.5f);
    LocalPose back;
    CHECK(LocalPose::fromMatrix(pose.toMatrix(), back));
    CHECK(nearVec3(back.position, 1.0f, -2.0f, 3.0f));
    CHECK(nearVec3(back.scale, 2.0f, 3.0f, 0.5f));
    CHECK(nearQuat(back.rotation, pose.rotation));

    pose.scale = Math::Vec3(-2.0f, 3.0f, 0.5f);
    CHECK(LocalPose::fromMatrix(pose.toMatrix(), back));
    CHECK(nearVec3(back.scale, -2.0f, 3.0f, 0.5f));
    CHECK(nearMat4(back.toMatrix(), pose.toMatrix()));

    CHECK(nearMat4(LocalPose().toMatrix(), Math::Mat4::Identity()));

    Math::Mat4 bad = Math::Mat4::Identity();
    bad.col3.x = 1.0f / 0.0f;
    CHECK(!LocalPose::fromMatrix(bad, back));
    Math::Mat4 projective = Math::Mat4::Identity();
    projective.col0.w = 0.5f;
    CHECK(!LocalPose::fromMatrix(projective, back));

    Skeleton skeleton;
    Math::Mat4 broken = Math::Mat4::Identity();
    broken.col3.y = 0.0f / 0.0f;
    CHECK(skeleton.addBone("ok", -1, translation(0.0f, 2.0f, 0.0f), Math::Mat4::Identity()));
    CHECK(skeleton.addBone("broken", 0, broken, Math::Mat4::Identity()));
    ct::Vector<LocalPose> bind;
    skeleton.bindPose(bind);
    CHECK(bind.size() == 2);
    if (bind.size() == 2)
    {
        CHECK(nearVec3(bind[0].position, 0.0f, 2.0f, 0.0f));
        CHECK(nearVec3(bind[1].position, 0.0f, 0.0f, 0.0f));
        CHECK(nearVec3(bind[1].scale, 1.0f, 1.0f, 1.0f));
        CHECK(nearQuat(bind[1].rotation, Math::Quaternion()));
    }
}

void test_evaluate()
{
    Skeleton skeleton = chain();
    ct::Vector<LocalPose> bind;
    skeleton.bindPose(bind);
    CHECK(bind.size() == 3);

    ct::Vector<Math::Mat4> globals;
    ct::Vector<Math::Mat4> palette;
    skeleton.evaluate(bind, globals, palette);
    CHECK(globals.empty() && palette.empty());

    CHECK(skeleton.finalize());
    skeleton.evaluate(bind, globals, palette);
    CHECK(globals.size() == 3 && palette.size() == 3);
    if (globals.size() != 3 || palette.size() != 3) return;
    CHECK(nearVec3(globals[0].GetTranslation(), 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(globals[1].GetTranslation(), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(globals[2].GetTranslation(), 0.0f, 2.0f, 0.0f));
    for (int i = 0; i < 3; ++i) CHECK(nearMat4(palette[i], Math::Mat4::Identity()));

    bind[1].rotation = Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 0.0f, 1.0f), kHalfPi);
    skeleton.evaluate(bind, globals, palette);
    CHECK(nearVec3(globals[1].GetTranslation(), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(globals[2].GetTranslation(), -1.0f, 1.0f, 0.0f));
    CHECK(nearVec3(palette[2].TransformPoint(Math::Vec3(0.0f, 2.0f, 0.0f)), -1.0f, 1.0f, 0.0f));
    CHECK(nearVec3(palette[0].TransformPoint(Math::Vec3(0.0f, 2.0f, 0.0f)), 0.0f, 2.0f, 0.0f));

    ct::Vector<LocalPose> wrong;
    wrong.resize(2);
    globals.clear();
    palette.clear();
    skeleton.evaluate(wrong, globals, palette);
    CHECK(globals.empty() && palette.empty());
}

AnimationClip spineClip()
{
    AnimationClip clip;
    clip.setName("bend");
    clip.setSkeleton(2);
    clip.setDuration(1.0f);
    BoneTrack track;
    track.bone = 1;
    track.times.push_back(0.0f);
    track.times.push_back(1.0f);
    track.positions.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
    track.positions.push_back(Math::Vec3(0.0f, 3.0f, 0.0f));
    track.rotations.push_back(Math::Quaternion());
    track.rotations.push_back(
            Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 0.0f, 1.0f), kHalfPi));
    track.scales.push_back(Math::Vec3(1.0f, 1.0f, 1.0f));
    track.scales.push_back(Math::Vec3(3.0f, 3.0f, 3.0f));
    clip.tracks().push_back(track);
    return clip;
}

void test_sample()
{
    Skeleton skeleton = chain();
    ct::Vector<LocalPose> bind;
    skeleton.bindPose(bind);
    const AnimationClip clip = spineClip();
    CHECK(clip.name() == "bend");
    CHECK(clip.skeleton() == 2);
    CHECK(near(clip.duration(), 1.0f));
    CHECK(AnimationClip().skeleton() == 0);

    const Math::Quaternion quarter =
            Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 0.0f, 1.0f), kHalfPi);
    const Math::Quaternion eighth =
            Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 0.0f, 1.0f), kHalfPi * 0.5f);

    ct::Vector<LocalPose> pose = bind;
    clip.sample(0.0f, pose);
    CHECK(nearVec3(pose[1].position, 0.0f, 1.0f, 0.0f));
    CHECK(nearQuat(pose[1].rotation, Math::Quaternion()));
    CHECK(nearVec3(pose[1].scale, 1.0f, 1.0f, 1.0f));

    pose = bind;
    clip.sample(0.5f, pose);
    CHECK(nearVec3(pose[1].position, 0.0f, 2.0f, 0.0f));
    CHECK(nearQuat(pose[1].rotation, eighth));
    CHECK(near(pose[1].rotation.LengthSquared(), 1.0f));
    CHECK(nearVec3(pose[1].scale, 2.0f, 2.0f, 2.0f));

    pose = bind;
    clip.sample(1.0f, pose);
    CHECK(nearVec3(pose[1].position, 0.0f, 3.0f, 0.0f));
    CHECK(nearQuat(pose[1].rotation, quarter));
    CHECK(nearVec3(pose[1].scale, 3.0f, 3.0f, 3.0f));

    pose = bind;
    clip.sample(5.0f, pose);
    CHECK(nearVec3(pose[1].position, 0.0f, 3.0f, 0.0f));
    CHECK(nearQuat(pose[1].rotation, quarter));

    pose = bind;
    clip.sample(-2.0f, pose);
    CHECK(nearVec3(pose[1].position, 0.0f, 1.0f, 0.0f));
    CHECK(nearQuat(pose[1].rotation, Math::Quaternion()));

    pose = bind;
    clip.sample(0.5f, pose);
    CHECK(nearVec3(pose[0].position, bind[0].position.x, bind[0].position.y, bind[0].position.z));
    CHECK(nearVec3(pose[2].position, bind[2].position.x, bind[2].position.y, bind[2].position.z));

    AnimationClip shortest;
    BoneTrack opposite;
    opposite.bone = 0;
    opposite.times.push_back(0.0f);
    opposite.times.push_back(1.0f);
    opposite.positions.resize(2);
    opposite.scales.resize(2, Math::Vec3(1.0f, 1.0f, 1.0f));
    opposite.rotations.push_back(Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f));
    opposite.rotations.push_back(Math::Quaternion(0.0f, 0.0f, 0.0f, -1.0f));
    shortest.tracks().push_back(opposite);
    pose = bind;
    shortest.sample(0.5f, pose);
    CHECK(nearQuat(pose[0].rotation, Math::Quaternion()));

    AnimationClip single;
    BoneTrack hold;
    hold.bone = 2;
    hold.times.push_back(4.0f);
    hold.positions.push_back(Math::Vec3(7.0f, 8.0f, 9.0f));
    hold.rotations.push_back(Math::Quaternion(0.0f, 0.0f, 0.0f, 2.0f));
    hold.scales.push_back(Math::Vec3(2.0f, 2.0f, 2.0f));
    single.tracks().push_back(hold);
    BoneTrack outside;
    outside.bone = 9;
    outside.times.push_back(0.0f);
    outside.positions.resize(1);
    outside.rotations.resize(1);
    outside.scales.resize(1);
    single.tracks().push_back(outside);
    for (float time = -1.0f; time < 6.0f; time += 2.5f)
    {
        pose = bind;
        single.sample(time, pose);
        CHECK(nearVec3(pose[2].position, 7.0f, 8.0f, 9.0f));
        CHECK(nearVec3(pose[2].scale, 2.0f, 2.0f, 2.0f));
        CHECK(nearQuat(pose[2].rotation, Math::Quaternion()));
        CHECK(nearVec3(pose[1].position, 0.0f, 1.0f, 0.0f));
    }

    AnimationClip partial;
    BoneTrack onlyRotation;
    onlyRotation.bone = 1;
    onlyRotation.times.push_back(0.0f);
    onlyRotation.times.push_back(1.0f);
    onlyRotation.positions.push_back(bind[1].position);
    onlyRotation.positions.push_back(bind[1].position);
    onlyRotation.scales.push_back(bind[1].scale);
    onlyRotation.scales.push_back(bind[1].scale);
    onlyRotation.rotations.push_back(Math::Quaternion());
    onlyRotation.rotations.push_back(quarter);
    partial.tracks().push_back(onlyRotation);
    pose = bind;
    partial.sample(0.5f, pose);
    CHECK(nearVec3(pose[1].position, 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(pose[1].scale, 1.0f, 1.0f, 1.0f));
    CHECK(nearQuat(pose[1].rotation, eighth));
}

void test_keyframes()
{
    AnimationClip clip;
    LocalPose pose;
    pose.position = Math::Vec3(0.0f, 1.0f, 0.0f);
    clip.setKeyframe(1, 1.0f, pose);
    pose.position = Math::Vec3(0.0f, 3.0f, 0.0f);
    clip.setKeyframe(1, 3.0f, pose);
    pose.position = Math::Vec3(0.0f, 2.0f, 0.0f);
    clip.setKeyframe(1, 2.0f, pose);
    pose.position = Math::Vec3(0.0f, 0.0f, 0.0f);
    clip.setKeyframe(1, 0.0f, pose);
    clip.setKeyframe(-1, 0.0f, pose);

    CHECK(clip.tracks().size() == 1);
    CHECK(near(clip.duration(), 3.0f));
    if (clip.tracks().size() != 1) return;
    BoneTrack& track = clip.tracks()[0];
    CHECK(track.bone == 1);
    CHECK(track.times.size() == 4 && track.positions.size() == 4 && track.rotations.size() == 4 &&
            track.scales.size() == 4);
    if (track.times.size() != 4 || track.positions.size() != 4) return;
    for (int i = 0; i < 4; ++i)
    {
        CHECK(near(track.times[i], static_cast<float>(i)));
        CHECK(nearVec3(track.positions[i], 0.0f, static_cast<float>(i), 0.0f));
    }

    pose.position = Math::Vec3(5.0f, 5.0f, 5.0f);
    clip.setKeyframe(1, 1.99999f, pose);
    CHECK(track.times.size() == 4);
    CHECK(nearVec3(track.positions[2], 5.0f, 5.0f, 5.0f));

    clip.setKeyframe(4, 0.5f, pose);
    CHECK(clip.tracks().size() == 2);
    CHECK(clip.tracks()[1].bone == 4);

    clip.removeKeyframe(1, 1.5f);
    CHECK(clip.tracks()[0].times.size() == 4);
    clip.removeKeyframe(7, 1.0f);
    clip.removeKeyframe(1, 1.0f);
    CHECK(clip.tracks()[0].times.size() == 3);
    CHECK(clip.tracks()[0].positions.size() == 3 && clip.tracks()[0].rotations.size() == 3 &&
            clip.tracks()[0].scales.size() == 3);
    CHECK(near(clip.tracks()[0].times[0], 0.0f) && near(clip.tracks()[0].times[1], 2.0f) &&
            near(clip.tracks()[0].times[2], 3.0f));
    CHECK(near(clip.duration(), 3.0f));

    clip.setDuration(-4.0f);
    CHECK(clip.duration() == 0.0f);
}

void test_events()
{
    AnimationClip clip;
    clip.setDuration(2.0f);
    clip.addEvent(1.5f, "land");
    clip.addEvent(0.5f, "step");
    clip.addEvent(1.5f, "dust");
    clip.addEvent(9.0f, "late");
    clip.addEvent(-1.0f, "early");
    clip.addEvent(1.0f, "");
    clip.addEvent(0.0f / 0.0f, "nan");
    CHECK(clip.events().size() == 5);
    if (clip.events().size() != 5) return;
    CHECK(clip.events()[0].name == "early" && near(clip.events()[0].time, 0.0f));
    CHECK(clip.events()[1].name == "step" && near(clip.events()[1].time, 0.5f));
    CHECK(clip.events()[2].name == "land" && near(clip.events()[2].time, 1.5f));
    CHECK(clip.events()[3].name == "dust" && near(clip.events()[3].time, 1.5f));
    CHECK(clip.events()[4].name == "late" && near(clip.events()[4].time, 2.0f));

    clip.removeEvent("land");
    CHECK(clip.events().size() == 4);
    CHECK(clip.events()[2].name == "dust");
    clip.removeEvent("none");
    CHECK(clip.events().size() == 4);
    clip.clearEvents();
    CHECK(clip.events().empty());

    AnimationClip open;
    open.addEvent(5.0f, "free");
    CHECK(open.events().size() == 1 && near(open.events()[0].time, 5.0f));
}

} // namespace

int main()
{
    test_skeleton();
    test_pose();
    test_evaluate();
    test_sample();
    test_keyframes();
    test_events();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures != 0;
}
