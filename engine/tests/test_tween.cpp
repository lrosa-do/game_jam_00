#include "Check.h"

#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Tween.h"

#include <ct/vector.hpp>

#include <math.h>

using namespace prisma3d;

namespace
{

const float kTolerance = 1e-4f;
const float kPi = 3.14159265358979f;

bool near(float a, float b, float tolerance = kTolerance)
{
    return fabsf(a - b) <= tolerance;
}

bool nearVec3(const Math::Vec3& a, float x, float y, float z)
{
    return near(a.x, x) && near(a.y, y) && near(a.z, z);
}

void run(SceneTree& tree, double delta, int count)
{
    for (int i = 0; i < count; ++i) tree.process(delta);
}

float ease(TransitionType trans, EaseType type, float t)
{
    return Tween::runEquation(trans, type, t, 0.0f, 1.0f, 1.0f);
}

void testEasing()
{
    for (int i = 0; i < static_cast<int>(TransitionType::Max); ++i)
    {
        for (int j = 0; j < static_cast<int>(EaseType::Max); ++j)
        {
            const TransitionType trans = static_cast<TransitionType>(i);
            const EaseType type = static_cast<EaseType>(j);
            CHECK(near(ease(trans, type, 0.0f), 0.0f));

            float expected = 1.0f;
            if (trans == TransitionType::Expo && type == EaseType::In) expected = 0.999f;
            if (trans == TransitionType::Expo && type == EaseType::OutIn) expected = 0.9995f;
            CHECK(near(ease(trans, type, 1.0f), expected));
        }
    }

    CHECK(near(ease(TransitionType::Linear, EaseType::In, 0.25f), 0.25f));
    CHECK(near(ease(TransitionType::Linear, EaseType::OutIn, 0.75f), 0.75f));

    CHECK(near(ease(TransitionType::Quad, EaseType::In, 0.5f), 0.25f));
    CHECK(near(ease(TransitionType::Quad, EaseType::Out, 0.5f), 0.75f));
    CHECK(near(ease(TransitionType::Quad, EaseType::InOut, 0.25f), 0.125f));
    CHECK(near(ease(TransitionType::Quad, EaseType::InOut, 0.75f), 0.875f));

    CHECK(near(ease(TransitionType::Cubic, EaseType::In, 0.5f), 0.125f));
    CHECK(near(ease(TransitionType::Cubic, EaseType::Out, 0.5f), 0.875f));
    CHECK(near(ease(TransitionType::Cubic, EaseType::InOut, 0.25f), 0.0625f));
    CHECK(near(ease(TransitionType::Cubic, EaseType::InOut, 0.75f), 0.9375f));

    CHECK(near(ease(TransitionType::Sine, EaseType::In, 0.5f), 0.29289322f));
    CHECK(near(ease(TransitionType::Sine, EaseType::Out, 0.5f), 0.70710678f));
    CHECK(near(ease(TransitionType::Sine, EaseType::InOut, 0.5f), 0.5f));

    CHECK(near(ease(TransitionType::Expo, EaseType::In, 0.5f), 0.03025f));
    CHECK(near(ease(TransitionType::Expo, EaseType::Out, 0.5f), 0.96971875f));
    CHECK(near(ease(TransitionType::Expo, EaseType::InOut, 0.25f), 0.015125f));

    CHECK(near(ease(TransitionType::Back, EaseType::In, 0.5f), -0.0876975f));
    CHECK(near(ease(TransitionType::Back, EaseType::In, 0.25f), -0.0641366f));
    CHECK(near(ease(TransitionType::Back, EaseType::Out, 0.5f), 1.0876975f));
    CHECK(near(ease(TransitionType::Back, EaseType::Out, 0.75f), 1.0641366f));

    CHECK(near(ease(TransitionType::Bounce, EaseType::Out, 0.25f), 0.47265625f));
    CHECK(near(ease(TransitionType::Bounce, EaseType::Out, 0.5f), 0.765625f));
    CHECK(near(ease(TransitionType::Bounce, EaseType::Out, 0.9f), 0.988125f));
    CHECK(near(ease(TransitionType::Bounce, EaseType::Out, 0.95f), 0.98453125f));

    CHECK(near(Tween::runEquation(TransitionType::Quad, EaseType::In, 1.0f, 10.0f, 20.0f, 2.0f),
            15.0f));
    CHECK(near(Tween::runEquation(TransitionType::Bounce, EaseType::In, 0.3f, 10.0f, 20.0f, 0.0f),
            30.0f));

    const TweenValue half = Tween::interpolateValue(Math::Vec3(0.0f, 0.0f, 0.0f),
            Math::Vec3(10.0f, 20.0f, 30.0f), 0.5, 1.0, TransitionType::Linear, EaseType::In);
    CHECK(half.type() == TweenValue::Type::Vec3);
    CHECK(nearVec3(half.asVec3(), 5.0f, 10.0f, 15.0f));
}

void testSequence()
{
    SceneTree tree;
    tree.initialize();
    Node3D* node = tree.getRoot()->createChild<Node3D>();

    Tween* tween = tree.createTween();
    CHECK(tween->isRunning());
    CHECK(tween->isValid());
    CHECK(tween->tweenPosition(node, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0) != nullptr);
    CHECK(tween->tweenPosition(node, Math::Vec3(10.0f, 10.0f, 0.0f), 1.0) != nullptr);
    CHECK(tween->hasTweeners());

    int finished = 0;
    ct::Vector<int> steps;
    tween->finished = [&finished]() { ++finished; };
    tween->stepFinished = [&steps](int index) { steps.push_back(index); };

    CHECK(nearVec3(node->getPosition(), 0.0f, 0.0f, 0.0f));

    run(tree, 0.25, 1);
    CHECK(nearVec3(node->getPosition(), 2.5f, 0.0f, 0.0f));
    run(tree, 0.25, 1);
    CHECK(nearVec3(node->getPosition(), 5.0f, 0.0f, 0.0f));
    run(tree, 0.25, 2);
    CHECK(node->getPosition().x == 10.0f);
    CHECK(node->getPosition().y == 0.0f);
    CHECK(steps.size() == 1 && steps[0] == 0);
    run(tree, 0.25, 1);
    CHECK(nearVec3(node->getPosition(), 10.0f, 2.5f, 0.0f));
    run(tree, 0.25, 3);
    CHECK(node->getPosition().x == 10.0f);
    CHECK(node->getPosition().y == 10.0f);
    CHECK(finished == 1);
    CHECK(steps.size() == 2 && steps[1] == 1);
    CHECK(near(static_cast<float>(tween->getTotalElapsedTime()), 2.0f));
    CHECK(!tween->isRunning());
    CHECK(tween->isValid());
    CHECK(tween->isPersistent());
    CHECK(tree.getTweenCount() == 1);

    run(tree, 0.25, 2);
    CHECK(finished == 1);
    CHECK(tree.hasTween(tween));
    CHECK(node->getPosition().x == 10.0f);

    node->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    tween->play();
    CHECK(tween->isRunning());
    run(tree, 0.25, 9);
    CHECK(finished == 2);
    CHECK(!tween->isRunning());
    CHECK(node->getPosition().x == 10.0f);
    CHECK(node->getPosition().y == 10.0f);

    tween->kill();
    CHECK(!tween->isValid());
    CHECK(tree.hasTween(tween));
    run(tree, 0.25, 1);
    CHECK(!tree.hasTween(tween));
    CHECK(tree.getTweenCount() == 0);
    CHECK(finished == 2);
}

void testOnce()
{
    SceneTree tree;
    tree.initialize();
    Node3D* node = tree.getRoot()->createChild<Node3D>();

    int finished = 0;
    tree.tweenOnce(
            [node, &finished](Tween& tween)
            {
                CHECK(!tween.isPersistent());
                CHECK(tween.getBoundNode() == nullptr);
                tween.tweenPosition(node, Math::Vec3(4.0f, 0.0f, 0.0f), 1.0);
                tween.finished = [&finished]() { ++finished; };
            });
    CHECK(tree.getTweenCount() == 1);
    tree.process(0.5);
    CHECK(nearVec3(node->getPosition(), 2.0f, 0.0f, 0.0f));
    CHECK(finished == 0);
    tree.process(0.5);
    CHECK(node->getPosition().x == 4.0f);
    CHECK(finished == 1);
    tree.process(0.1);
    CHECK(tree.getTweenCount() == 0);
    CHECK(finished == 1);

    Node3D* owner = tree.getRoot()->createChild<Node3D>();
    owner->tweenOnce(
            [node, owner](Tween& tween)
            {
                CHECK(tween.getBoundNode() == owner);
                tween.tweenPosition(node, Math::Vec3(8.0f, 0.0f, 0.0f), 1.0);
            });
    CHECK(tree.getTweenCount() == 1);
    owner->queueFree();
    tree.process(0.25);
    CHECK(nearVec3(node->getPosition(), 5.0f, 0.0f, 0.0f));
    tree.process(0.25);
    CHECK(tree.getTweenCount() == 0);
    CHECK(nearVec3(node->getPosition(), 5.0f, 0.0f, 0.0f));

    tree.tweenOnce(ct::Function<void(Tween&)>());
    CHECK(tree.getTweenCount() == 0);
}

void testLeftover()
{
    SceneTree tree;
    tree.initialize();
    Node3D* node = tree.getRoot()->createChild<Node3D>();

    Tween* tween = tree.createTween();
    tween->tweenPosition(node, Math::Vec3(3.0f, 0.0f, 0.0f), 0.3);
    tween->tweenPosition(node, Math::Vec3(3.0f, 6.0f, 0.0f), 0.3);

    tree.process(0.5);
    CHECK(nearVec3(node->getPosition(), 3.0f, 4.0f, 0.0f));
    tree.process(0.5);
    CHECK(node->getPosition().x == 3.0f);
    CHECK(node->getPosition().y == 6.0f);
}

void testParallel()
{
    SceneTree tree;
    tree.initialize();
    Node3D* a = tree.getRoot()->createChild<Node3D>();
    Node3D* b = tree.getRoot()->createChild<Node3D>();

    {
        Tween* tween = tree.createTween();
        tween->tweenPosition(a, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0);
        tween->parallel()->tweenPosition(b, Math::Vec3(0.0f, 10.0f, 0.0f), 2.0);
        int stepCount = 0;
        int finished = 0;
        tween->stepFinished = [&stepCount](int) { ++stepCount; };
        tween->finished = [&finished]() { ++finished; };

        run(tree, 0.5, 2);
        CHECK(nearVec3(a->getPosition(), 10.0f, 0.0f, 0.0f));
        CHECK(nearVec3(b->getPosition(), 0.0f, 5.0f, 0.0f));
        CHECK(stepCount == 0);
        run(tree, 0.5, 2);
        CHECK(nearVec3(b->getPosition(), 0.0f, 10.0f, 0.0f));
        CHECK(stepCount == 1);
        CHECK(finished == 1);
    }

    a->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    b->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));

    {
        Tween* tween = tree.createTween();
        tween->setParallel(true);
        tween->tweenPosition(a, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0);
        tween->tweenPosition(b, Math::Vec3(0.0f, 10.0f, 0.0f), 2.0);
        run(tree, 0.5, 1);
        CHECK(nearVec3(a->getPosition(), 5.0f, 0.0f, 0.0f));
        CHECK(nearVec3(b->getPosition(), 0.0f, 2.5f, 0.0f));
        run(tree, 0.5, 3);
        CHECK(!tween->isRunning());
    }

    a->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    b->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));

    {
        Tween* tween = tree.createTween();
        tween->setParallel(true);
        tween->tweenPosition(a, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0);
        tween->chain()->tweenPosition(b, Math::Vec3(0.0f, 10.0f, 0.0f), 1.0);
        run(tree, 0.5, 1);
        CHECK(nearVec3(a->getPosition(), 5.0f, 0.0f, 0.0f));
        CHECK(nearVec3(b->getPosition(), 0.0f, 0.0f, 0.0f));
        run(tree, 0.5, 2);
        CHECK(nearVec3(b->getPosition(), 0.0f, 5.0f, 0.0f));
        run(tree, 0.5, 1);
        CHECK(nearVec3(b->getPosition(), 0.0f, 10.0f, 0.0f));
    }
}

void testOptions()
{
    SceneTree tree;
    tree.initialize();
    Node3D* a = tree.getRoot()->createChild<Node3D>();
    Node3D* b = tree.getRoot()->createChild<Node3D>();

    {
        Tween* tween = tree.createTween();
        tween->tweenPosition(a, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0)
                ->from(Math::Vec3(100.0f, 0.0f, 0.0f));
        run(tree, 0.5, 1);
        CHECK(nearVec3(a->getPosition(), 55.0f, 0.0f, 0.0f));
        run(tree, 0.5, 1);
        CHECK(a->getPosition().x == 10.0f);
    }

    a->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    b->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));

    {
        Tween* tween = tree.createTween();
        tween->setParallel(true);
        tween->tweenInterval(1.0);
        tween->chain()->tweenPosition(a, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0)->fromCurrent();
        tween->tweenPosition(b, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0);
        a->setPosition(Math::Vec3(4.0f, 0.0f, 0.0f));
        b->setPosition(Math::Vec3(4.0f, 0.0f, 0.0f));
        run(tree, 0.5, 3);
        CHECK(nearVec3(a->getPosition(), 5.0f, 0.0f, 0.0f));
        CHECK(nearVec3(b->getPosition(), 7.0f, 0.0f, 0.0f));
        tween->kill();
    }

    a->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));

    {
        Tween* tween = tree.createTween();
        tween->tweenPosition(a, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0)->asRelative();
        run(tree, 0.5, 1);
        CHECK(nearVec3(a->getPosition(), 6.0f, 0.0f, 0.0f));
        run(tree, 0.5, 1);
        CHECK(a->getPosition().x == 11.0f);
    }

    a->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));

    {
        Tween* tween = tree.createTween();
        tween->tweenPosition(a, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0)->setDelay(0.5);
        run(tree, 0.25, 1);
        CHECK(nearVec3(a->getPosition(), 0.0f, 0.0f, 0.0f));
        a->setPosition(Math::Vec3(2.0f, 0.0f, 0.0f));
        run(tree, 0.75, 1);
        CHECK(nearVec3(a->getPosition(), 6.0f, 0.0f, 0.0f));
        run(tree, 0.5, 1);
        CHECK(a->getPosition().x == 10.0f);
    }

    a->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    b->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));

    {
        Tween* tween = tree.createTween();
        tween->setParallel(true);
        tween->setTrans(TransitionType::Quad)->setEase(EaseType::Out);
        CHECK(tween->getTrans() == TransitionType::Quad);
        CHECK(tween->getEase() == EaseType::Out);
        tween->tweenPosition(a, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0);
        tween->tweenPosition(b, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0)
                ->setTrans(TransitionType::Cubic)
                ->setEase(EaseType::In);
        tween->setTrans(TransitionType::Sine);
        run(tree, 0.5, 1);
        CHECK(nearVec3(a->getPosition(), 7.5f, 0.0f, 0.0f));
        CHECK(nearVec3(b->getPosition(), 1.25f, 0.0f, 0.0f));
        tween->kill();
    }

    a->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));

    {
        Tween* tween = tree.createTween();
        tween->tweenPosition(a, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0)
                ->setCustomInterpolator([](float t) { return t * t; });
        run(tree, 0.5, 1);
        CHECK(nearVec3(a->getPosition(), 2.5f, 0.0f, 0.0f));
        run(tree, 0.5, 1);
        CHECK(a->getPosition().x == 10.0f);
    }

    CHECK(TweenValue(2).type() == TweenValue::Type::Float);
}

void testValues()
{
    SceneTree tree;
    tree.initialize();
    Node3D* node = tree.getRoot()->createChild<Node3D>();

    double number = 1.0;
    Math::Vec3 vector(0.0f, 0.0f, 0.0f);
    Math::Vec4 color(0.0f, 0.0f, 0.0f, 1.0f);
    Math::Quaternion rotation = Math::Quaternion::Identity();

    Tween* tween = tree.createTween();
    tween->setParallel(true);
    tween->tweenProperty([&number]() { return TweenValue(number); },
                 [&number](const TweenValue& value) { number = value.asFloat(); }, TweenValue(10.0),
                 1.0)
            ->asRelative();
    tween->tweenProperty([&vector]() { return TweenValue(vector); },
            [&vector](const TweenValue& value) { vector = value.asVec3(); },
            TweenValue(Math::Vec3(2.0f, 4.0f, 6.0f)), 1.0);
    tween->tweenProperty([&color]() { return TweenValue(color); }, [&color](const TweenValue& value)
            { color = value.asColor(); }, TweenValue(Math::Vec4(1.0f, 0.5f, 0.25f, 0.0f)), 1.0);
    tween->tweenProperty([&rotation]() { return TweenValue(rotation); },
            [&rotation](const TweenValue& value) { rotation = value.asQuaternion(); },
            TweenValue(Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 1.0f, 0.0f), kPi / 2)),
            1.0);
    tween->tweenRotation(node,
            Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 1.0f, 0.0f), kPi / 2), 1.0);

    run(tree, 0.5, 1);
    CHECK(near(static_cast<float>(number), 6.0f));
    CHECK(nearVec3(vector, 1.0f, 2.0f, 3.0f));
    CHECK(near(color.x, 0.5f) && near(color.y, 0.25f) && near(color.z, 0.125f) &&
            near(color.w, 0.5f));
    CHECK(near(rotation.x, 0.0f) && near(rotation.y, 0.3826834f) && near(rotation.z, 0.0f) &&
            near(rotation.w, 0.9238795f));
    const Math::Quaternion q = node->getQuaternion();
    CHECK(near(2.0f * acosf(q.w), kPi / 4));
    run(tree, 0.5, 1);
    CHECK(near(static_cast<float>(number), 11.0f));
    CHECK(near(rotation.y, 0.7071068f) && near(rotation.w, 0.7071068f));

    Tween* methods = tree.createTween();
    methods->setParallel(true);
    double floatValue = -1.0;
    Math::Vec3 vecValue;
    Math::Vec4 colorValue;
    Math::Quaternion quatValue;
    methods->tweenMethod([&floatValue](const TweenValue& v) { floatValue = v.asFloat(); }, 0.0,
            10.0, 1.0);
    methods->tweenMethod([&vecValue](const TweenValue& v) { vecValue = v.asVec3(); },
            Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(4.0f, 8.0f, 12.0f), 1.0);
    methods->tweenMethod([&colorValue](const TweenValue& v) { colorValue = v.asColor(); },
            Math::Vec4(0.0f, 0.0f, 0.0f, 0.0f), Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f), 1.0);
    methods->tweenMethod([&quatValue](const TweenValue& v) { quatValue = v.asQuaternion(); },
            Math::Quaternion::Identity(),
            Math::Quaternion::FromAxisAngle(Math::Vec3(1.0f, 0.0f, 0.0f), kPi * 2.0f / 3.0f), 1.0);
    run(tree, 0.5, 1);
    CHECK(near(static_cast<float>(floatValue), 5.0f));
    CHECK(nearVec3(vecValue, 2.0f, 4.0f, 6.0f));
    CHECK(near(colorValue.x, 0.5f) && near(colorValue.w, 0.5f));
    CHECK(near(quatValue.x, 0.5f) && near(quatValue.w, 0.8660254f));
    run(tree, 0.5, 1);
    CHECK(floatValue == 10.0);

    Tween* relative = tree.createTween();
    node->setQuaternion(Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 1.0f, 0.0f), kPi / 2));
    relative->tweenRotation(node,
                    Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 1.0f, 0.0f), kPi / 2), 1.0)
            ->asRelative();
    run(tree, 0.5, 1);
    const Math::Quaternion mid = node->getQuaternion();
    CHECK(near(mid.y, 0.9238795f) && near(mid.w, 0.3826834f));
    run(tree, 0.5, 1);
    const Math::Quaternion end = node->getQuaternion();
    CHECK(near(fabsf(end.y), 1.0f) && near(end.w, 0.0f));

    Tween* invalid = tree.createTween();
    CHECK(invalid->tweenProperty([]() { return TweenValue(1.0); }, [](const TweenValue&) {},
                  TweenValue(Math::Vec3(1.0f, 1.0f, 1.0f)), 1.0) == nullptr);
    CHECK(invalid->tweenMethod([](const TweenValue&) {}, 1.0, Math::Vec3(1.0f, 1.0f, 1.0f), 1.0) ==
            nullptr);
    PropertyTweener* property = invalid->tweenProperty([]() { return TweenValue(1.0); },
            [](const TweenValue&) {}, TweenValue(2.0), 1.0);
    CHECK(property != nullptr);
    CHECK(property->from(Math::Vec3(1.0f, 1.0f, 1.0f)) == nullptr);
    CHECK(Tween::interpolateValue(0.0, 1.0, 0.5, 1.0, TransitionType::Max, EaseType::In).type() ==
            TweenValue::Type::Float);
}

void testIntervalAndCallback()
{
    SceneTree tree;
    tree.initialize();

    ct::Vector<int> events;
    int intervalFinished = 0;
    Tween* tween = tree.createTween();
    tween->tweenCallback([&events]() { events.push_back(1); });
    IntervalTweener* interval = tween->tweenInterval(1.0);
    interval->finished = [&intervalFinished]() { ++intervalFinished; };
    tween->tweenCallback([&events]() { events.push_back(2); });
    tween->tweenCallback([&events]() { events.push_back(3); })->setDelay(0.5);

    tree.process(0.1);
    CHECK(events.size() == 1 && events[0] == 1);
    CHECK(intervalFinished == 0);
    run(tree, 0.1, 8);
    CHECK(events.size() == 1);
    tree.process(0.2);
    CHECK(events.size() == 2 && events[1] == 2);
    CHECK(intervalFinished == 1);
    tree.process(0.25);
    CHECK(events.size() == 2);
    tree.process(0.25);
    CHECK(events.size() == 3 && events[2] == 3);
    run(tree, 0.5, 1);
    CHECK(events.size() == 3);
    CHECK(intervalFinished == 1);
}

void testLoops()
{
    SceneTree tree;
    tree.initialize();

    {
        Tween* tween = tree.createTween();
        tween->setLoops(3);
        CHECK(tween->getLoopsLeft() == 3);
        double value = 0.0;
        tween->tweenMethod([&value](const TweenValue& v) { value = v.asFloat(); }, 0.0, 1.0, 1.0);
        ct::Vector<int> loops;
        int finished = 0;
        tween->loopFinished = [&loops](int count) { loops.push_back(count); };
        tween->finished = [&finished]() { ++finished; };

        run(tree, 0.5, 1);
        CHECK(near(static_cast<float>(value), 0.5f));
        run(tree, 0.5, 1);
        CHECK(loops.size() == 1 && loops[0] == 1);
        CHECK(tween->getLoopsLeft() == 2);
        run(tree, 0.5, 2);
        CHECK(loops.size() == 2 && loops[1] == 2);
        CHECK(tween->getLoopsLeft() == 1);
        CHECK(finished == 0);
        run(tree, 0.5, 2);
        CHECK(loops.size() == 2);
        CHECK(finished == 1);
        CHECK(tween->getLoopsLeft() == 0);
        CHECK(value == 1.0);
    }

    {
        Tween* tween = tree.createTween();
        tween->setLoops(0);
        CHECK(tween->getLoopsLeft() == -1);
        tween->tweenInterval(1.0);
        int loops = 0;
        int finished = 0;
        tween->loopFinished = [&loops](int) { ++loops; };
        tween->finished = [&finished]() { ++finished; };
        run(tree, 0.5, 10);
        CHECK(loops == 5);
        CHECK(finished == 0);
        CHECK(tween->isRunning());
        CHECK(tween->isValid());
        CHECK(tween->getLoopsLeft() == -1);
        tween->kill();
        tree.process(0.1);
        CHECK(!tree.hasTween(tween));
    }

    {
        Tween* tween = tree.createTween();
        tween->setLoops(0);
        int calls = 0;
        tween->tweenCallback([&calls]() { ++calls; });
        tree.process(0.5);
        CHECK(calls == 2);
        CHECK(!tree.hasTween(tween));

        Tween* manual = tree.createTween();
        manual->setLoops(-1);
        manual->tweenCallback([&calls]() { ++calls; });
        CHECK(!manual->customStep(0.5));
        CHECK(calls == 4);
    }
}

void testControl()
{
    SceneTree tree;
    tree.initialize();
    Node3D* node = tree.getRoot()->createChild<Node3D>();

    {
        Tween* tween = tree.createTween();
        tween->tweenPosition(node, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0);
        run(tree, 0.25, 1);
        CHECK(nearVec3(node->getPosition(), 2.5f, 0.0f, 0.0f));
        tween->pause();
        CHECK(!tween->isRunning());
        run(tree, 0.5, 1);
        CHECK(nearVec3(node->getPosition(), 2.5f, 0.0f, 0.0f));
        tween->play();
        CHECK(tween->isRunning());
        run(tree, 0.25, 1);
        CHECK(nearVec3(node->getPosition(), 5.0f, 0.0f, 0.0f));
        CHECK(near(static_cast<float>(tween->getTotalElapsedTime()), 0.5f));
        tween->kill();
        CHECK(!tween->isRunning());
        CHECK(!tween->isValid());
        run(tree, 0.25, 1);
        CHECK(nearVec3(node->getPosition(), 5.0f, 0.0f, 0.0f));
    }

    {
        Tween* tween = tree.createTween();
        double value = -1.0;
        int finished = 0;
        tween->tweenMethod([&value](const TweenValue& v) { value = v.asFloat(); }, 0.0, 1.0, 1.0);
        tween->finished = [&finished]() { ++finished; };
        run(tree, 0.5, 1);
        CHECK(near(static_cast<float>(value), 0.5f));
        tween->stop();
        CHECK(!tween->isRunning());
        CHECK(tween->getTotalElapsedTime() == 0.0);
        run(tree, 0.5, 1);
        CHECK(near(static_cast<float>(value), 0.5f));
        tween->play();
        run(tree, 0.25, 1);
        CHECK(near(static_cast<float>(value), 0.25f));
        CHECK(near(static_cast<float>(tween->getTotalElapsedTime()), 0.25f));
        run(tree, 0.75, 1);
        CHECK(finished == 1);
        CHECK(value == 1.0);
        tween->stop();
        tween->play();
        run(tree, 1.0, 1);
        CHECK(finished == 2);
    }

    {
        Tween* tween = tree.createTween();
        tween->setSpeedScale(2.0f);
        double value = 0.0;
        tween->tweenMethod([&value](const TweenValue& v) { value = v.asFloat(); }, 0.0, 1.0, 1.0);
        run(tree, 0.25, 1);
        CHECK(near(static_cast<float>(value), 0.5f));
        CHECK(near(static_cast<float>(tween->getTotalElapsedTime()), 0.5f));
        tween->kill();
    }

    {
        Tween* tween = tree.createTween();
        double value = 0.0;
        bool innerResult = false;
        Tween* raw = tween;
        tween->tweenMethod(
                [&value, &innerResult, raw](const TweenValue& v)
                {
                    value = v.asFloat();
                    innerResult = raw->customStep(0.5);
                },
                0.0, 1.0, 1.0);
        tween->pause();
        CHECK(tween->customStep(0.5));
        CHECK(near(static_cast<float>(value), 0.5f));
        CHECK(innerResult);
        CHECK(!tween->isRunning());
        CHECK(near(static_cast<float>(tween->getTotalElapsedTime()), 0.5f));
        run(tree, 1.0, 1);
        CHECK(near(static_cast<float>(value), 0.5f));
        tween->kill();
    }
}

void testProcessMode()
{
    SceneTree tree;
    tree.initialize();
    Node3D* node = tree.getRoot()->createChild<Node3D>();
    Node3D* other = tree.getRoot()->createChild<Node3D>();

    Tween* physics = tree.createTween();
    CHECK(physics->getProcessMode() == TweenProcessMode::Idle);
    physics->setProcessMode(TweenProcessMode::Physics);
    CHECK(physics->getProcessMode() == TweenProcessMode::Physics);
    physics->tweenPosition(node, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0);

    Tween* idle = tree.createTween();
    idle->tweenPosition(other, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0);

    tree.process(0.5);
    CHECK(nearVec3(node->getPosition(), 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(other->getPosition(), 5.0f, 0.0f, 0.0f));
    tree.physicsProcess(0.5);
    CHECK(nearVec3(node->getPosition(), 5.0f, 0.0f, 0.0f));
    CHECK(nearVec3(other->getPosition(), 5.0f, 0.0f, 0.0f));
}

void testPause()
{
    SceneTree tree;
    tree.initialize();
    Node3D* boundNode = tree.getRoot()->createChild<Node3D>();
    Node3D* nodes[4];
    for (int i = 0; i < 4; ++i) nodes[i] = tree.getRoot()->createChild<Node3D>();

    Tween* bound = tree.createTween();
    CHECK(bound->getPauseMode() == TweenPauseMode::Bound);
    bound->tweenPosition(nodes[0], Math::Vec3(10.0f, 0.0f, 0.0f), 1.0);

    Tween* stop = tree.createTween();
    stop->setPauseMode(TweenPauseMode::Stop);
    CHECK(stop->getPauseMode() == TweenPauseMode::Stop);
    stop->tweenPosition(nodes[1], Math::Vec3(10.0f, 0.0f, 0.0f), 1.0);

    Tween* process = tree.createTween();
    process->setPauseMode(TweenPauseMode::Process);
    process->tweenPosition(nodes[2], Math::Vec3(10.0f, 0.0f, 0.0f), 1.0);

    Tween* attached = boundNode->createTween();
    CHECK(attached->getBoundNode() == boundNode);
    attached->tweenPosition(nodes[3], Math::Vec3(10.0f, 0.0f, 0.0f), 1.0);

    tree.setPaused(true);
    tree.process(0.5);
    CHECK(nearVec3(nodes[0]->getPosition(), 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(nodes[1]->getPosition(), 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(nodes[2]->getPosition(), 5.0f, 0.0f, 0.0f));
    CHECK(nearVec3(nodes[3]->getPosition(), 0.0f, 0.0f, 0.0f));

    tree.setPaused(false);
    tree.process(0.5);
    CHECK(nearVec3(nodes[0]->getPosition(), 5.0f, 0.0f, 0.0f));
    CHECK(nearVec3(nodes[1]->getPosition(), 5.0f, 0.0f, 0.0f));
    CHECK(nearVec3(nodes[2]->getPosition(), 10.0f, 0.0f, 0.0f));
    CHECK(nearVec3(nodes[3]->getPosition(), 5.0f, 0.0f, 0.0f));
}

void testBoundNode()
{
    SceneTree tree;
    tree.initialize();
    Node3D holder;
    Node3D* target = tree.getRoot()->createChild<Node3D>();
    Node3D* bound = tree.getRoot()->createChild<Node3D>();

    Tween* tween = bound->createTween();
    tween->tweenPosition(target, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0);
    run(tree, 0.25, 1);
    CHECK(nearVec3(target->getPosition(), 2.5f, 0.0f, 0.0f));

    CHECK(bound->reparent(&holder));
    run(tree, 0.25, 2);
    CHECK(nearVec3(target->getPosition(), 2.5f, 0.0f, 0.0f));
    CHECK(tween->isValid());
    CHECK(tween->isRunning());

    CHECK(bound->reparent(tree.getRoot()));
    run(tree, 0.25, 1);
    CHECK(nearVec3(target->getPosition(), 5.0f, 0.0f, 0.0f));
}

void testBoundNodeOutsideTree()
{
    SceneTree tree;
    tree.initialize();
    Node3D holder;
    Node3D* target = tree.getRoot()->createChild<Node3D>();
    Node3D* orphan = holder.createChild<Node3D>();

    Tween* tween = orphan->createTween();
    CHECK(tween->getBoundNode() == orphan);
    tween->tweenPosition(target, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0);
    run(tree, 0.5, 2);
    CHECK(nearVec3(target->getPosition(), 0.0f, 0.0f, 0.0f));
    CHECK(tween->isValid());
    CHECK(tween->isRunning());
    CHECK(tween->customStep(0.5));
    CHECK(nearVec3(target->getPosition(), 0.0f, 0.0f, 0.0f));

    CHECK(orphan->reparent(tree.getRoot()));
    run(tree, 0.5, 1);
    CHECK(nearVec3(target->getPosition(), 5.0f, 0.0f, 0.0f));
}

void testBoundNodeFreed()
{
    SceneTree tree;
    tree.initialize();
    Node3D* target = tree.getRoot()->createChild<Node3D>();
    Node3D* bound = tree.getRoot()->createChild<Node3D>();

    Tween* tween = bound->createTween();
    tween->tweenPosition(target, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0);
    int finished = 0;
    tween->finished = [&finished]() { ++finished; };

    bound->queueFree();
    tree.process(0.25);
    CHECK(nearVec3(target->getPosition(), 2.5f, 0.0f, 0.0f));
    CHECK(tween->getBoundNode() == nullptr);
    tree.process(0.25);
    CHECK(nearVec3(target->getPosition(), 2.5f, 0.0f, 0.0f));
    CHECK(!tree.hasTween(tween));
    CHECK(finished == 0);

    Tween* paused = target->createTween();
    paused->tweenInterval(1.0);
    target->queueFree();
    tree.process(0.25);
    tree.setPaused(true);
    tree.process(0.25);
    tree.setPaused(false);
    tree.process(0.25);
    CHECK(!tree.hasTween(paused));
}

void testTargetFreed()
{
    SceneTree tree;
    tree.initialize();
    Node3D* doomed = tree.getRoot()->createChild<Node3D>();
    Node3D* survivor = tree.getRoot()->createChild<Node3D>();

    Tween* tween = tree.createTween();
    tween->setParallel(true);
    PropertyTweener* doomedTweener =
            tween->tweenPosition(doomed, Math::Vec3(10.0f, 0.0f, 0.0f), 2.0);
    tween->tweenPosition(survivor, Math::Vec3(10.0f, 0.0f, 0.0f), 2.0);
    int doomedFinished = 0;
    int finished = 0;
    doomedTweener->finished = [&doomedFinished]() { ++doomedFinished; };
    tween->finished = [&finished]() { ++finished; };

    doomed->queueFree();
    tree.process(0.5);
    CHECK(nearVec3(survivor->getPosition(), 2.5f, 0.0f, 0.0f));
    CHECK(doomedFinished == 0);
    tree.process(0.5);
    CHECK(nearVec3(survivor->getPosition(), 5.0f, 0.0f, 0.0f));
    CHECK(doomedFinished == 1);
    CHECK(finished == 0);
    tree.process(1.0);
    CHECK(survivor->getPosition().x == 10.0f);
    CHECK(doomedFinished == 1);
    CHECK(finished == 1);

    Node3D* later = tree.getRoot()->createChild<Node3D>();
    Tween* sequence = tree.createTween();
    sequence->tweenInterval(1.0);
    sequence->tweenPosition(later, Math::Vec3(10.0f, 0.0f, 0.0f), 1.0);
    int sequenceFinished = 0;
    sequence->finished = [&sequenceFinished]() { ++sequenceFinished; };
    later->queueFree();
    tree.process(0.5);
    tree.process(0.75);
    CHECK(sequenceFinished == 1);
    tree.process(0.1);
    CHECK(tree.hasTween(sequence));
    CHECK(sequence->isValid());
    CHECK(!sequence->isRunning());
}

void testNoTweeners()
{
    SceneTree tree;
    tree.initialize();

    Tween* tween = tree.createTween();
    CHECK(!tween->hasTweeners());
    int finished = 0;
    tween->finished = [&finished]() { ++finished; };
    CHECK(tween->isValid());
    CHECK(tree.getTweenCount() == 1);
    tree.process(0.1);
    CHECK(!tree.hasTween(tween));
    CHECK(finished == 0);
    CHECK(tree.getTweenCount() == 0);

    Tween* manual = tree.createTween();
    CHECK(!manual->customStep(0.1));
}

} // namespace

int main()
{
    testEasing();
    testSequence();
    testOnce();
    testLeftover();
    testParallel();
    testOptions();
    testValues();
    testIntervalAndCallback();
    testLoops();
    testControl();
    testProcessMode();
    testPause();
    testBoundNode();
    testBoundNodeOutsideTree();
    testBoundNodeFreed();
    testTargetFreed();
    testNoTweeners();

    if (failures)
    {
        printf("%d failures\n", failures);
        return 1;
    }
    printf("ok\n");
    return 0;
}
