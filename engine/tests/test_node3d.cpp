#include "Check.h"

#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneTree.h"

#include <ct/vector.hpp>

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

bool nearVec(const Math::Vec3& a, const Math::Vec3& b)
{
    return near(a.x, b.x) && near(a.y, b.y) && near(a.z, b.z);
}

bool nearVec4(const Math::Vec4& a, const Math::Vec4& b)
{
    return near(a.x, b.x) && near(a.y, b.y) && near(a.z, b.z) && near(a.w, b.w);
}

bool nearMat(const Math::Mat4& a, const Math::Mat4& b)
{
    return nearVec4(a.col0, b.col0) && nearVec4(a.col1, b.col1) && nearVec4(a.col2, b.col2) &&
           nearVec4(a.col3, b.col3);
}

Math::Mat4 makeMat(const Math::Vec3& x, const Math::Vec3& y, const Math::Vec3& z,
        const Math::Vec3& t)
{
    return Math::Mat4(Math::Vec4(x, 0.0f), Math::Vec4(y, 0.0f), Math::Vec4(z, 0.0f),
            Math::Vec4(t, 1.0f));
}

Math::Vec3 basisX(const Math::Mat4& m)
{
    return m.col0.xyz();
}

Math::Vec3 basisY(const Math::Mat4& m)
{
    return m.col1.xyz();
}

Math::Vec3 basisZ(const Math::Mat4& m)
{
    return m.col2.xyz();
}

class RecordingNode3D : public Node3D
{
public:
    int count(int what) const
    {
        int total = 0;
        for (std::size_t i = 0; i < events.size(); ++i)
        {
            if (events[i] == what) ++total;
        }
        return total;
    }

    ct::Vector<int> events;

protected:
    void notificationHandler(int what) override
    {
        if (what != kNotificationChildOrderChanged && what != kNotificationParented &&
                what != kNotificationUnparented && what != kNotificationPostEnterTree &&
                what != kNotificationReady)
            events.push_back(what);
        Node3D::notificationHandler(what);
    }
};

void testLocalComposition()
{
    Node3D node;
    CHECK(nearMat(node.getTransform(), Math::Mat4::Identity()));
    CHECK(nearVec(node.getScale(), Math::Vec3(1.0f, 1.0f, 1.0f)));
    CHECK(nearVec(node.getPosition(), Math::Vec3(0.0f, 0.0f, 0.0f)));

    node.setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    node.setScale(Math::Vec3(2.0f, 3.0f, 4.0f));
    node.setQuaternion(Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 1.0f, 0.0f), kHalfPi));

    const Math::Mat4 expected = makeMat(Math::Vec3(0.0f, 0.0f, -2.0f), Math::Vec3(0.0f, 3.0f, 0.0f),
            Math::Vec3(4.0f, 0.0f, 0.0f), Math::Vec3(1.0f, 2.0f, 3.0f));
    CHECK(nearMat(node.getTransform(), expected));
    CHECK(nearVec(node.getPosition(), Math::Vec3(1.0f, 2.0f, 3.0f)));
    CHECK(nearVec(node.getScale(), Math::Vec3(2.0f, 3.0f, 4.0f)));

    node.setPosition(Math::Vec3(-1.0f, 0.5f, 7.0f));
    const Math::Mat4 moved = makeMat(Math::Vec3(0.0f, 0.0f, -2.0f), Math::Vec3(0.0f, 3.0f, 0.0f),
            Math::Vec3(4.0f, 0.0f, 0.0f), Math::Vec3(-1.0f, 0.5f, 7.0f));
    CHECK(nearMat(node.getTransform(), moved));

    Node3D rolled;
    rolled.setRotation(Math::Vec3(0.0f, 0.0f, kHalfPi));
    CHECK(nearMat(rolled.getTransform(),
            makeMat(Math::Vec3(0.0f, 1.0f, 0.0f), Math::Vec3(-1.0f, 0.0f, 0.0f),
                    Math::Vec3(0.0f, 0.0f, 1.0f), Math::Vec3(0.0f, 0.0f, 0.0f))));

    Node3D pitched;
    pitched.setRotation(Math::Vec3(kHalfPi, 0.0f, 0.0f));
    CHECK(nearMat(pitched.getTransform(),
            makeMat(Math::Vec3(1.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 1.0f),
                    Math::Vec3(0.0f, -1.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 0.0f))));

    Node3D euler;
    euler.setRotation(Math::Vec3(0.3f, 0.4f, 0.5f));
    CHECK(nearVec(euler.getRotation(), Math::Vec3(0.3f, 0.4f, 0.5f)));
}

Math::Mat4 eulerMatrix(const Math::Vec3& angles, const Math::Vec3& position,
        const Math::Vec3& scale)
{
    return Math::Mat4::Translation(position) * Math::Mat4::RotationZ(angles.z) *
           Math::Mat4::RotationY(angles.y) * Math::Mat4::RotationX(angles.x) *
           Math::Mat4::Scale(scale);
}

bool exactVec(const Math::Vec3& a, const Math::Vec3& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

void testStoredEuler()
{
    const float degrees = 0.017453292519943295f;

    Node3D flipped;
    flipped.setRotation(Math::Vec3(0.0f, 180.0f * degrees, 0.0f));
    CHECK(exactVec(flipped.getRotation(), Math::Vec3(0.0f, 180.0f * degrees, 0.0f)));
    flipped.setRotationDegrees(Math::Vec3(0.0f, 180.0f, 0.0f));
    CHECK(nearVec(flipped.getRotationDegrees(), Math::Vec3(0.0f, 180.0f, 0.0f)));

    const Math::Vec3 angles(30.0f * degrees, 120.0f * degrees, 40.0f * degrees);
    const Math::Vec3 position(1.0f, -2.0f, 3.0f);
    const Math::Vec3 scale(2.0f, 3.0f, 0.5f);
    Node3D node;
    node.setPosition(position);
    node.setScale(scale);
    node.setRotation(angles);
    CHECK(exactVec(node.getRotation(), angles));
    CHECK(nearMat(node.getTransform(), eulerMatrix(angles, position, scale)));
    const Math::Quaternion wanted = Math::Quaternion::FromEulerAngles(angles.x, angles.y, angles.z);
    CHECK(nearMat(Math::Mat4(wanted.ToMat3()), Math::Mat4(node.getQuaternion().ToMat3())));

    node.setScale(Math::Vec3(1.0f, 1.0f, 1.0f));
    node.setPosition(Math::Vec3(9.0f, 8.0f, 7.0f));
    CHECK(exactVec(node.getRotation(), angles));
    CHECK(nearMat(node.getTransform(),
            eulerMatrix(angles, Math::Vec3(9.0f, 8.0f, 7.0f), Math::Vec3(1.0f, 1.0f, 1.0f))));
    const Math::Vec3 inDegrees = node.getRotationDegrees();
    CHECK(near(inDegrees.x, 30.0f) && near(inDegrees.y, 120.0f) && near(inDegrees.z, 40.0f));
    CHECK(nearVec(node.getRotation() * (1.0f / degrees), inDegrees));

    node.setRotation(Math::Vec3(7.0f, -9.0f, 11.0f));
    CHECK(exactVec(node.getRotation(), Math::Vec3(7.0f, -9.0f, 11.0f)));

    const Math::Quaternion turned = Math::Quaternion::FromEulerAngles(0.2f, 1.9f, -0.4f);
    node.setRotation(angles);
    node.setQuaternion(turned);
    CHECK(nearVec(node.getRotation(), turned.Normalized().ToEulerAngles()));
    node.setScale(scale);
    node.setPosition(position);
    CHECK(nearVec(node.getRotation(), turned.Normalized().ToEulerAngles()));

    node.setRotation(angles);
    node.rotateY(0.3f);
    {
        Math::Quaternion expected =
                Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 1.0f, 0.0f), 0.3f) * wanted;
        CHECK(nearVec(node.getRotation(), expected.Normalized().ToEulerAngles()));
    }

    node.setRotation(angles);
    node.rotateObjectLocal(Math::Vec3(1.0f, 0.0f, 0.0f), 0.4f);
    {
        const Math::Mat3 basis = node.getBasis();
        Math::Vec3 length(basis.col0.Length(), basis.col1.Length(), basis.col2.Length());
        const Math::Quaternion expected = Math::Quaternion::FromMat3(
                Math::Mat3(basis.col0 / length.x, basis.col1 / length.y, basis.col2 / length.z));
        CHECK(nearVec(node.getRotation(), expected.Normalized().ToEulerAngles()));
    }

    node.setRotation(angles);
    node.setTransform(eulerMatrix(Math::Vec3(0.1f, 0.2f, 0.3f), position, scale));
    CHECK(nearVec(node.getRotation(),
            Math::Quaternion::FromEulerAngles(0.1f, 0.2f, 0.3f).Normalized().ToEulerAngles()));
    CHECK(nearVec(node.getRotation(), Math::Vec3(0.1f, 0.2f, 0.3f)));

    node.setRotation(angles);
    node.setBasis(Math::Mat3::RotationY(0.5f));
    CHECK(nearVec(node.getRotation(), Math::Vec3(0.0f, 0.5f, 0.0f)));

    node.setRotation(angles);
    node.setIdentity();
    CHECK(nearVec(node.getRotation(), Math::Vec3(0.0f, 0.0f, 0.0f)));

    {
        SceneTree tree;
        tree.initialize();
        Node3D* parent = tree.getRoot()->createChild<Node3D>();
        Node3D* child = parent->createChild<Node3D>();
        parent->setPosition(Math::Vec3(5.0f, 0.0f, 0.0f));
        parent->setRotation(Math::Vec3(0.0f, 180.0f * degrees, 0.0f));
        child->setPosition(position);
        child->setScale(scale);
        child->setRotation(angles);
        CHECK(exactVec(parent->getRotation(), Math::Vec3(0.0f, 180.0f * degrees, 0.0f)));
        CHECK(exactVec(child->getRotation(), angles));
        CHECK(nearMat(child->getTransform(), eulerMatrix(angles, position, scale)));
        const Math::Mat4 parentMatrix = eulerMatrix(Math::Vec3(0.0f, 180.0f * degrees, 0.0f),
                Math::Vec3(5.0f, 0.0f, 0.0f), Math::Vec3(1.0f, 1.0f, 1.0f));
        CHECK(nearMat(child->getGlobalTransform(),
                parentMatrix * eulerMatrix(angles, position, scale)));

        child->lookAt(Math::Vec3(10.0f, 4.0f, -3.0f));
        const Math::Quaternion looked = child->getQuaternion();
        CHECK(nearVec(child->getRotation(), looked.Normalized().ToEulerAngles()));
        CHECK(nearVec(child->getScale(), scale));
        const Math::Vec3 afterLook = child->getRotation();
        child->setScale(Math::Vec3(1.0f, 2.0f, 3.0f));
        child->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
        CHECK(exactVec(child->getRotation(), afterLook));

        child->setRotation(angles);
        child->setGlobalTransform(Math::Mat4::Translation(Math::Vec3(1.0f, 1.0f, 1.0f)));
        CHECK(nearVec(child->getRotation(), child->getQuaternion().ToEulerAngles()));
    }
}

void testSetTransformDecomposition()
{
    const Math::Mat4 source = makeMat(Math::Vec3(0.0f, 0.0f, -2.0f), Math::Vec3(0.0f, 3.0f, 0.0f),
            Math::Vec3(4.0f, 0.0f, 0.0f), Math::Vec3(1.0f, 2.0f, 3.0f));

    Node3D node;
    node.setTransform(source);
    CHECK(nearMat(node.getTransform(), source));
    CHECK(nearVec(node.getPosition(), Math::Vec3(1.0f, 2.0f, 3.0f)));
    CHECK(nearVec(node.getScale(), Math::Vec3(2.0f, 3.0f, 4.0f)));
    CHECK(nearVec(node.getQuaternion() * Math::Vec3(1.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.0f, -1.0f)));

    node.setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    CHECK(nearMat(node.getTransform(),
            makeMat(Math::Vec3(0.0f, 0.0f, -2.0f), Math::Vec3(0.0f, 3.0f, 0.0f),
                    Math::Vec3(4.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 0.0f))));

    Node3D mirrored;
    mirrored.setTransform(Math::Mat4::Scale(Math::Vec3(-1.0f, 2.0f, 2.0f)));
    CHECK(near(mirrored.getScale().x, -1.0f));
    CHECK(near(mirrored.getScale().y, 2.0f));
    mirrored.setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));
    CHECK(nearMat(mirrored.getTransform(),
            makeMat(Math::Vec3(-1.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 2.0f, 0.0f),
                    Math::Vec3(0.0f, 0.0f, 2.0f), Math::Vec3(1.0f, 0.0f, 0.0f))));
}

void testParentChildGlobals()
{
    SceneTree tree;
    tree.initialize();

    Node3D* parent = tree.getRoot()->createChild<Node3D>();
    Node3D* child = parent->createChild<Node3D>();
    Node3D* grandchild = child->createChild<Node3D>();

    parent->setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));
    parent->setQuaternion(Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 1.0f, 0.0f), kHalfPi));
    child->setPosition(Math::Vec3(0.0f, 0.0f, 5.0f));

    CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(15.0f, 0.0f, 0.0f)));
    CHECK(nearMat(child->getGlobalTransform(),
            parent->getGlobalTransform() * child->getTransform()));

    parent->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(5.0f, 0.0f, 0.0f)));

    parent->rotateY(-kHalfPi);
    CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(0.0f, 0.0f, 5.0f)));

    parent->setQuaternion(Math::Quaternion::Identity());
    parent->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    parent->setScale(Math::Vec3(2.0f, 2.0f, 2.0f));
    child->setPosition(Math::Vec3(1.0f, 1.0f, 1.0f));
    child->setScale(Math::Vec3(3.0f, 3.0f, 3.0f));
    grandchild->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));

    CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(3.0f, 4.0f, 5.0f)));
    CHECK(nearVec(grandchild->getGlobalPosition(), Math::Vec3(9.0f, 4.0f, 5.0f)));
    CHECK(nearVec(basisX(grandchild->getGlobalTransform()), Math::Vec3(6.0f, 0.0f, 0.0f)));

    CHECK(nearVec(grandchild->getGlobalPosition(), Math::Vec3(9.0f, 4.0f, 5.0f)));
    parent->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    CHECK(nearVec(grandchild->getGlobalPosition(), Math::Vec3(8.0f, 2.0f, 2.0f)));
    CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(2.0f, 2.0f, 2.0f)));

    parent->setScale(Math::Vec3(1.0f, 1.0f, 1.0f));
    child->setScale(Math::Vec3(1.0f, 1.0f, 1.0f));
    CHECK(nearVec(grandchild->getGlobalPosition(), Math::Vec3(2.0f, 1.0f, 1.0f)));
    CHECK(nearMat(grandchild->getGlobalTransform(),
            parent->getTransform() * child->getTransform() * grandchild->getTransform()));

    Node3D* other = tree.getRoot()->createChild<Node3D>();
    other->setPosition(Math::Vec3(0.0f, 100.0f, 0.0f));
    child->reparent(other);
    CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(1.0f, 101.0f, 1.0f)));
    CHECK(nearVec(grandchild->getGlobalPosition(), Math::Vec3(2.0f, 101.0f, 1.0f)));
}

void testPlainNodeBetween()
{
    SceneTree tree;
    tree.initialize();

    Node3D* parent = tree.getRoot()->createChild<Node3D>();
    Node* plain = parent->createChild<Node>();
    Node3D* child = plain->createChild<Node3D>();

    CHECK(child->getParentNode3D() == nullptr);
    CHECK(plain->getParent() == parent);
    parent->setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));
    child->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));
    CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(1.0f, 0.0f, 0.0f)));

    Node3D* direct = parent->createChild<Node3D>();
    CHECK(direct->getParentNode3D() == parent);
    direct->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));
    CHECK(nearVec(direct->getGlobalPosition(), Math::Vec3(11.0f, 0.0f, 0.0f)));
}

void testDetachedGlobals()
{
    Node3D parent;
    Node3D* child = parent.createChild<Node3D>();
    parent.setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));
    child->setPosition(Math::Vec3(0.0f, 2.0f, 0.0f));
    CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(1.0f, 2.0f, 0.0f)));
    parent.setPosition(Math::Vec3(5.0f, 0.0f, 0.0f));
    CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(5.0f, 2.0f, 0.0f)));
    child->setGlobalPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    CHECK(nearVec(child->getPosition(), Math::Vec3(-5.0f, 0.0f, 0.0f)));
}

void testSetGlobalTransform()
{
    SceneTree tree;
    tree.initialize();

    Node3D* parent = tree.getRoot()->createChild<Node3D>();
    Node3D* child = parent->createChild<Node3D>();

    parent->setPosition(Math::Vec3(5.0f, 0.0f, 0.0f));
    parent->setQuaternion(Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 1.0f, 0.0f), kHalfPi));
    parent->setScale(Math::Vec3(2.0f, 2.0f, 2.0f));

    const Math::Mat4 global =
            Math::Mat4::Translation(Math::Vec3(1.0f, 2.0f, 3.0f)) * Math::Mat4::RotationZ(0.5f);
    child->setGlobalTransform(global);

    CHECK(nearMat(child->getGlobalTransform(), global));
    CHECK(nearVec(child->getPosition(), Math::Vec3(-1.5f, 1.0f, -2.0f)));
    CHECK(nearMat(child->getTransform(), parent->getGlobalTransform().InverseAffine() * global));
    CHECK(nearVec(child->getScale(), Math::Vec3(0.5f, 0.5f, 0.5f)));

    child->setGlobalPosition(Math::Vec3(5.0f, 0.0f, -2.0f));
    CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(5.0f, 0.0f, -2.0f)));
    CHECK(nearVec(child->getPosition(), Math::Vec3(1.0f, 0.0f, 0.0f)));

    parent->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(0.0f, 0.0f, -2.0f)));
}

void testToLocalToGlobal()
{
    SceneTree tree;
    tree.initialize();

    Node3D* node = tree.getRoot()->createChild<Node3D>();
    node->setPosition(Math::Vec3(5.0f, 0.0f, 0.0f));
    node->setQuaternion(Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 1.0f, 0.0f), kHalfPi));
    node->setScale(Math::Vec3(2.0f, 2.0f, 2.0f));

    CHECK(nearVec(node->toGlobal(Math::Vec3(1.0f, 0.0f, 0.0f)), Math::Vec3(5.0f, 0.0f, -2.0f)));
    CHECK(nearVec(node->toLocal(Math::Vec3(5.0f, 0.0f, -2.0f)), Math::Vec3(1.0f, 0.0f, 0.0f)));
    CHECK(nearVec(node->toGlobal(Math::Vec3(0.0f, 0.0f, 0.0f)), node->getGlobalPosition()));

    const Math::Vec3 point(3.0f, -4.0f, 7.0f);
    CHECK(nearVec(node->toLocal(node->toGlobal(point)), point));
    CHECK(nearVec(node->toGlobal(node->toLocal(point)), point));

    Node3D* child = node->createChild<Node3D>();
    child->setPosition(Math::Vec3(0.0f, 1.0f, 0.0f));
    CHECK(nearVec(child->toGlobal(Math::Vec3(0.0f, 0.0f, 0.0f)), Math::Vec3(5.0f, 2.0f, 0.0f)));
    CHECK(nearVec(child->toLocal(Math::Vec3(5.0f, 2.0f, 0.0f)), Math::Vec3(0.0f, 0.0f, 0.0f)));
}

void testLookAt()
{
    {
        SceneTree tree;
        tree.initialize();
        Node3D* node = tree.getRoot()->createChild<Node3D>();

        node->lookAt(Math::Vec3(10.0f, 0.0f, 0.0f));
        const Math::Mat4 global = node->getGlobalTransform();
        CHECK(nearVec(-basisZ(global), Math::Vec3(1.0f, 0.0f, 0.0f)));
        CHECK(nearVec(basisY(global), Math::Vec3(0.0f, 1.0f, 0.0f)));
        CHECK(nearVec(basisX(global), Math::Vec3(0.0f, 0.0f, 1.0f)));
        CHECK(nearVec(node->getGlobalPosition(), Math::Vec3(0.0f, 0.0f, 0.0f)));

        node->lookAt(Math::Vec3(0.0f, 0.0f, -5.0f));
        CHECK(nearMat(node->getGlobalTransform(), Math::Mat4::Identity()));

        node->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
        node->lookAt(Math::Vec3(1.0f, 2.0f, 3.0f));
        CHECK(nearVec(node->getGlobalPosition(), Math::Vec3(1.0f, 2.0f, 3.0f)));
        CHECK(nearMat(node->getGlobalTransform(),
                Math::Mat4::Translation(Math::Vec3(1.0f, 2.0f, 3.0f))));

        node->lookAt(Math::Vec3(1.0f, 12.0f, 3.0f));
        CHECK(nearVec(node->getGlobalPosition(), Math::Vec3(1.0f, 2.0f, 3.0f)));
        CHECK(nearMat(node->getGlobalTransform(),
                Math::Mat4::Translation(Math::Vec3(1.0f, 2.0f, 3.0f))));
    }

    {
        SceneTree tree;
        tree.initialize();
        Node3D* parent = tree.getRoot()->createChild<Node3D>();
        Node3D* child = parent->createChild<Node3D>();
        parent->setPosition(Math::Vec3(5.0f, 5.0f, 5.0f));
        parent->setQuaternion(
                Math::Quaternion::FromAxisAngle(Math::Vec3(1.0f, 1.0f, 0.0f).Normalized(), 0.6f));
        child->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
        child->setScale(Math::Vec3(2.0f, 2.0f, 2.0f));

        const Math::Vec3 before = child->getGlobalPosition();
        const Math::Vec3 target(-3.0f, 4.0f, 9.0f);
        child->lookAt(target);

        const Math::Mat4 global = child->getGlobalTransform();
        CHECK(nearVec(child->getGlobalPosition(), before));
        CHECK(nearVec(-basisZ(global).Normalized(), (target - before).Normalized()));
        CHECK(nearVec(child->getScale(), Math::Vec3(2.0f, 2.0f, 2.0f)));
        CHECK(near(basisZ(global).Length(), 2.0f));
        CHECK(near(basisX(global).Dot(basisY(global)), 0.0f));
        CHECK(basisX(global).Cross(basisY(global)).Dot(basisZ(global)) > 0.0f);

        const Math::Vec3 position(0.0f, 0.0f, 0.0f);
        child->lookAtFromPosition(position, Math::Vec3(0.0f, 0.0f, 4.0f));
        CHECK(nearVec(child->getGlobalPosition(), position));
        CHECK(nearVec(-basisZ(child->getGlobalTransform()).Normalized(),
                Math::Vec3(0.0f, 0.0f, 1.0f)));
    }
}

void testRotations()
{
    Node3D node;
    node.setPosition(Math::Vec3(4.0f, 5.0f, 6.0f));
    node.rotateY(kHalfPi);
    CHECK(nearVec(basisX(node.getTransform()), Math::Vec3(0.0f, 0.0f, -1.0f)));
    CHECK(nearVec(basisZ(node.getTransform()), Math::Vec3(1.0f, 0.0f, 0.0f)));
    CHECK(nearVec(basisY(node.getTransform()), Math::Vec3(0.0f, 1.0f, 0.0f)));
    CHECK(nearVec(node.getPosition(), Math::Vec3(4.0f, 5.0f, 6.0f)));
    CHECK(nearVec(node.getQuaternion() * Math::Vec3(1.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.0f, -1.0f)));

    Node3D z;
    z.rotateZ(kHalfPi);
    CHECK(nearVec(basisX(z.getTransform()), Math::Vec3(0.0f, 1.0f, 0.0f)));

    Node3D x;
    x.rotateX(kHalfPi);
    CHECK(nearVec(basisY(x.getTransform()), Math::Vec3(0.0f, 0.0f, 1.0f)));

    Node3D combined;
    combined.rotateX(kHalfPi);
    combined.rotateY(kHalfPi);
    CHECK(nearVec(combined.getQuaternion() * Math::Vec3(0.0f, 0.0f, 1.0f),
            Math::Vec3(0.0f, -1.0f, 0.0f)));
    CHECK(nearVec(combined.getQuaternion() * Math::Vec3(1.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.0f, -1.0f)));

    Node3D arbitrary;
    arbitrary.rotate(Math::Vec3(0.0f, 0.0f, 2.0f), kHalfPi);
    CHECK(nearVec(basisX(arbitrary.getTransform()), Math::Vec3(0.0f, 1.0f, 0.0f)));
    arbitrary.rotate(Math::Vec3(0.0f, 0.0f, 0.0f), 1.0f);
    CHECK(nearVec(basisX(arbitrary.getTransform()), Math::Vec3(0.0f, 1.0f, 0.0f)));
}

void testTranslations()
{
    Node3D node;
    node.translate(Math::Vec3(1.0f, 2.0f, 3.0f));
    CHECK(nearVec(node.getPosition(), Math::Vec3(1.0f, 2.0f, 3.0f)));

    Node3D rotated;
    rotated.rotateY(kHalfPi);
    rotated.translateObjectLocal(Math::Vec3(0.0f, 0.0f, 1.0f));
    CHECK(nearVec(rotated.getPosition(), Math::Vec3(1.0f, 0.0f, 0.0f)));
    rotated.translateObjectLocal(Math::Vec3(1.0f, 0.0f, 0.0f));
    CHECK(nearVec(rotated.getPosition(), Math::Vec3(1.0f, 0.0f, -1.0f)));

    Node3D scaled;
    scaled.rotateY(kHalfPi);
    scaled.setScale(Math::Vec3(2.0f, 2.0f, 2.0f));
    scaled.translateObjectLocal(Math::Vec3(0.0f, 0.0f, 1.0f));
    CHECK(nearVec(scaled.getPosition(), Math::Vec3(2.0f, 0.0f, 0.0f)));

    Node3D viaTranslate;
    viaTranslate.rotateY(kHalfPi);
    viaTranslate.translate(Math::Vec3(0.0f, 0.0f, 1.0f));
    CHECK(nearVec(viaTranslate.getPosition(), Math::Vec3(1.0f, 0.0f, 0.0f)));
}

void testGlobalOperations()
{
    SceneTree tree;
    tree.initialize();

    Node3D* parent = tree.getRoot()->createChild<Node3D>();
    Node3D* child = parent->createChild<Node3D>();

    parent->setPosition(Math::Vec3(0.0f, 10.0f, 0.0f));
    parent->setQuaternion(Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 0.0f, 1.0f), kHalfPi));
    child->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));

    CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(0.0f, 11.0f, 0.0f)));
    child->globalTranslate(Math::Vec3(1.0f, 0.0f, 0.0f));
    CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(1.0f, 11.0f, 0.0f)));
    CHECK(nearVec(child->getPosition(), Math::Vec3(1.0f, -1.0f, 0.0f)));

    Node3D* free = tree.getRoot()->createChild<Node3D>();
    free->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    free->globalRotate(Math::Vec3(0.0f, 1.0f, 0.0f), kHalfPi);
    CHECK(nearVec(free->getGlobalPosition(), Math::Vec3(1.0f, 2.0f, 3.0f)));
    CHECK(nearVec(basisX(free->getGlobalTransform()), Math::Vec3(0.0f, 0.0f, -1.0f)));
    free->globalRotate(Math::Vec3(1.0f, 0.0f, 0.0f), kHalfPi);
    CHECK(nearVec(basisX(free->getGlobalTransform()), Math::Vec3(0.0f, 1.0f, 0.0f)));
    CHECK(nearVec(basisZ(free->getGlobalTransform()), Math::Vec3(1.0f, 0.0f, 0.0f)));
}

void testTransformNotifications()
{
    SceneTree tree;
    tree.initialize();

    Node holder;
    RecordingNode3D* node = holder.createChild<RecordingNode3D>();
    RecordingNode3D* child = node->createChild<RecordingNode3D>();
    node->setNotifyTransform(true);
    child->setNotifyTransform(true);
    CHECK(node->isTransformNotificationEnabled());
    CHECK(!node->isLocalTransformNotificationEnabled());
    CHECK(node->reparent(tree.getRoot()));

    CHECK(node->count(Node::kNotificationTransformChanged) == 0);
    tree.process(0.1);
    CHECK(node->count(Node::kNotificationTransformChanged) == 1);
    CHECK(child->count(Node::kNotificationTransformChanged) == 1);
    tree.process(0.1);
    CHECK(node->count(Node::kNotificationTransformChanged) == 1);

    node->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));
    node->rotateY(0.5f);
    node->setScale(Math::Vec3(2.0f, 2.0f, 2.0f));
    node->translate(Math::Vec3(0.0f, 1.0f, 0.0f));
    CHECK(node->count(Node::kNotificationTransformChanged) == 1);
    tree.process(0.1);
    CHECK(node->count(Node::kNotificationTransformChanged) == 2);
    CHECK(child->count(Node::kNotificationTransformChanged) == 2);
    tree.process(0.1);
    CHECK(node->count(Node::kNotificationTransformChanged) == 2);

    child->setPosition(Math::Vec3(0.0f, 0.0f, 1.0f));
    tree.physicsProcess(0.1);
    CHECK(node->count(Node::kNotificationTransformChanged) == 2);
    CHECK(child->count(Node::kNotificationTransformChanged) == 3);
    tree.physicsProcess(0.1);
    CHECK(child->count(Node::kNotificationTransformChanged) == 3);

    node->setPosition(Math::Vec3(7.0f, 0.0f, 0.0f));
    node->forceUpdateTransform();
    CHECK(node->count(Node::kNotificationTransformChanged) == 3);
    CHECK(child->count(Node::kNotificationTransformChanged) == 3);
    tree.process(0.1);
    CHECK(node->count(Node::kNotificationTransformChanged) == 3);
    CHECK(child->count(Node::kNotificationTransformChanged) == 4);
    node->forceUpdateTransform();
    CHECK(node->count(Node::kNotificationTransformChanged) == 3);

    node->setNotifyTransform(false);
    node->setPosition(Math::Vec3(8.0f, 0.0f, 0.0f));
    tree.process(0.1);
    CHECK(node->count(Node::kNotificationTransformChanged) == 3);
    CHECK(child->count(Node::kNotificationTransformChanged) == 5);

    CHECK(node->reparent(&holder));
    node->setPosition(Math::Vec3(9.0f, 0.0f, 0.0f));
    tree.process(0.1);
    CHECK(child->count(Node::kNotificationTransformChanged) == 5);
    CHECK(node->reparent(tree.getRoot()));
    tree.process(0.1);
    CHECK(child->count(Node::kNotificationTransformChanged) == 6);
}

void testTransformNotificationsWithRemoval()
{
    SceneTree tree;
    tree.initialize();

    Node holder;
    RecordingNode3D* removed = holder.createChild<RecordingNode3D>();
    RecordingNode3D* deleted = holder.createChild<RecordingNode3D>();
    RecordingNode3D* survivor = holder.createChild<RecordingNode3D>();
    removed->setNotifyTransform(true);
    deleted->setNotifyTransform(true);
    survivor->setNotifyTransform(true);
    removed->reparent(tree.getRoot());
    deleted->reparent(tree.getRoot());
    survivor->reparent(tree.getRoot());
    tree.process(0.1);

    removed->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));
    deleted->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));
    survivor->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));
    removed->reparent(&holder);
    {
        Node doomedHolder;
        deleted->reparent(&doomedHolder);
    }
    tree.process(0.1);
    CHECK(removed->count(Node::kNotificationTransformChanged) == 1);
    CHECK(survivor->count(Node::kNotificationTransformChanged) == 2);

    survivor->setPosition(Math::Vec3(2.0f, 0.0f, 0.0f));
    survivor->queueFree();
    tree.process(0.1);
    CHECK(tree.getNodeCount() == 1);
}

void testLocalTransformNotifications()
{
    SceneTree tree;
    tree.initialize();

    RecordingNode3D* node = tree.getRoot()->createChild<RecordingNode3D>();

    node->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));
    CHECK(node->count(Node3D::kNotificationLocalTransformChanged) == 0);

    node->setNotifyLocalTransform(true);
    CHECK(node->isLocalTransformNotificationEnabled());
    node->setPosition(Math::Vec3(2.0f, 0.0f, 0.0f));
    CHECK(node->count(Node3D::kNotificationLocalTransformChanged) == 1);
    node->setScale(Math::Vec3(2.0f, 2.0f, 2.0f));
    CHECK(node->count(Node3D::kNotificationLocalTransformChanged) == 2);
    node->setRotation(Math::Vec3(0.1f, 0.0f, 0.0f));
    CHECK(node->count(Node3D::kNotificationLocalTransformChanged) == 3);
    node->setTransform(Math::Mat4::Identity());
    CHECK(node->count(Node3D::kNotificationLocalTransformChanged) == 4);
    node->rotateY(0.1f);
    CHECK(node->count(Node3D::kNotificationLocalTransformChanged) == 5);
    tree.process(0.1);
    CHECK(node->count(Node::kNotificationTransformChanged) == 0);
}

void testVisibility()
{
    SceneTree tree;
    tree.initialize();

    Node holder;
    RecordingNode3D* parent = holder.createChild<RecordingNode3D>();
    RecordingNode3D* child = parent->createChild<RecordingNode3D>();

    parent->hide();
    CHECK(!parent->isVisible());
    CHECK(parent->count(Node3D::kNotificationVisibilityChanged) == 0);
    parent->show();
    CHECK(parent->isVisible());

    CHECK(parent->reparent(tree.getRoot()));
    CHECK(parent->isVisible());
    CHECK(child->isVisible());
    CHECK(parent->isVisibleInTree());
    CHECK(child->isVisibleInTree());

    parent->hide();
    CHECK(!parent->isVisible());
    CHECK(child->isVisible());
    CHECK(!parent->isVisibleInTree());
    CHECK(!child->isVisibleInTree());
    CHECK(parent->count(Node3D::kNotificationVisibilityChanged) == 1);
    CHECK(child->count(Node3D::kNotificationVisibilityChanged) == 1);

    parent->hide();
    CHECK(parent->count(Node3D::kNotificationVisibilityChanged) == 1);

    parent->show();
    CHECK(parent->isVisibleInTree());
    CHECK(child->isVisibleInTree());
    CHECK(parent->count(Node3D::kNotificationVisibilityChanged) == 2);
    CHECK(child->count(Node3D::kNotificationVisibilityChanged) == 2);

    child->setVisible(false);
    CHECK(parent->count(Node3D::kNotificationVisibilityChanged) == 2);
    CHECK(child->count(Node3D::kNotificationVisibilityChanged) == 3);
    CHECK(parent->isVisibleInTree());
    CHECK(!child->isVisibleInTree());

    parent->hide();
    CHECK(parent->count(Node3D::kNotificationVisibilityChanged) == 3);
    CHECK(child->count(Node3D::kNotificationVisibilityChanged) == 3);
    parent->show();
    CHECK(child->count(Node3D::kNotificationVisibilityChanged) == 3);
    CHECK(!child->isVisibleInTree());
    child->show();
    CHECK(child->isVisibleInTree());
}

void testWorldNotifications()
{
    SceneTree tree;
    tree.initialize();

    Node holder;
    RecordingNode3D* node = holder.createChild<RecordingNode3D>();
    CHECK(node->events.empty());
    CHECK(node->reparent(tree.getRoot()));
    CHECK(node->events.size() == 2);
    CHECK(node->events[0] == Node::kNotificationEnterTree);
    CHECK(node->events[1] == Node3D::kNotificationEnterWorld);

    node->events.clear();
    CHECK(node->reparent(&holder));
    CHECK(node->events.size() == 2);
    CHECK(node->events[0] == Node::kNotificationExitTree);
    CHECK(node->events[1] == Node3D::kNotificationExitWorld);
}

void testReparentRecomputesGlobal()
{
    SceneTree tree;
    tree.initialize();

    Node3D* a = tree.getRoot()->createChild<Node3D>();
    Node3D* b = tree.getRoot()->createChild<Node3D>();
    a->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));
    b->setPosition(Math::Vec3(0.0f, 1.0f, 0.0f));

    Node3D* child = a->createChild<Node3D>();
    child->setPosition(Math::Vec3(0.0f, 0.0f, 1.0f));
    CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(1.0f, 0.0f, 1.0f)));
    CHECK(child->reparent(b));
    CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(0.0f, 1.0f, 1.0f)));
    b->setPosition(Math::Vec3(0.0f, 5.0f, 0.0f));
    CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(0.0f, 5.0f, 1.0f)));
}

} // namespace

bool nearBasis(const Math::Mat3& basis, const Math::Vec3& x, const Math::Vec3& y,
        const Math::Vec3& z)
{
    return nearVec(basis.col0, x) && nearVec(basis.col1, y) && nearVec(basis.col2, z);
}

void testBasisAndGlobalHelpers()
{
    const Math::Vec3 unitX(1.0f, 0.0f, 0.0f);
    const Math::Vec3 unitY(0.0f, 1.0f, 0.0f);
    const Math::Vec3 unitZ(0.0f, 0.0f, 1.0f);

    SceneTree tree;
    tree.initialize();
    Node3D* parent = tree.getRoot()->createChild<Node3D>();
    parent->setPosition(Math::Vec3(5.0f, 0.0f, 0.0f));
    parent->rotateY(kHalfPi);
    CHECK(nearBasis(parent->getBasis(), Math::Vec3(0.0f, 0.0f, -1.0f), unitY, unitX));
    Node3D* node = parent->createChild<Node3D>();
    CHECK(nearBasis(node->getGlobalBasis(), Math::Vec3(0.0f, 0.0f, -1.0f), unitY, unitX));

    node->setRotationDegrees(Math::Vec3(0.0f, 30.0f, 0.0f));
    CHECK(nearVec(node->getRotation(), Math::Vec3(0.0f, kHalfPi / 3.0f, 0.0f)));
    CHECK(fabsf(node->getRotationDegrees().y - 30.0f) < 1e-2f);
    node->setRotation(Math::Vec3(0.0f, 0.0f, 0.0f));

    node->setScale(Math::Vec3(2.0f, 2.0f, 2.0f));
    node->setGlobalRotation(Math::Vec3(0.0f, 0.0f, 0.0f));
    CHECK(nearBasis(node->getGlobalBasis(), unitX * 2.0f, unitY * 2.0f, unitZ * 2.0f));
    CHECK(nearVec(node->getGlobalRotation(), Math::Vec3(0.0f, 0.0f, 0.0f)));
    CHECK(nearVec(node->getScale(), Math::Vec3(2.0f, 2.0f, 2.0f)));
    CHECK(nearVec(node->getGlobalPosition(), Math::Vec3(5.0f, 0.0f, 0.0f)));
    node->setGlobalRotationDegrees(Math::Vec3(0.0f, 45.0f, 0.0f));
    CHECK(nearVec(node->getGlobalRotation(), Math::Vec3(0.0f, kHalfPi * 0.5f, 0.0f)));
    CHECK(fabsf(node->getGlobalRotationDegrees().y - 45.0f) < 1e-2f);
    CHECK(near(node->getGlobalBasis().col0.Length(), 2.0f));

    node->setGlobalBasis(Math::Mat3(unitX, unitY, unitZ));
    CHECK(nearBasis(node->getGlobalBasis(), unitX, unitY, unitZ));
    CHECK(nearBasis(node->getBasis(), unitZ, unitY, Math::Vec3(-1.0f, 0.0f, 0.0f)));
    CHECK(nearVec(node->getGlobalPosition(), Math::Vec3(5.0f, 0.0f, 0.0f)));

    Node3D local;
    local.setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    local.setBasis(Math::Mat3(unitX, unitY * 2.0f, unitZ * 3.0f));
    CHECK(nearVec(local.getScale(), Math::Vec3(1.0f, 2.0f, 3.0f)));
    CHECK(nearVec(local.getPosition(), Math::Vec3(1.0f, 2.0f, 3.0f)));
    local.orthonormalize();
    CHECK(nearBasis(local.getBasis(), unitX, unitY, unitZ));
    CHECK(nearVec(local.getPosition(), Math::Vec3(1.0f, 2.0f, 3.0f)));

    Node3D inParent;
    inParent.rotateY(kHalfPi);
    inParent.rotate(unitX, kHalfPi);
    CHECK(nearVec(inParent.getBasis().col1, unitZ));
    Node3D inObject;
    inObject.rotateY(kHalfPi);
    inObject.rotateObjectLocal(unitX, kHalfPi);
    CHECK(nearVec(inObject.getBasis().col1, unitX));
    inObject.rotateObjectLocal(Math::Vec3(0.0f, 0.0f, 0.0f), kHalfPi);
    CHECK(nearVec(inObject.getBasis().col1, unitX));

    Node3D scaled;
    scaled.rotateY(kHalfPi);
    scaled.scale(Math::Vec3(2.0f, 1.0f, 1.0f));
    CHECK(nearBasis(scaled.getBasis(), Math::Vec3(0.0f, 0.0f, -1.0f), unitY, unitX * 2.0f));
    scaled.scaleObjectLocal(Math::Vec3(3.0f, 1.0f, 1.0f));
    CHECK(nearBasis(scaled.getBasis(), Math::Vec3(0.0f, 0.0f, -3.0f), unitY, unitX * 2.0f));
    CHECK(nearVec(scaled.getScale(), Math::Vec3(3.0f, 1.0f, 2.0f)));

    Node3D* wide = parent->createChild<Node3D>();
    wide->globalScale(Math::Vec3(3.0f, 1.0f, 1.0f));
    CHECK(nearBasis(wide->getGlobalBasis(), Math::Vec3(0.0f, 0.0f, -1.0f), unitY, unitX * 3.0f));
    CHECK(nearVec(wide->getGlobalPosition(), Math::Vec3(5.0f, 0.0f, 0.0f)));

    wide->setPosition(Math::Vec3(1.0f, 1.0f, 1.0f));
    wide->setIdentity();
    CHECK(nearMat(wide->getTransform(), Math::Mat4::Identity()));
}

int main()
{
    testLocalComposition();
    testSetTransformDecomposition();
    testStoredEuler();
    testParentChildGlobals();
    testPlainNodeBetween();
    testDetachedGlobals();
    testSetGlobalTransform();
    testToLocalToGlobal();
    testLookAt();
    testRotations();
    testTranslations();
    testGlobalOperations();
    testBasisAndGlobalHelpers();
    testTransformNotifications();
    testTransformNotificationsWithRemoval();
    testLocalTransformNotifications();
    testVisibility();
    testWorldNotifications();
    testReparentRecomputesGlobal();

    if (failures) printf("%d checks failed\n", failures);
    return failures ? 1 : 0;
}
