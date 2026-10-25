#include "k2d/Physics2DSerializer.h"

#include "k2d/CharacterBody2D.h"
#include "k2d/GameObject.h"
#include "k2d/RigidBody2D.h"
#include "k2d/Serializer.h"

#include <cstring>

namespace k2d
{

namespace
{
ct::Json writeVec2(const Math::Vec2& v)
{
    ct::Json a = ct::Json::array();
    a.push_back(ct::Json(v.x));
    a.push_back(ct::Json(v.y));
    return a;
}

Math::Vec2 readVec2(const ct::Json& j, const Math::Vec2& fallback)
{
    if (!j.is_array() || j.size() < 2)
        return fallback;
    return Math::Vec2((float)j[0].as_double(fallback.x), (float)j[1].as_double(fallback.y));
}

ct::Json writePoints(const ct::Vector<Math::Vec2>& points)
{
    ct::Json a = ct::Json::array();
    for (size_t i = 0; i < points.size(); ++i)
        a.push_back(writeVec2(points[i]));
    return a;
}

void readPoints(const ct::Json& j, ct::Vector<Math::Vec2>& out)
{
    out.clear();
    if (!j.is_array())
        return;
    for (size_t i = 0; i < j.size(); ++i)
        out.push_back(readVec2(j[i], Math::Vec2(0.0f, 0.0f)));
}

const char* bodyTypeName(kx::BodyType type)
{
    switch (type)
    {
    case kx::BodyType::Static:
        return "static";
    case kx::BodyType::Kinematic:
        return "kinematic";
    default:
        return "dynamic";
    }
}

kx::BodyType bodyTypeFrom(const char* name)
{
    if (std::strcmp(name, "static") == 0)
        return kx::BodyType::Static;
    if (std::strcmp(name, "kinematic") == 0)
        return kx::BodyType::Kinematic;
    return kx::BodyType::Dynamic;
}

const char* shapeKindName(ShapeKind2D kind)
{
    switch (kind)
    {
    case ShapeKind2D::Box:
        return "box";
    case ShapeKind2D::Circle:
        return "circle";
    case ShapeKind2D::Polygon:
        return "polygon";
    case ShapeKind2D::Edge:
        return "edge";
    default:
        return "chain";
    }
}

ShapeKind2D shapeKindFrom(const char* name)
{
    if (std::strcmp(name, "box") == 0)
        return ShapeKind2D::Box;
    if (std::strcmp(name, "circle") == 0)
        return ShapeKind2D::Circle;
    if (std::strcmp(name, "polygon") == 0)
        return ShapeKind2D::Polygon;
    if (std::strcmp(name, "edge") == 0)
        return ShapeKind2D::Edge;
    return ShapeKind2D::Chain;
}

ct::Json writeShapes(const ct::Vector<ShapeDesc2D>& shapes)
{
    ct::Json array = ct::Json::array();
    for (size_t i = 0; i < shapes.size(); ++i)
    {
        const ShapeDesc2D& shape = shapes[i];
        // readShapes drops these (and the engine cannot build them).
        if ((shape.kind == ShapeKind2D::Polygon && shape.points.size() < 3) ||
            (shape.kind == ShapeKind2D::Chain && shape.points.size() < 2))
            continue;
        ct::Json entry = ct::Json::object();
        entry.set("kind", ct::Json(shapeKindName(shape.kind)));
        entry.set("offset", writeVec2(shape.offset));
        entry.set("density", ct::Json((double)shape.density));
        entry.set("sensor", ct::Json(shape.sensor));
        entry.set("category", ct::Json((int64_t)shape.category));
        entry.set("mask", ct::Json((int64_t)shape.mask));
        switch (shape.kind)
        {
        case ShapeKind2D::Box:
            entry.set("size", writeVec2(shape.size));
            break;
        case ShapeKind2D::Circle:
            entry.set("radius", ct::Json((double)shape.radius));
            break;
        case ShapeKind2D::Polygon:
            entry.set("points", writePoints(shape.points));
            break;
        case ShapeKind2D::Edge:
            entry.set("start", writeVec2(shape.start));
            entry.set("end", writeVec2(shape.end));
            break;
        case ShapeKind2D::Chain:
            entry.set("points", writePoints(shape.points));
            entry.set("loop", ct::Json(shape.loop));
            break;
        }
        array.push_back(entry);
    }
    return array;
}

void readShapes(const ct::Json& data, RigidBody2D& body)
{
    body.clearShapes();
    if (!data.is_array())
        return;

    for (size_t i = 0; i < data.size(); ++i)
    {
        const ct::Json& entry = data[i];
        const ShapeKind2D kind = shapeKindFrom(entry["kind"].as_cstr("box"));
        const Math::Vec2 offset = readVec2(entry["offset"], Math::Vec2(0.0f, 0.0f));
        const float density = (float)entry["density"].as_double(1.0);

        int index = -1;
        switch (kind)
        {
        case ShapeKind2D::Box:
            index = body.addBoxShape(readVec2(entry["size"], Math::Vec2(32.0f, 32.0f)), offset, density);
            break;
        case ShapeKind2D::Circle:
            index = body.addCircleShape((float)entry["radius"].as_double(16.0), offset, density);
            break;
        case ShapeKind2D::Polygon:
        {
            ct::Vector<Math::Vec2> points;
            readPoints(entry["points"], points);
            if (points.size() >= 3)
                index = body.addPolygonShape(points.data(), (int)points.size(), offset, density);
            break;
        }
        case ShapeKind2D::Edge:
            index = body.addEdgeShape(readVec2(entry["start"], Math::Vec2(-32.0f, 0.0f)),
                                      readVec2(entry["end"], Math::Vec2(32.0f, 0.0f)), offset);
            break;
        case ShapeKind2D::Chain:
        {
            ct::Vector<Math::Vec2> points;
            readPoints(entry["points"], points);
            if (points.size() >= 2)
                index = body.addChainShape(points.data(), (int)points.size(), entry["loop"].as_bool(false), offset);
            break;
        }
        }

        if (index >= 0)
        {
            // addEdgeShape/addChainShape take no density; restore the saved one.
            if (kind == ShapeKind2D::Edge || kind == ShapeKind2D::Chain)
                body.setShapeDensity(index, density);
            body.setShapeSensor(index, entry["sensor"].as_bool(false));
            body.setShapeFilter(index, (uint16_t)entry["category"].as_int(1), (uint16_t)entry["mask"].as_int(0xFFFF));
        }
    }
}

Component* createRigidBody(GameObject& owner)
{
    return owner.addComponent<RigidBody2D>();
}

void writeRigidBody(const Component& component, ct::Json& data, Assets*)
{
    const RigidBody2D& body = static_cast<const RigidBody2D&>(component);
    data.set("bodyType", ct::Json(bodyTypeName(body.bodyType())));
    data.set("density", ct::Json((double)body.density()));
    data.set("friction", ct::Json((double)body.friction()));
    data.set("restitution", ct::Json((double)body.restitution()));
    data.set("linearDamping", ct::Json((double)body.linearDamping()));
    data.set("angularDamping", ct::Json((double)body.angularDamping()));
    data.set("gravityScale", ct::Json((double)body.gravityScale()));
    data.set("fixedRotation", ct::Json(body.fixedRotation()));
    data.set("bullet", ct::Json(body.bullet()));
    data.set("shapes", writeShapes(body.shapes()));
}

void readRigidBody(Component& component, const ct::Json& data, Assets*)
{
    RigidBody2D& body = static_cast<RigidBody2D&>(component);
    body.setBodyType(bodyTypeFrom(data["bodyType"].as_cstr("dynamic")));
    body.setDensity((float)data["density"].as_double(1.0));
    body.setFriction((float)data["friction"].as_double(0.3));
    body.setRestitution((float)data["restitution"].as_double(0.0));
    body.setLinearDamping((float)data["linearDamping"].as_double(0.0));
    body.setAngularDamping((float)data["angularDamping"].as_double(0.0));
    body.setGravityScale((float)data["gravityScale"].as_double(1.0));
    body.setFixedRotation(data["fixedRotation"].as_bool(false));
    body.setBullet(data["bullet"].as_bool(false));
    readShapes(data["shapes"], body);
}

bool isRigidBody(const Component& component)
{
    return dynamic_cast<const RigidBody2D*>(&component) != nullptr;
}

Component* createCharacterBody(GameObject& owner)
{
    return owner.addComponent<CharacterBody2D>();
}

void writeCharacterBody(const Component& component, ct::Json& data, Assets*)
{
    const CharacterBody2D& body = static_cast<const CharacterBody2D&>(component);
    data.set("velocity", writeVec2(body.velocity()));
    data.set("safeMargin", ct::Json((double)body.safeMargin()));
    data.set("maxSlides", ct::Json((int64_t)body.maxSlides()));
    data.set("motionMode",
             ct::Json(body.motionMode() == CharacterBody2D::MotionMode::Grounded ? "grounded" : "floating"));
    data.set("upDirection", writeVec2(body.upDirection()));
    data.set("floorMaxAngle", ct::Json((double)body.floorMaxAngleDegrees()));
}

void readCharacterBody(Component& component, const ct::Json& data, Assets*)
{
    CharacterBody2D& body = static_cast<CharacterBody2D&>(component);
    body.setVelocity(readVec2(data["velocity"], Math::Vec2(0.0f, 0.0f)));
    body.setSafeMargin((float)data["safeMargin"].as_double(kx::kLinearSlop));
    body.setMaxSlides((int)data["maxSlides"].as_int(4));
    body.setMotionMode(std::strcmp(data["motionMode"].as_cstr("floating"), "grounded") == 0
                           ? CharacterBody2D::MotionMode::Grounded
                           : CharacterBody2D::MotionMode::Floating);
    body.setUpDirection(readVec2(data["upDirection"], Math::Vec2(0.0f, -1.0f)));
    body.setFloorMaxAngleDegrees((float)data["floorMaxAngle"].as_double(45.0));
}

bool isCharacterBody(const Component& component)
{
    return dynamic_cast<const CharacterBody2D*>(&component) != nullptr;
}

} // namespace

void RegisterPhysics2DSerializers()
{
    Serializer::RegisterType(ComponentType::RigidBody, "RigidBody2D", &createRigidBody, &writeRigidBody, &readRigidBody,
                             &isRigidBody);
    Serializer::RegisterType(ComponentType::CharacterBody, "CharacterBody2D", &createCharacterBody, &writeCharacterBody,
                             &readCharacterBody, &isCharacterBody);
}

} // namespace k2d
