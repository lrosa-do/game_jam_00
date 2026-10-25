#include "ZenInternal.h"

#include "mathc.h"

namespace prisma3d
{

namespace
{

struct Float4
{
    float v[4];
};

void fillFloat4(void* buffer, int argc, zen::Value* args, float fallbackW)
{
    Float4* data = static_cast<Float4*>(buffer);
    data->v[0] = 0.0f;
    data->v[1] = 0.0f;
    data->v[2] = 0.0f;
    data->v[3] = fallbackW;
    for (int i = 0; i < argc && i < 4; ++i)
        data->v[i] = static_cast<float>(zen::to_number(args[i]));
}

void vec4Ctor(zen::VM*, void* buffer, int argc, zen::Value* args)
{
    fillFloat4(buffer, argc, args, 0.0f);
}

void quatCtor(zen::VM*, void* buffer, int argc, zen::Value* args)
{
    fillFloat4(buffer, argc, args, 1.0f);
}

void colorCtor(zen::VM*, void* buffer, int argc, zen::Value* args)
{
    fillFloat4(buffer, argc, args, 1.0f);
}

zen::Value makeFloat4(zen::VM& vm, zen::NativeStructDef* def, float x, float y, float z, float w)
{
    zen::Value args[4] = { zen::val_float(x), zen::val_float(y), zen::val_float(z),
        zen::val_float(w) };
    return vm.make_native_struct(def, args, 4);
}

const Float4* readFloat4(zen::Value value, const zen::NativeStructDef* def)
{
    if (!zen::is_native_struct(value)) return nullptr;
    const zen::ObjNativeStruct* object = zen::as_native_struct(value);
    if (object->def != def) return nullptr;
    return static_cast<const Float4*>(object->data);
}

const char* typeLabel(zen::Value value)
{
    if (zen::is_nil(value)) return "None";
    if (zen::is_bool(value)) return "bool";
    if (zen::is_int(value)) return "int";
    if (zen::is_float(value)) return "float";
    if (zen::is_string(value)) return "string";
    if (zen::is_vec2(value)) return "Vec2";
    if (zen::is_vec3(value)) return "Vec3";
    if (zen::is_native_struct(value))
    {
        const zen::ObjNativeStruct* object = zen::as_native_struct(value);
        return object->def && object->def->name ? object->def->name->chars : "struct";
    }
    if (zen::is_instance(value))
    {
        const zen::ObjClass* klass = zen::as_instance(value)->klass;
        return klass && klass->name ? klass->name->chars : "instance";
    }
    return "object";
}

bool isNumber(zen::Value value)
{
    return zen::is_int(value) || zen::is_float(value) || zen::is_bool(value);
}

} // namespace

void ZenScripting::Impl::registerValueTypes()
{
    vec4Def = vm.register_native_struct("Vec4", sizeof(Float4), &vec4Ctor)
                      .f32("x", 0)
                      .f32("y", 4)
                      .f32("z", 8)
                      .f32("w", 12)
                      .end();
    quatDef = vm.register_native_struct("Quat", sizeof(Float4), &quatCtor)
                      .f32("x", 0)
                      .f32("y", 4)
                      .f32("z", 8)
                      .f32("w", 12)
                      .end();
    colorDef = vm.register_native_struct("Color", sizeof(Float4), &colorCtor)
                       .f32("r", 0)
                       .f32("g", 4)
                       .f32("b", 8)
                       .f32("a", 12)
                       .end();
}

zen::Value ZenScripting::Impl::toZen(const PropertyValue& value)
{
    switch (value.type())
    {
        case PropertyType::Nil:
            return zen::val_nil();
        case PropertyType::Bool:
            return zen::val_bool(value.asBool());
        case PropertyType::Int:
            return zen::val_int(value.asInt());
        case PropertyType::Float:
            return zen::val_float(value.asFloat());
        case PropertyType::Vec2:
        {
            const Math::Vec2 v = value.asVec2();
            return zen::val_vec2(v.x, v.y);
        }
        case PropertyType::Vec3:
        {
            const Math::Vec3 v = value.asVec3();
            return zen::val_vec3(v.x, v.y, v.z);
        }
        case PropertyType::Vec4:
        {
            const Math::Vec4 v = value.asVec4();
            return makeFloat4(vm, vec4Def, v.x, v.y, v.z, v.w);
        }
        case PropertyType::Quat:
        {
            const Math::Quaternion v = value.asQuat();
            return makeFloat4(vm, quatDef, v.x, v.y, v.z, v.w);
        }
        case PropertyType::Color:
        {
            const Math::Vec4 v = value.asColor();
            return makeFloat4(vm, colorDef, v.x, v.y, v.z, v.w);
        }
        case PropertyType::String:
        case PropertyType::NodePath:
        case PropertyType::ResourcePath:
        {
            const ct::String text = value.asString();
            return zen::val_obj(reinterpret_cast<zen::Obj*>(
                    vm.make_string(text.c_str(), static_cast<int>(text.size()))));
        }
    }
    return zen::val_nil();
}

bool ZenScripting::Impl::fromZen(zen::Value value, PropertyType type, PropertyValue& out,
        ct::String& error)
{
    auto fail = [&](const char* expected)
    {
        error = ct::String("expected ") + expected + ", got " + typeLabel(value);
        return false;
    };
    switch (type)
    {
        case PropertyType::Nil:
            out = PropertyValue();
            return true;
        case PropertyType::Bool:
            out = PropertyValue::fromBool(zen::is_truthy_full(value));
            return true;
        case PropertyType::Int:
            if (!isNumber(value)) return fail("an int");
            out = PropertyValue::fromInt(zen::to_integer(value));
            return true;
        case PropertyType::Float:
            if (!isNumber(value)) return fail("a number");
            out = PropertyValue::fromFloat(zen::to_number(value));
            return true;
        case PropertyType::Vec2:
            if (!zen::is_vec2(value)) return fail("a Vec2");
            out = PropertyValue::fromVec2(Math::Vec2(zen::vec_x(value), zen::vec_y(value)));
            return true;
        case PropertyType::Vec3:
            if (!zen::is_vec3(value)) return fail("a Vec3");
            out = PropertyValue::fromVec3(
                    Math::Vec3(zen::vec_x(value), zen::vec_y(value), zen::vec_z(value)));
            return true;
        case PropertyType::Vec4:
        {
            const Float4* data = readFloat4(value, vec4Def);
            if (!data) return fail("a Vec4");
            out = PropertyValue::fromVec4(
                    Math::Vec4(data->v[0], data->v[1], data->v[2], data->v[3]));
            return true;
        }
        case PropertyType::Quat:
        {
            const Float4* data = readFloat4(value, quatDef);
            if (!data) return fail("a Quat");
            out = PropertyValue::fromQuat(
                    Math::Quaternion(data->v[0], data->v[1], data->v[2], data->v[3]));
            return true;
        }
        case PropertyType::Color:
        {
            if (zen::is_vec3(value))
            {
                out = PropertyValue::fromColor(
                        Math::Vec4(zen::vec_x(value), zen::vec_y(value), zen::vec_z(value), 1.0f));
                return true;
            }
            const Float4* data = readFloat4(value, colorDef);
            if (!data) data = readFloat4(value, vec4Def);
            if (!data) return fail("a Color");
            out = PropertyValue::fromColor(
                    Math::Vec4(data->v[0], data->v[1], data->v[2], data->v[3]));
            return true;
        }
        case PropertyType::String:
        case PropertyType::NodePath:
        case PropertyType::ResourcePath:
        {
            if (!zen::is_string(value)) return fail("a string");
            const ct::String text(zen::safe_string_chars(value),
                    static_cast<std::size_t>(zen::safe_string_len(value)));
            if (type == PropertyType::NodePath) out = PropertyValue::fromNodePath(text);
            else if (type == PropertyType::ResourcePath)
                out = PropertyValue::fromResourcePath(text);
            else
                out = PropertyValue::fromString(text);
            return true;
        }
    }
    error = "unsupported property type";
    return false;
}

} // namespace prisma3d
