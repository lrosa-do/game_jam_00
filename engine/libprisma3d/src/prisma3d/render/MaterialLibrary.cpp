#include "prisma3d/render/MaterialLibrary.h"

#include "prisma3d/core/FileSystem.h"

#include "platform.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace prisma3d
{

namespace
{

template<class T>
struct NamedValue
{
    const char* name;
    T value;
};

template<class T, std::size_t N>
const char* nameOf(const NamedValue<T> (&table)[N], T value)
{
    for (const NamedValue<T>& entry: table)
        if (entry.value == value) return entry.name;
    return table[0].name;
}

template<class T, std::size_t N>
bool valueOf(const NamedValue<T> (&table)[N], const char* name, T& out)
{
    for (const NamedValue<T>& entry: table)
    {
        if (strcmp(entry.name, name) == 0)
        {
            out = entry.value;
            return true;
        }
    }
    return false;
}

const NamedValue<MaterialTransparency> kTransparencyNames[] = {
    { "None", MaterialTransparency::Opaque },
    { "Cutout", MaterialTransparency::AlphaScissor },
    { "Blend", MaterialTransparency::AlphaBlend },
};

const NamedValue<MaterialAlphaAntialiasing> kAlphaAntialiasingNames[] = {
    { "Off", MaterialAlphaAntialiasing::Off },
    { "AlphaToCoverage", MaterialAlphaAntialiasing::AlphaToCoverage },
};

const NamedValue<MaterialBlend> kBlendNames[] = {
    { "Alpha", MaterialBlend::Mix },
    { "Add", MaterialBlend::Add },
    { "Multiply", MaterialBlend::Mul },
    { "Subtract", MaterialBlend::Sub },
    { "Screen", MaterialBlend::Screen },
};

const NamedValue<MaterialBlend> kDetailBlendNames[] = {
    { "Alpha", MaterialBlend::Mix },
    { "Add", MaterialBlend::Add },
    { "Multiply", MaterialBlend::Mul },
    { "Subtract", MaterialBlend::Sub },
};

const NamedValue<MaterialDetailUv> kDetailUvNames[] = {
    { "Uv1", MaterialDetailUv::Uv1 },
    { "Uv2", MaterialDetailUv::Uv2 },
};

const NamedValue<MaterialCull> kCullNames[] = {
    { "Back", MaterialCull::Back },
    { "Front", MaterialCull::Front },
    { "None", MaterialCull::Disabled },
};

const NamedValue<MaterialEmissionOperator> kEmissionOperatorNames[] = {
    { "Add", MaterialEmissionOperator::Add },
    { "Multiply", MaterialEmissionOperator::Multiply },
};

const NamedValue<MaterialTextureChannel> kChannelNames[] = {
    { "Red", MaterialTextureChannel::Red },
    { "Green", MaterialTextureChannel::Green },
    { "Blue", MaterialTextureChannel::Blue },
    { "Alpha", MaterialTextureChannel::Alpha },
    { "Grayscale", MaterialTextureChannel::Grayscale },
};

const NamedValue<MaterialBillboard> kBillboardNames[] = {
    { "Disabled", MaterialBillboard::Disabled },
    { "FixedY", MaterialBillboard::FixedY },
};

const char* const kTextureKeys[kMaterialTextureCount] = { "albedo", "metallic_map", "roughness_map",
    "emission", "normal", "occlusion", "clearcoat_map", "orm", "detail_albedo", "detail_normal",
    "detail_mask", "refraction_map" };

enum class TokType
{
    End,
    Ident,
    Str,
    Num,
    LBrace,
    RBrace,
    LParen,
    RParen,
    Comma,
    Equals,
    LBracket,
    RBracket
};

struct Tok
{
    TokType type = TokType::End;
    ct::String text;
    double num = 0.0;
    int line = 1;
};

void appendInteger(ct::String& out, long long value)
{
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%lld", value);
    out += buffer;
}

struct Lexer
{
    const char* p;
    const char* end;
    int line = 1;
    ct::String errorMsg;

    explicit Lexer(const ct::String& text)
            : p(text.c_str()),
              end(text.c_str() + text.size())
    {
    }

    void skipTrivia()
    {
        for (;;)
        {
            while (p < end && (*p == ' ' || *p == '\t' || *p == '\r')) ++p;
            if (p < end && *p == '\n')
            {
                ++p;
                ++line;
                continue;
            }
            if (p + 1 < end && p[0] == '/' && p[1] == '/')
            {
                while (p < end && *p != '\n') ++p;
                continue;
            }
            if (p < end && *p == '#')
            {
                while (p < end && *p != '\n') ++p;
                continue;
            }
            break;
        }
    }

    static bool identStart(char c)
    {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
    }

    static bool identChar(char c)
    {
        return identStart(c) || (c >= '0' && c <= '9');
    }

    static bool digit(char c)
    {
        return c >= '0' && c <= '9';
    }

    bool next(Tok& out)
    {
        skipTrivia();
        out.line = line;
        out.text = ct::String();
        if (p >= end)
        {
            out.type = TokType::End;
            return true;
        }
        const char c = *p;
        switch (c)
        {
            case '{':
                ++p;
                out.type = TokType::LBrace;
                return true;
            case '}':
                ++p;
                out.type = TokType::RBrace;
                return true;
            case '(':
                ++p;
                out.type = TokType::LParen;
                return true;
            case ')':
                ++p;
                out.type = TokType::RParen;
                return true;
            case ',':
                ++p;
                out.type = TokType::Comma;
                return true;
            case '=':
                ++p;
                out.type = TokType::Equals;
                return true;
            case '[':
                ++p;
                out.type = TokType::LBracket;
                return true;
            case ']':
                ++p;
                out.type = TokType::RBracket;
                return true;
            default:
                break;
        }
        if (c == '"')
        {
            ++p;
            const char* start = p;
            while (p < end && *p != '"' && *p != '\n') ++p;
            if (p >= end || *p != '"')
            {
                errorMsg = "unterminated string";
                return false;
            }
            out.type = TokType::Str;
            out.text = ct::String(start, static_cast<std::size_t>(p - start));
            ++p;
            return true;
        }
        if (digit(c) ||
                ((c == '-' || c == '+' || c == '.') && p + 1 < end && (digit(p[1]) || p[1] == '.')))
        {
            char* parsedEnd = nullptr;
            out.num = strtod(p, &parsedEnd);
            if (parsedEnd == p || parsedEnd > end)
            {
                errorMsg = "malformed number";
                return false;
            }
            if (parsedEnd < end && (identChar(*parsedEnd) || *parsedEnd == '.'))
            {
                errorMsg = "malformed number";
                return false;
            }
            p = parsedEnd;
            out.type = TokType::Num;
            return true;
        }
        if (identStart(c))
        {
            const char* start = p;
            while (p < end && identChar(*p)) ++p;
            out.type = TokType::Ident;
            out.text = ct::String(start, static_cast<std::size_t>(p - start));
            return true;
        }
        errorMsg = "unexpected character '";
        errorMsg += ct::String(&c, 1);
        errorMsg += "'";
        if (c == ':')
            errorMsg += " (a material does not inherit from another; every key it needs is "
                        "written in its own block)";
        return false;
    }
};

struct Value
{
    enum Kind
    {
        Bool,
        Number,
        Vec,
        String,
        Ident,
        List
    };
    Kind kind = Number;
    float v[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    int count = 0;
    bool b = false;
    ct::String text;
    ct::Vector<ct::String> items;
};

class Parser
{
public:
    Parser(const ct::String& text, ct::Vector<ct::String>& warnings)
            : mLexer(text),
              mWarnings(warnings)
    {
    }

    bool parseFile(ct::Vector<MaterialDef>& outMaterials)
    {
        advance();
        while (mOk && mCurrent.type != TokType::End)
        {
            if (!expectIdent("material")) return false;
            if (!expect(TokType::Str, "material name string")) return false;
            if (mPrevious.text.empty())
            {
                fail(ct::String("material name must not be empty"));
                return false;
            }
            MaterialDef material;
            material.info.name = mPrevious.text;
            if (!expect(TokType::LBrace, "'{'")) return false;
            if (!parseBody(material)) return false;
            if (!expect(TokType::RBrace, "'}'")) return false;
            outMaterials.push_back(static_cast<MaterialDef&&>(material));
        }
        return mOk;
    }

    const ct::String& error() const
    {
        return mError;
    }

private:
    Lexer mLexer;
    Tok mCurrent;
    Tok mPrevious;
    bool mOk = true;
    ct::String mError;
    ct::Vector<ct::String>& mWarnings;
    int mRateLine[kMaterialTextureCount] = {};

    void advance()
    {
        mPrevious = mCurrent;
        if (!mLexer.next(mCurrent))
        {
            mCurrent.type = TokType::End;
            failAt(mLexer.line, mLexer.errorMsg);
        }
    }

    void failAt(int line, const ct::String& message)
    {
        if (!mOk) return;
        mOk = false;
        mError = ct::String("line ");
        appendInteger(mError, line);
        mError += ": ";
        mError += message;
    }

    void fail(const ct::String& message)
    {
        failAt(mCurrent.line, message);
    }

    void warn(int line, const ct::String& message)
    {
        ct::String text("line ");
        appendInteger(text, line);
        text += ": ";
        text += message;
        mWarnings.push_back(text);
    }

    static const char* describe(const Tok& tok)
    {
        switch (tok.type)
        {
            case TokType::End:
                return "end of file";
            case TokType::Ident:
                return "identifier";
            case TokType::Str:
                return "string";
            case TokType::Num:
                return "number";
            case TokType::LBrace:
                return "'{'";
            case TokType::RBrace:
                return "'}'";
            case TokType::LParen:
                return "'('";
            case TokType::RParen:
                return "')'";
            case TokType::Comma:
                return "','";
            case TokType::Equals:
                return "'='";
            case TokType::LBracket:
                return "'['";
            case TokType::RBracket:
                return "']'";
        }
        return "token";
    }

    bool expect(TokType type, const char* what)
    {
        if (!mOk) return false;
        if (mCurrent.type != type)
        {
            ct::String message("expected ");
            message += what;
            message += ", got ";
            message += describe(mCurrent);
            if (mCurrent.type == TokType::Ident || mCurrent.type == TokType::Str)
            {
                message += " '";
                message += mCurrent.text;
                message += "'";
            }
            fail(message);
            return false;
        }
        advance();
        return mOk;
    }

    bool expectIdent(const char* name)
    {
        if (!mOk) return false;
        if (mCurrent.type != TokType::Ident || mCurrent.text != name)
        {
            ct::String message("expected '");
            message += name;
            message += "', got ";
            message += describe(mCurrent);
            if (mCurrent.type == TokType::Ident)
            {
                message += " '";
                message += mCurrent.text;
                message += "'";
            }
            fail(message);
            return false;
        }
        advance();
        return mOk;
    }

    bool parseValue(Value& out)
    {
        if (!mOk) return false;
        if (mCurrent.type == TokType::Ident)
        {
            if (mCurrent.text == "true" || mCurrent.text == "false")
            {
                out.kind = Value::Bool;
                out.b = mCurrent.text == "true";
            }
            else
            {
                out.kind = Value::Ident;
                out.text = mCurrent.text;
            }
            advance();
            return mOk;
        }
        if (mCurrent.type == TokType::Str)
        {
            out.kind = Value::String;
            out.text = mCurrent.text;
            advance();
            return mOk;
        }
        if (mCurrent.type == TokType::Num)
        {
            out.kind = Value::Number;
            out.v[0] = static_cast<float>(mCurrent.num);
            out.count = 1;
            advance();
            return mOk;
        }
        if (mCurrent.type == TokType::LParen)
        {
            advance();
            out.kind = Value::Vec;
            out.count = 0;
            for (;;)
            {
                if (!expect(TokType::Num, "number")) return false;
                if (out.count == 4)
                {
                    fail(ct::String("a vector has at most four components"));
                    return false;
                }
                out.v[out.count++] = static_cast<float>(mPrevious.num);
                if (mCurrent.type != TokType::Comma) break;
                advance();
            }
            if (out.count < 2)
            {
                fail(ct::String("a vector has at least two components"));
                return false;
            }
            return expect(TokType::RParen, "')'");
        }
        if (mCurrent.type == TokType::LBracket)
        {
            advance();
            out.kind = Value::List;
            while (mCurrent.type != TokType::RBracket)
            {
                if (!expect(TokType::Str, "a quoted string inside [ ]")) return false;
                out.items.push_back(mPrevious.text);
                if (mCurrent.type != TokType::Comma) break;
                advance();
            }
            return expect(TokType::RBracket, "']'");
        }
        ct::String message("expected a value, got ");
        message += describe(mCurrent);
        fail(message);
        return false;
    }

    bool parseBody(MaterialDef& material)
    {
        for (std::size_t slot = 0; slot < kMaterialTextureCount; ++slot) mRateLine[slot] = 0;
        while (mOk && mCurrent.type != TokType::RBrace)
        {
            if (!expect(TokType::Ident, "key")) return false;
            const ct::String key = mPrevious.text;
            const int line = mPrevious.line;
            if (mCurrent.type == TokType::LBrace)
            {
                if (!skipBlock(key, line)) return false;
                continue;
            }
            if (!expect(TokType::Equals, "'='")) return false;
            Value value;
            if (!parseValue(value)) return false;
            if (!assign(material, key, value, line)) return false;
        }
        for (std::size_t slot = 0; mOk && slot < kMaterialTextureCount; ++slot)
        {
            if (mRateLine[slot] == 0 || !material.sequences[slot].empty()) continue;
            ct::String message("'");
            message += kTextureKeys[slot];
            message += "_fps' ignored: '";
            message += kTextureKeys[slot];
            message += "' is not a sequence";
            warn(mRateLine[slot], message);
        }
        return mOk;
    }

    bool skipBlock(const ct::String& key, int line)
    {
        ct::String message("unknown block '");
        message += key;
        message += "' ignored";
        warn(line, message);
        if (!expect(TokType::LBrace, "'{'")) return false;
        int depth = 1;
        while (depth > 0)
        {
            if (mCurrent.type == TokType::End)
            {
                ct::String unclosed("block '");
                unclosed += key;
                unclosed += "' is never closed";
                failAt(line, unclosed);
                return false;
            }
            if (mCurrent.type == TokType::LBrace) ++depth;
            else if (mCurrent.type == TokType::RBrace)
                --depth;
            advance();
            if (!mOk) return false;
        }
        return true;
    }

    bool typeError(const ct::String& key, int line, const char* expected)
    {
        ct::String message("'");
        message += key;
        message += "' expects ";
        message += expected;
        failAt(line, message);
        return false;
    }

    bool number(const Value& value, const ct::String& key, int line, float& out)
    {
        if (value.kind != Value::Number) return typeError(key, line, "a number");
        out = value.v[0];
        return true;
    }

    bool boolean(const Value& value, const ct::String& key, int line, bool& out)
    {
        if (value.kind != Value::Bool) return typeError(key, line, "true or false");
        out = value.b;
        return true;
    }

    bool vector(const Value& value, const ct::String& key, int line, int components, float* out)
    {
        if (value.kind != Value::Vec || value.count != components)
            return typeError(key, line,
                    components == 2 ? "(x, y)" : (components == 3 ? "(x, y, z)" : "(r, g, b, a)"));
        for (int i = 0; i < components; ++i) out[i] = value.v[i];
        return true;
    }

    bool colorValue(const Value& value, const ct::String& key, int line, Math::Vec4& out)
    {
        if (value.kind != Value::Vec || (value.count != 3 && value.count != 4))
            return typeError(key, line, "(r, g, b) or (r, g, b, a)");
        out = Math::Vec4(value.v[0], value.v[1], value.v[2], value.count == 4 ? value.v[3] : 1.0f);
        return true;
    }

    bool vec2(const Value& value, const ct::String& key, int line, Math::Vec2& out)
    {
        float components[2] = {};
        if (!vector(value, key, line, 2, components)) return false;
        out = Math::Vec2(components[0], components[1]);
        return true;
    }

    bool vec3(const Value& value, const ct::String& key, int line, Math::Vec3& out)
    {
        float components[3] = {};
        if (!vector(value, key, line, 3, components)) return false;
        out = Math::Vec3(components[0], components[1], components[2]);
        return true;
    }

    bool texturePath(const Value& value, const ct::String& key, int line, MaterialDef& m,
            std::size_t slot)
    {
        if (value.kind == Value::String)
        {
            m.paths[slot] = value.text;
            m.sequences[slot].clear();
            return true;
        }
        if (value.kind != Value::List)
            return typeError(key, line, "a quoted path or a list of quoted paths");
        if (value.items.empty())
        {
            failAt(line, ct::String("'") + key + "': a sequence needs at least one file");
            return false;
        }
        if (value.items.size() > kMaxTextureSequence)
        {
            ct::String message("'");
            message += key;
            message += "': a sequence has at most ";
            appendInteger(message, static_cast<long long>(kMaxTextureSequence));
            message += " files";
            failAt(line, message);
            return false;
        }
        m.sequences[slot] = value.items;
        m.paths[slot] = ct::String();
        return true;
    }

    bool sequenceRate(const Value& value, const ct::String& key, int line, MaterialDef& m,
            std::size_t slot)
    {
        if (value.kind != Value::Number || !(value.v[0] > 0.0f))
            return typeError(key, line, "a number greater than 0");
        m.sequenceFps[slot] = value.v[0];
        mRateLine[slot] = line;
        return true;
    }

    static bool isRateKey(const ct::String& key, const char* slotKey)
    {
        const std::size_t length = strlen(slotKey);
        return key.size() == length + 4 && strncmp(key.c_str(), slotKey, length) == 0 &&
               strcmp(key.c_str() + length, "_fps") == 0;
    }

    template<class T, std::size_t N>
    bool enumeration(const Value& value, const ct::String& key, int line,
            const NamedValue<T> (&table)[N], T& out)
    {
        if (value.kind != Value::Ident) return typeError(key, line, "a name without quotes");
        if (!valueOf(table, value.text.c_str(), out))
        {
            ct::String message("'");
            message += key;
            message += "': unknown value '";
            message += value.text;
            message += "'";
            failAt(line, message);
            return false;
        }
        return true;
    }

    bool assign(MaterialDef& m, const ct::String& key, const Value& value, int line)
    {
        MaterialInfo& info = m.info;

        for (std::size_t slot = 0; slot < kMaterialTextureCount; ++slot)
        {
            if (key == kTextureKeys[slot]) return texturePath(value, key, line, m, slot);
            if (isRateKey(key, kTextureKeys[slot])) return sequenceRate(value, key, line, m, slot);
        }

        if (key == "transparency")
            return enumeration(value, key, line, kTransparencyNames, info.transparency);
        if (key == "alpha_cutoff") return number(value, key, line, info.alphaScissorThreshold);
        if (key == "alpha_antialiasing")
            return enumeration(value, key, line, kAlphaAntialiasingNames, info.alphaAntialiasing);
        if (key == "alpha_antialiasing_edge")
            return number(value, key, line, info.alphaAntialiasingEdge);
        if (key == "blend") return enumeration(value, key, line, kBlendNames, info.blend);
        if (key == "cull") return enumeration(value, key, line, kCullNames, info.cull);
        if (key == "depth_test") return boolean(value, key, line, info.depthTest);
        if (key == "lit")
        {
            bool lit = true;
            if (!boolean(value, key, line, lit)) return false;
            info.shading = lit ? MaterialShading::Lit : MaterialShading::Unshaded;
            return true;
        }

        if (key == "color") return colorValue(value, key, line, info.albedo);
        if (key == "metallic") return number(value, key, line, info.metallic);
        if (key == "specular") return number(value, key, line, info.specular);
        if (key == "roughness") return number(value, key, line, info.roughness);
        if (key == "metallic_channel")
            return enumeration(value, key, line, kChannelNames, info.metallicChannel);
        if (key == "roughness_channel")
            return enumeration(value, key, line, kChannelNames, info.roughnessChannel);
        if (key == "occlusion_channel")
            return enumeration(value, key, line, kChannelNames, info.aoChannel);
        if (key == "refraction") return number(value, key, line, info.refraction);
        if (key == "refraction_channel")
            return enumeration(value, key, line, kChannelNames, info.refractionChannel);
        if (key == "ao_light_affect") return number(value, key, line, info.aoLightAffect);
        if (key == "emission_color") return vec3(value, key, line, info.emission);
        if (key == "emission_energy") return number(value, key, line, info.emissionEnergy);
        if (key == "emission_operator")
            return enumeration(value, key, line, kEmissionOperatorNames, info.emissionOperator);
        if (key == "normal_scale") return number(value, key, line, info.normalScale);
        if (key == "clearcoat") return number(value, key, line, info.clearcoat);
        if (key == "clearcoat_roughness") return number(value, key, line, info.clearcoatRoughness);

        if (key == "uv1_scale") return vec3(value, key, line, info.uv1Scale);
        if (key == "uv1_offset") return vec3(value, key, line, info.uv1Offset);
        if (key == "uv2_scale") return vec3(value, key, line, info.uv2Scale);
        if (key == "uv2_offset") return vec3(value, key, line, info.uv2Offset);

        if (key == "detail") return boolean(value, key, line, info.detail);
        if (key == "detail_blend")
            return enumeration(value, key, line, kDetailBlendNames, info.detailBlend);
        if (key == "detail_uv") return enumeration(value, key, line, kDetailUvNames, info.detailUv);

        if (key == "billboard")
            return enumeration(value, key, line, kBillboardNames, info.billboard);
        if (key == "billboard_keep_scale")
            return boolean(value, key, line, info.billboardKeepScale);
        if (key == "receive_shadows") return boolean(value, key, line, info.receiveShadows);
        if (key == "backlight") return vec3(value, key, line, info.backlight);

        if (key == "grass") return boolean(value, key, line, info.grass);
        if (key == "grass_draw_distance") return number(value, key, line, info.grassDrawDistance);
        if (key == "grass_camera_bend") return number(value, key, line, info.grassCameraBend);
        if (key == "grass_tint_a") return vec3(value, key, line, info.grassTintA);
        if (key == "grass_tint_b") return vec3(value, key, line, info.grassTintB);
        if (key == "grass_wind_direction") return vec3(value, key, line, info.grassWindDirection);
        if (key == "grass_wind_amplitude") return number(value, key, line, info.grassWindAmplitude);
        if (key == "grass_wind_speed") return number(value, key, line, info.grassWindSpeed);
        if (key == "leaf_wind") return boolean(value, key, line, info.leafWind);
        if (key == "leaf_wind_direction") return vec3(value, key, line, info.leafWindDirection);
        if (key == "leaf_wind_strength") return number(value, key, line, info.leafWindStrength);
        if (key == "leaf_wind_speed") return number(value, key, line, info.leafWindSpeed);
        if (key == "leaf_wind_height") return number(value, key, line, info.leafWindHeight);
        if (key == "water") return boolean(value, key, line, info.water);
        if (key == "water_wind_direction")
            return vec2(value, key, line, info.waterData.windDirection);
        if (key == "water_wind_speed") return number(value, key, line, info.waterData.windSpeed);
        if (key == "water_waves_big") return vec2(value, key, line, info.waterData.bigWaves);
        if (key == "water_waves_mid") return vec2(value, key, line, info.waterData.midWaves);
        if (key == "water_waves_small") return vec2(value, key, line, info.waterData.smallWaves);
        if (key == "water_wave_scale") return number(value, key, line, info.waterData.waveScale);
        if (key == "water_wave_chop") return number(value, key, line, info.waterData.waveChop);
        if (key == "water_bump") return number(value, key, line, info.waterData.bump);
        if (key == "water_bump2") return number(value, key, line, info.waterData.bump2);
        if (key == "water_specular_mix")
            return number(value, key, line, info.waterData.specularMix);
        if (key == "water_depth_fade") return vec3(value, key, line, info.waterData.depthFade);
        if (key == "water_depth_scale") return number(value, key, line, info.waterData.depthScale);
        if (key == "water_shore_fade") return number(value, key, line, info.waterData.shoreFade);
        if (key == "terrain_morph") return boolean(value, key, line, info.terrainMorph);

        ct::String message("unknown key '");
        message += key;
        message += "' ignored";
        warn(line, message);
        return true;
    }
};

void appendNumber(ct::String& out, float value)
{
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%g", static_cast<double>(value));
    out += buffer;
}

void appendVector(ct::String& out, const float* values, int count)
{
    out += "(";
    for (int i = 0; i < count; ++i)
    {
        if (i > 0) out += ", ";
        appendNumber(out, values[i]);
    }
    out += ")";
}

void writeKey(ct::String& out, const char* key)
{
    out += "    ";
    out += key;
    const std::size_t width = 16;
    const std::size_t length = strlen(key);
    for (std::size_t i = length; i < width; ++i) out += " ";
    out += " = ";
}

void writeNumber(ct::String& out, const char* key, float value)
{
    writeKey(out, key);
    appendNumber(out, value);
    out += "\n";
}

void writeBool(ct::String& out, const char* key, bool value)
{
    writeKey(out, key);
    out += value ? "true" : "false";
    out += "\n";
}

void writeVector(ct::String& out, const char* key, const float* values, int count)
{
    writeKey(out, key);
    appendVector(out, values, count);
    out += "\n";
}

void writeName(ct::String& out, const char* key, const char* value)
{
    writeKey(out, key);
    out += value;
    out += "\n";
}

void writeString(ct::String& out, const char* key, const ct::String& value)
{
    writeKey(out, key);
    out += "\"";
    out += value;
    out += "\"\n";
}

void writeList(ct::String& out, const char* key, const ct::Vector<ct::String>& values)
{
    writeKey(out, key);
    out += "[";
    for (std::size_t i = 0; i < values.size(); ++i)
    {
        if (i > 0) out += ", ";
        out += "\"";
        out += values[i];
        out += "\"";
    }
    out += "]\n";
}

void writeTexture(ct::String& out, const MaterialDef& def, MaterialTexture texture)
{
    const std::size_t slot = static_cast<std::size_t>(texture);
    const char* key = kTextureKeys[slot];
    if (!def.sequences[slot].empty())
    {
        writeList(out, key, def.sequences[slot]);
        if (def.sequenceFps[slot] > 0.0f)
        {
            ct::String rateKey(key);
            rateKey += "_fps";
            writeNumber(out, rateKey.c_str(), def.sequenceFps[slot]);
        }
        return;
    }
    if (!def.paths[slot].empty()) writeString(out, key, def.paths[slot]);
}

bool sameVector(const float* a, const float* b, int count)
{
    for (int i = 0; i < count; ++i)
        if (a[i] != b[i]) return false;
    return true;
}

void writeMaterial(ct::String& out, const MaterialDef& def)
{
    static const MaterialInfo defaults = MaterialInfo();
    const MaterialInfo& m = def.info;
    out += "material \"";
    out += m.name;
    out += "\" {\n";

    if (m.transparency != defaults.transparency)
        writeName(out, "transparency", nameOf(kTransparencyNames, m.transparency));
    if (m.alphaScissorThreshold != defaults.alphaScissorThreshold)
        writeNumber(out, "alpha_cutoff", m.alphaScissorThreshold);
    if (m.alphaAntialiasing != defaults.alphaAntialiasing)
        writeName(out, "alpha_antialiasing", nameOf(kAlphaAntialiasingNames, m.alphaAntialiasing));
    if (m.alphaAntialiasingEdge != defaults.alphaAntialiasingEdge)
        writeNumber(out, "alpha_antialiasing_edge", m.alphaAntialiasingEdge);
    if (m.blend != defaults.blend) writeName(out, "blend", nameOf(kBlendNames, m.blend));
    if (m.cull != defaults.cull) writeName(out, "cull", nameOf(kCullNames, m.cull));
    if (m.depthTest != defaults.depthTest) writeBool(out, "depth_test", m.depthTest);
    if (m.shading != defaults.shading) writeBool(out, "lit", m.shading == MaterialShading::Lit);

    if (!sameVector(m.albedo.v, defaults.albedo.v, 4)) writeVector(out, "color", m.albedo.v, 4);
    writeTexture(out, def, MaterialTexture::Albedo);
    if (m.metallic != defaults.metallic) writeNumber(out, "metallic", m.metallic);
    writeTexture(out, def, MaterialTexture::Metallic);
    if (m.metallicChannel != defaults.metallicChannel)
        writeName(out, "metallic_channel", nameOf(kChannelNames, m.metallicChannel));
    if (m.specular != defaults.specular) writeNumber(out, "specular", m.specular);
    if (m.roughness != defaults.roughness) writeNumber(out, "roughness", m.roughness);
    writeTexture(out, def, MaterialTexture::Roughness);
    if (m.roughnessChannel != defaults.roughnessChannel)
        writeName(out, "roughness_channel", nameOf(kChannelNames, m.roughnessChannel));
    writeTexture(out, def, MaterialTexture::Orm);
    writeTexture(out, def, MaterialTexture::AmbientOcclusion);
    if (m.aoChannel != defaults.aoChannel)
        writeName(out, "occlusion_channel", nameOf(kChannelNames, m.aoChannel));
    if (m.aoLightAffect != defaults.aoLightAffect)
        writeNumber(out, "ao_light_affect", m.aoLightAffect);
    if (m.refraction != defaults.refraction) writeNumber(out, "refraction", m.refraction);
    writeTexture(out, def, MaterialTexture::Refraction);
    if (m.refractionChannel != defaults.refractionChannel)
        writeName(out, "refraction_channel", nameOf(kChannelNames, m.refractionChannel));
    if (!sameVector(m.emission.v, defaults.emission.v, 3))
        writeVector(out, "emission_color", m.emission.v, 3);
    if (m.emissionEnergy != defaults.emissionEnergy)
        writeNumber(out, "emission_energy", m.emissionEnergy);
    if (m.emissionOperator != defaults.emissionOperator)
        writeName(out, "emission_operator", nameOf(kEmissionOperatorNames, m.emissionOperator));
    writeTexture(out, def, MaterialTexture::Emission);

    writeTexture(out, def, MaterialTexture::Normal);
    if (m.normalScale != defaults.normalScale) writeNumber(out, "normal_scale", m.normalScale);
    if (m.clearcoat != defaults.clearcoat) writeNumber(out, "clearcoat", m.clearcoat);
    if (m.clearcoatRoughness != defaults.clearcoatRoughness)
        writeNumber(out, "clearcoat_roughness", m.clearcoatRoughness);
    writeTexture(out, def, MaterialTexture::Clearcoat);

    if (!sameVector(m.uv1Scale.v, defaults.uv1Scale.v, 3))
        writeVector(out, "uv1_scale", m.uv1Scale.v, 3);
    if (!sameVector(m.uv1Offset.v, defaults.uv1Offset.v, 3))
        writeVector(out, "uv1_offset", m.uv1Offset.v, 3);
    if (!sameVector(m.uv2Scale.v, defaults.uv2Scale.v, 3))
        writeVector(out, "uv2_scale", m.uv2Scale.v, 3);
    if (!sameVector(m.uv2Offset.v, defaults.uv2Offset.v, 3))
        writeVector(out, "uv2_offset", m.uv2Offset.v, 3);

    if (m.detail != defaults.detail) writeBool(out, "detail", m.detail);
    writeTexture(out, def, MaterialTexture::DetailAlbedo);
    writeTexture(out, def, MaterialTexture::DetailNormal);
    writeTexture(out, def, MaterialTexture::DetailMask);
    if (m.detailBlend != defaults.detailBlend)
        writeName(out, "detail_blend", nameOf(kDetailBlendNames, m.detailBlend));
    if (m.detailUv != defaults.detailUv)
        writeName(out, "detail_uv", nameOf(kDetailUvNames, m.detailUv));

    if (m.billboard != defaults.billboard)
        writeName(out, "billboard", nameOf(kBillboardNames, m.billboard));
    if (m.billboardKeepScale != defaults.billboardKeepScale)
        writeBool(out, "billboard_keep_scale", m.billboardKeepScale);
    if (m.receiveShadows != defaults.receiveShadows)
        writeBool(out, "receive_shadows", m.receiveShadows);
    if (!sameVector(m.backlight.v, defaults.backlight.v, 3))
        writeVector(out, "backlight", m.backlight.v, 3);

    if (m.grass != defaults.grass) writeBool(out, "grass", m.grass);
    if (m.grassDrawDistance != defaults.grassDrawDistance)
        writeNumber(out, "grass_draw_distance", m.grassDrawDistance);
    if (m.grassCameraBend != defaults.grassCameraBend)
        writeNumber(out, "grass_camera_bend", m.grassCameraBend);
    if (!sameVector(m.grassTintA.v, defaults.grassTintA.v, 3))
        writeVector(out, "grass_tint_a", m.grassTintA.v, 3);
    if (!sameVector(m.grassTintB.v, defaults.grassTintB.v, 3))
        writeVector(out, "grass_tint_b", m.grassTintB.v, 3);
    if (!sameVector(m.grassWindDirection.v, defaults.grassWindDirection.v, 3))
        writeVector(out, "grass_wind_direction", m.grassWindDirection.v, 3);
    if (m.grassWindAmplitude != defaults.grassWindAmplitude)
        writeNumber(out, "grass_wind_amplitude", m.grassWindAmplitude);
    if (m.grassWindSpeed != defaults.grassWindSpeed)
        writeNumber(out, "grass_wind_speed", m.grassWindSpeed);
    if (m.leafWind != defaults.leafWind) writeBool(out, "leaf_wind", m.leafWind);
    if (!sameVector(m.leafWindDirection.v, defaults.leafWindDirection.v, 3))
        writeVector(out, "leaf_wind_direction", m.leafWindDirection.v, 3);
    if (m.leafWindStrength != defaults.leafWindStrength)
        writeNumber(out, "leaf_wind_strength", m.leafWindStrength);
    if (m.leafWindSpeed != defaults.leafWindSpeed)
        writeNumber(out, "leaf_wind_speed", m.leafWindSpeed);
    if (m.leafWindHeight != defaults.leafWindHeight)
        writeNumber(out, "leaf_wind_height", m.leafWindHeight);
    if (m.terrainMorph != defaults.terrainMorph) writeBool(out, "terrain_morph", m.terrainMorph);
    if (m.water != defaults.water) writeBool(out, "water", m.water);
    const WaterMaterialData& water = m.waterData;
    const WaterMaterialData& waterDefaults = defaults.waterData;
    if (!sameVector(water.windDirection.v, waterDefaults.windDirection.v, 2))
        writeVector(out, "water_wind_direction", water.windDirection.v, 2);
    if (water.windSpeed != waterDefaults.windSpeed)
        writeNumber(out, "water_wind_speed", water.windSpeed);
    if (!sameVector(water.bigWaves.v, waterDefaults.bigWaves.v, 2))
        writeVector(out, "water_waves_big", water.bigWaves.v, 2);
    if (!sameVector(water.midWaves.v, waterDefaults.midWaves.v, 2))
        writeVector(out, "water_waves_mid", water.midWaves.v, 2);
    if (!sameVector(water.smallWaves.v, waterDefaults.smallWaves.v, 2))
        writeVector(out, "water_waves_small", water.smallWaves.v, 2);
    if (water.waveScale != waterDefaults.waveScale)
        writeNumber(out, "water_wave_scale", water.waveScale);
    if (water.waveChop != waterDefaults.waveChop)
        writeNumber(out, "water_wave_chop", water.waveChop);
    if (water.bump != waterDefaults.bump) writeNumber(out, "water_bump", water.bump);
    if (water.bump2 != waterDefaults.bump2) writeNumber(out, "water_bump2", water.bump2);
    if (water.specularMix != waterDefaults.specularMix)
        writeNumber(out, "water_specular_mix", water.specularMix);
    if (!sameVector(water.depthFade.v, waterDefaults.depthFade.v, 3))
        writeVector(out, "water_depth_fade", water.depthFade.v, 3);
    if (water.depthScale != waterDefaults.depthScale)
        writeNumber(out, "water_depth_scale", water.depthScale);
    if (water.shoreFade != waterDefaults.shoreFade)
        writeNumber(out, "water_shore_fade", water.shoreFade);

    out += "}\n";
}

} // namespace

const MaterialDef* MaterialLibraryData::find(const char* name) const
{
    if (!name) return nullptr;
    for (std::size_t i = 0; i < materials.size(); ++i)
        if (materials[i].info.name == name) return &materials[i];
    return nullptr;
}

const char* MaterialLibraryFormat::textureKey(MaterialTexture slot)
{
    const std::size_t index = static_cast<std::size_t>(slot);
    return index < kMaterialTextureCount ? kTextureKeys[index] : "";
}

bool MaterialLibraryFormat::parse(const char* text, std::size_t size, MaterialLibraryData& out,
        ct::String& error)
{
    out.materials.clear();
    out.warnings.clear();
    error.clear();
    const ct::String source = size ? ct::String(text, size) : ct::String();
    Parser parser(source, out.warnings);
    if (parser.parseFile(out.materials)) return true;
    out.materials.clear();
    error = parser.error();
    return false;
}

void MaterialLibraryFormat::write(const MaterialLibraryData& data, ct::String& out)
{
    for (std::size_t i = 0; i < data.materials.size(); ++i)
    {
        if (!out.empty()) out += "\n";
        writeMaterial(out, data.materials[i]);
    }
}

bool MaterialLibraryFormat::load(const ct::String& path, MaterialLibraryData& out,
        ct::String& error)
{
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(path);
    if (bytes.empty())
    {
        out.materials.clear();
        out.warnings.clear();
        error = path + ": not found or empty";
        return false;
    }
    if (parse(reinterpret_cast<const char*>(bytes.data()), bytes.size(), out, error)) return true;
    error = path + ": " + error;
    return false;
}

bool MaterialLibraryFormat::save(const ct::String& path, const MaterialLibraryData& data,
        ct::String& error)
{
    ct::String text;
    write(data, text);
    const ct::String temporary = path + ".tmp";
    fs_remove_path(temporary.c_str());
    if (!file_write(temporary.c_str(), text.data(), text.size()))
    {
        error = ct::String("could not write '") + temporary + "'";
        return false;
    }
    if (!fs_rename_path(temporary.c_str(), path.c_str()))
    {
        fs_remove_path(temporary.c_str());
        error = ct::String("could not replace '") + path + "'";
        return false;
    }
    error.clear();
    return true;
}

} // namespace prisma3d
