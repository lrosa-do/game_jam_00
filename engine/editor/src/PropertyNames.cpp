#include "prisma3d/editor/PropertyNames.h"

#include <ct/vector.hpp>

#include <string.h>

namespace prisma3d
{

namespace
{

struct Remap
{
    const char* from;
    const char* to;
};

const Remap kRemaps[] = {
    { "2d", "2D" },
    { "3d", "3D" },
    { "4d", "4D" },
    { "aa", "AA" },
    { "aabb", "AABB" },
    { "adb", "ADB" },
    { "actool", "actool" },
    { "ao", "AO" },
    { "androidxr", "AndroidXR" },
    { "api", "API" },
    { "apk", "APK" },
    { "arm32", "arm32" },
    { "arm64", "arm64" },
    { "arm64-v8a", "arm64-v8a" },
    { "armeabi-v7a", "armeabi-v7a" },
    { "arvr", "ARVR" },
    { "astc", "ASTC" },
    { "bbcode", "BBCode" },
    { "bg", "BG" },
    { "bidi", "BiDi" },
    { "bp", "BP" },
    { "bpc", "BPC" },
    { "bpm", "BPM" },
    { "bptc", "BPTC" },
    { "bvh", "BVH" },
    { "ca", "CA" },
    { "colr", "COLR" },
    { "ccdik", "CCDIK" },
    { "cd", "CD" },
    { "cpu", "CPU" },
    { "csg", "CSG" },
    { "d3d12", "D3D12" },
    { "db", "dB" },
    { "dof", "DoF" },
    { "dpi", "DPI" },
    { "dtls", "DTLS" },
    { "eol", "EOL" },
    { "erp", "ERP" },
    { "etc2", "ETC2" },
    { "fabrik", "FABRIK" },
    { "fbx", "FBX" },
    { "fbx2gltf", "FBX2glTF" },
    { "fft", "FFT" },
    { "fg", "FG" },
    { "filesystem", "FileSystem" },
    { "fov", "FOV" },
    { "fps", "FPS" },
    { "fs", "FS" },
    { "fsr", "FSR" },
    { "fxaa", "FXAA" },
    { "gdscript", "GDScript" },
    { "ggx", "GGX" },
    { "gi", "GI" },
    { "gl", "GL" },
    { "glb", "GLB" },
    { "gles", "GLES" },
    { "gles2", "GLES2" },
    { "gles3", "GLES3" },
    { "gltf", "glTF" },
    { "gridmap", "GridMap" },
    { "gpu", "GPU" },
    { "gui", "GUI" },
    { "guid", "GUID" },
    { "hdr", "HDR" },
    { "hidpi", "hiDPI" },
    { "hipass", "High-pass" },
    { "hl", "HL" },
    { "hsv", "HSV" },
    { "html", "HTML" },
    { "http", "HTTP" },
    { "id", "ID" },
    { "ids", "IDs" },
    { "igd", "IGD" },
    { "ik", "IK" },
    { "image@2x", "Image @2x" },
    { "image@3x", "Image @3x" },
    { "ime", "IME" },
    { "iod", "IOD" },
    { "ios", "iOS" },
    { "ip", "IP" },
    { "ipad", "iPad" },
    { "iphone", "iPhone" },
    { "ipv6", "IPv6" },
    { "ir", "IR" },
    { "itunes", "iTunes" },
    { "jit", "JIT" },
    { "k1", "K1" },
    { "k2", "K2" },
    { "kb", "(KB)" },
    { "lcd", "LCD" },
    { "ldr", "LDR" },
    { "linuxbsd", "Linux/*BSD" },
    { "lod", "LOD" },
    { "lods", "LODs" },
    { "loongarch64", "loongarch64" },
    { "lowpass", "Low-pass" },
    { "macos", "macOS" },
    { "mb", "(MB)" },
    { "mjpeg", "MJPEG" },
    { "mms", "MMS" },
    { "ms", "(ms)" },
    { "msaa", "MSAA" },
    { "msdf", "MSDF" },
    { "msec", "(msec)" },
    { "navmesh", "NavMesh" },
    { "nfc", "NFC" },
    { "ogv", "OGV" },
    { "oidn", "OIDN" },
    { "ok", "OK" },
    { "opengl", "OpenGL" },
    { "opengl3", "OpenGL 3" },
    { "opentype", "OpenType" },
    { "openxr", "OpenXR" },
    { "osslsigncode", "osslsigncode" },
    { "path3d", "Path3D" },
    { "pck", "PCK" },
    { "png", "PNG" },
    { "po2", "(Power of 2)" },
    { "ppc64", "ppc64" },
    { "pvrtc", "PVRTC" },
    { "pvs", "PVS" },
    { "rcodesign", "rcodesign" },
    { "rdo", "RDO" },
    { "rgb", "RGB" },
    { "rid", "RID" },
    { "rmb", "RMB" },
    { "rpc", "RPC" },
    { "rv64", "rv64" },
    { "s3tc", "S3TC" },
    { "scp", "SCP" },
    { "scrcpy", "scrcpy" },
    { "sdf", "SDF" },
    { "sdfgi", "SDFGI" },
    { "sdk", "SDK" },
    { "sec", "(sec)" },
    { "signtool", "signtool" },
    { "smaa", "SMAA" },
    { "sms", "SMS" },
    { "srgb", "sRGB" },
    { "ssao", "SSAO" },
    { "ssh", "SSH" },
    { "ssil", "SSIL" },
    { "ssl", "SSL" },
    { "sss", "SSS" },
    { "stderr", "stderr" },
    { "stdout", "stdout" },
    { "sv", "SV" },
    { "svg", "SVG" },
    { "taa", "TAA" },
    { "tcp", "TCP" },
    { "textfile", "TextFile" },
    { "tls", "TLS" },
    { "tv", "TV" },
    { "tvos", "tvOS" },
    { "uastc", "UASTC" },
    { "ui", "UI" },
    { "uri", "URI" },
    { "url", "URL" },
    { "urls", "URLs" },
    { "usb", "USB" },
    { "uid", "UID" },
    { "uuid", "UUID" },
    { "uv", "UV" },
    { "uv1", "UV1" },
    { "uv2", "UV2" },
    { "vector2", "Vector2" },
    { "visionos", "visionOS" },
    { "vpn", "VPN" },
    { "vram", "VRAM" },
    { "vrs", "VRS" },
    { "vsync", "V-Sync" },
    { "wap", "WAP" },
    { "webp", "WebP" },
    { "webrtc", "WebRTC" },
    { "websocket", "WebSocket" },
    { "wintab", "WinTab" },
    { "winink", "Windows Ink" },
    { "wifi", "Wi-Fi" },
    { "x86", "x86" },
    { "x86_32", "x86_32" },
    { "x86_64", "x86_64" },
    { "xr", "XR" },
    { "xray", "X-Ray" },
    { "xy", "XY" },
    { "xz", "XZ" },
    { "yz", "YZ" },
};

const char* const kStopWords[] = { "a", "an", "and", "as", "at", "by", "for", "in", "not", "of",
    "on", "or", "over", "per", "the", "then", "to", "with" };

bool isUpper(char c)
{
    return c >= 'A' && c <= 'Z';
}

bool isLower(char c)
{
    return c >= 'a' && c <= 'z';
}

bool isDigit(char c)
{
    return c >= '0' && c <= '9';
}

ct::String separateCompoundWords(const ct::String& text)
{
    if (text.empty()) return text;

    std::size_t start = 0;
    ct::String separated;
    bool previousUpper = isUpper(text[0]);
    bool previousLower = isLower(text[0]);
    bool previousDigit = isDigit(text[0]);

    for (std::size_t i = 1; i < text.size(); ++i)
    {
        const bool currentUpper = isUpper(text[i]);
        const bool currentLower = isLower(text[i]);
        const bool currentDigit = isDigit(text[i]);
        const bool nextLower = i + 1 < text.size() && isLower(text[i + 1]);

        const bool a = previousLower && currentUpper;
        const bool b = (previousUpper || previousDigit) && currentUpper && nextLower;
        const bool c = previousDigit && currentLower && nextLower;
        const bool d = (previousUpper || previousLower) && currentDigit;

        if (a || b || c || d)
        {
            separated += text.substr(start, i - start);
            separated += ' ';
            start = i;
        }

        previousUpper = currentUpper;
        previousLower = currentLower;
        previousDigit = currentDigit;
    }
    separated += text.substr(start, text.size() - start);

    for (std::size_t i = 0; i < separated.size(); ++i)
    {
        char& c = separated.data()[i];
        if (c == '\t' || c == '\n' || c == '\r' || c == '_' || c == '-') c = ' ';
        if (isUpper(c)) c = static_cast<char>(c - 'A' + 'a');
    }
    return separated;
}

ct::String capitalizeWords(const ct::String& text)
{
    const ct::String words = separateCompoundWords(text).trimmed();
    ct::String result;
    std::size_t start = 0;
    for (std::size_t i = 0; i <= words.size(); ++i)
    {
        if (i != words.size() && words[i] != ' ') continue;
        ct::String slice = words.substr(start, i - start);
        start = i + 1;
        if (slice.empty()) continue;
        if (isLower(slice[0])) slice.data()[0] = static_cast<char>(slice[0] - 'a' + 'A');
        if (!result.empty()) result += ' ';
        result += slice;
    }
    return result;
}

bool isStopWord(const ct::String& word)
{
    for (const char* stop: kStopWords)
        if (word == stop) return true;
    return false;
}

const char* remapOf(const ct::String& word)
{
    for (const Remap& remap: kRemaps)
        if (word == remap.from) return remap.to;
    return nullptr;
}

} // namespace

ct::String capitalizePropertyName(const ct::String& name)
{
    ct::Vector<ct::String> parts;
    std::size_t start = 0;
    for (std::size_t i = 0; i <= name.size(); ++i)
    {
        if (i != name.size() && name[i] != '_') continue;
        if (i > start) parts.push_back(name.substr(start, i - start));
        start = i + 1;
    }

    ct::String result;
    for (std::size_t i = 0; i < parts.size(); ++i)
    {
        ct::String word = parts[i];
        const bool inner = i > 0 && i + 1 < parts.size();
        if (!(inner && isStopWord(word)))
        {
            const char* remap = remapOf(word);
            word = remap ? ct::String(remap) : capitalizeWords(word);
        }
        if (i > 0) result += ' ';
        result += word;
    }
    return result;
}

const char* propertyLabelOverride(const ct::String& name)
{
    if (name == "collision_type") return "Group";
    if (name == "reflector") return "Planar Reflector";
    return nullptr;
}

} // namespace prisma3d
