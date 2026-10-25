#include "prisma3d/render/ShaderCompiler.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/render/ShaderSources.h"

#include "miniz.h"
#include "platform.h"

#include <SPIRV/GlslangToSpv.h>
#include <glslang/Public/ResourceLimits.h>
#include <glslang/Public/ShaderLang.h>
#include <spirv_glsl.hpp>

#include <ct/hashmap.hpp>
#include <ct/vector.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const char* const kToolchainTag = "prisma3d shader cache 1, glslang 15.4.0, spirv-cross";
const std::uint32_t kCacheMagic = 0x32435350u;
const char* const kSourcePrefix = "shaders/";

struct Compiled
{
    ct::Vector<std::uint32_t> spirv;
    ct::String glsl;
    ct::String essl[4];
    ct::Vector<ct::String> bindingNames;
    ct::Vector<prisma::ShaderBinding> bindings;
    prisma::ShaderBlob blob;
    bool ok = false;
};

struct State
{
    ct::HashMap<std::uint64_t, Compiled*> compiled;
    ct::Vector<std::uint8_t> sources;
    ct::String cacheDirectory;
    ct::Vector<ct::String> cacheFiles;
    unsigned long long sourcesHash = 0;
    bool sourcesReady = false;
    bool glslangReady = false;
    bool cacheEnabled = true;
    bool cacheChecked = false;
    ShaderCompilerStats stats;
};

State* sState = nullptr;

State& state()
{
    if (!sState) sState = new State();
    return *sState;
}

std::uint64_t fnv(std::uint64_t hash, const void* data, std::size_t size)
{
    const std::uint8_t* bytes = static_cast<const std::uint8_t*>(data);
    for (std::size_t i = 0; i < size; ++i)
    {
        hash ^= bytes[i];
        hash *= 1099511628211ull;
    }
    return hash;
}

std::uint64_t fnvText(std::uint64_t hash, const char* text)
{
    return fnv(hash, text, strlen(text));
}

bool ensureSources()
{
    State& s = state();
    if (s.sourcesReady) return !s.sources.empty();
    s.sourcesReady = true;
    s.sources.resize(kShaderSourcesUnpackedSize + 1);
    mz_ulong size = kShaderSourcesUnpackedSize;
    if (mz_uncompress(s.sources.data(), &size, kShaderSourcesPacked, kShaderSourcesPackedSize) !=
                    MZ_OK ||
            size != kShaderSourcesUnpackedSize)
    {
        log_error("shaders: the embedded shader sources could not be inflated");
        s.sources.clear();
        return false;
    }
    s.sourcesHash = fnv(14695981039346656037ull, s.sources.data(), kShaderSourcesUnpackedSize);
    return true;
}

const char* embeddedSource(const char* name, std::size_t& size)
{
    size = 0;
    if (!ensureSources()) return nullptr;
    for (unsigned i = 0; i < kShaderSourceCount; ++i)
    {
        if (strcmp(kShaderSources[i].name, name) != 0) continue;
        size = kShaderSources[i].size - 1;
        return reinterpret_cast<const char*>(state().sources.data()) + kShaderSources[i].offset;
    }
    return nullptr;
}

struct Includer : public glslang::TShader::Includer
{
    struct Loaded
    {
        ct::String name;
        ct::Vector<std::uint8_t> bytes;
    };

    ct::Vector<Loaded*> loaded;

    ~Includer() override
    {
        for (std::size_t i = 0; i < loaded.size(); ++i) delete loaded[i];
    }

    IncludeResult* includeLocal(const char* headerName, const char*, size_t) override
    {
        return include(headerName);
    }

    IncludeResult* includeSystem(const char* headerName, const char*, size_t) override
    {
        return include(headerName);
    }

    IncludeResult* include(const char* headerName)
    {
        const FileSystem& files = FileSystem::getSingleton();
        const ct::String external = ct::String(kSourcePrefix) + headerName;
        if (files.exists(external))
        {
            Loaded* entry = new Loaded();
            entry->name = headerName;
            entry->bytes = files.readBinary(external);
            if (!entry->bytes.empty())
            {
                loaded.push_back(entry);
                return new IncludeResult(headerName,
                        reinterpret_cast<const char*>(entry->bytes.data()), entry->bytes.size(),
                        nullptr);
            }
            delete entry;
        }
        std::size_t size = 0;
        const char* text = embeddedSource(headerName, size);
        if (!text) return nullptr;
        return new IncludeResult(headerName, text, size, nullptr);
    }

    void releaseInclude(IncludeResult* result) override
    {
        delete result;
    }
};

EShLanguage languageOf(prisma::ShaderStage stage)
{
    switch (stage)
    {
        case prisma::ShaderStage::Fragment:
            return EShLangFragment;
        case prisma::ShaderStage::Compute:
            return EShLangCompute;
        case prisma::ShaderStage::Geometry:
            return EShLangGeometry;
        case prisma::ShaderStage::TessControl:
            return EShLangTessControl;
        case prisma::ShaderStage::TessEval:
            return EShLangTessEvaluation;
        default:
            return EShLangVertex;
    }
}

const char* stageName(prisma::ShaderStage stage)
{
    switch (stage)
    {
        case prisma::ShaderStage::Fragment:
            return "fragment";
        case prisma::ShaderStage::Compute:
            return "compute";
        case prisma::ShaderStage::Geometry:
            return "geometry";
        case prisma::ShaderStage::TessControl:
            return "tess control";
        case prisma::ShaderStage::TessEval:
            return "tess eval";
        default:
            return "vertex";
    }
}

bool ensureGlslang()
{
    State& s = state();
    if (s.glslangReady) return true;
    s.glslangReady = glslang::InitializeProcess();
    if (!s.glslangReady) log_error("shaders: glslang could not be initialised");
    return s.glslangReady;
}

const ct::String& cacheDirectory()
{
    State& s = state();
    if (s.cacheChecked) return s.cacheDirectory;
    s.cacheChecked = true;
    if (s.cacheDirectory.empty())
    {
        const char* xdg = getenv("XDG_CACHE_HOME");
        const char* home = getenv("HOME");
        if (xdg && xdg[0]) s.cacheDirectory = ct::String(xdg) + "/prisma3d/shaders";
        else if (home && home[0])
            s.cacheDirectory = ct::String(home) + "/.cache/prisma3d/shaders";
    }
    if (!s.cacheDirectory.empty() && !fs_create_directory(s.cacheDirectory.c_str()))
    {
        log_warn("shaders: the cache folder %s could not be created", s.cacheDirectory.c_str());
        s.cacheDirectory = ct::String();
    }
    return s.cacheDirectory;
}

ct::String cacheName(std::uint64_t key)
{
    char name[32];
    snprintf(name, sizeof(name), "%016llx.psc", static_cast<unsigned long long>(key));
    return ct::String(name);
}

ct::String cachePath(std::uint64_t key)
{
    const ct::String& directory = cacheDirectory();
    if (directory.empty()) return ct::String();
    return directory + "/" + cacheName(key);
}

void rememberCacheFile(const ct::String& name)
{
    ct::Vector<ct::String>& files = state().cacheFiles;
    for (std::size_t i = 0; i < files.size(); ++i)
        if (files[i] == name) return;
    files.push_back(name);
}

bool readDisk(const ct::String& path, ct::Vector<std::uint8_t>& out)
{
    FILE* file = fopen(path.c_str(), "rb");
    if (!file) return false;
    std::uint8_t chunk[65536];
    for (;;)
    {
        const std::size_t got = fread(chunk, 1, sizeof(chunk), file);
        for (std::size_t i = 0; i < got; ++i) out.push_back(chunk[i]);
        if (got < sizeof(chunk)) break;
    }
    fclose(file);
    return !out.empty();
}

void append(ct::Vector<std::uint8_t>& out, const void* data, std::size_t size)
{
    const std::uint8_t* bytes = static_cast<const std::uint8_t*>(data);
    for (std::size_t i = 0; i < size; ++i) out.push_back(bytes[i]);
}

void appendU32(ct::Vector<std::uint8_t>& out, std::uint32_t value)
{
    append(out, &value, sizeof(value));
}

struct Reader
{
    const std::uint8_t* data;
    std::size_t size;
    std::size_t cursor = 0;
    bool ok = true;

    std::uint32_t u32()
    {
        std::uint32_t value = 0;
        if (cursor + 4 > size)
        {
            ok = false;
            return 0;
        }
        memcpy(&value, data + cursor, 4);
        cursor += 4;
        return value;
    }

    const std::uint8_t* bytes(std::size_t count)
    {
        if (cursor + count > size)
        {
            ok = false;
            return nullptr;
        }
        const std::uint8_t* at = data + cursor;
        cursor += count;
        return at;
    }
};

void finishBlob(Compiled& c, prisma::ShaderStage stage)
{
    for (std::size_t i = 0; i < c.bindingNames.size() && i < c.bindings.size(); ++i)
        c.bindings[i].name = c.bindingNames[i].c_str();
    prisma::ShaderBlob& blob = c.blob;
    blob.stage = stage;
    blob.spirv = c.spirv.data();
    blob.spirvSize = static_cast<std::uint32_t>(c.spirv.size() * sizeof(std::uint32_t));
    blob.glsl = c.glsl.empty() ? nullptr : c.glsl.c_str();
    blob.essl300 = c.essl[0].empty() ? nullptr : c.essl[0].c_str();
    blob.essl310 = c.essl[1].empty() ? nullptr : c.essl[1].c_str();
    blob.essl320 = c.essl[2].empty() ? nullptr : c.essl[2].c_str();
    blob.essl320Inner = c.essl[3].empty() ? nullptr : c.essl[3].c_str();
    blob.bindings = c.bindings.empty() ? nullptr : c.bindings.data();
    blob.bindingCount = static_cast<std::uint32_t>(c.bindings.size());
    blob.packed = nullptr;
    blob.packedSize = 0;
}

bool loadCached(Compiled& c, std::uint64_t key)
{
    const ct::String name = cacheName(key);
    ct::Vector<std::uint8_t> packed =
            FileSystem::getSingleton().readBinary(ct::String(kShaderCachePackFolder) + "/" + name);
    if (packed.empty())
    {
        const ct::String path = cachePath(key);
        if (path.empty() || !readDisk(path, packed)) return false;
    }
    Reader header{ packed.data(), packed.size() };
    if (header.u32() != kCacheMagic) return false;
    const std::uint32_t rawSize = header.u32();
    if (!header.ok || rawSize == 0 || rawSize > (256u << 20)) return false;
    ct::Vector<std::uint8_t> file;
    file.resize(rawSize);
    mz_ulong inflated = rawSize;
    if (mz_uncompress(file.data(), &inflated, packed.data() + 8,
                static_cast<mz_ulong>(packed.size() - 8)) != MZ_OK ||
            inflated != rawSize)
        return false;
    Reader r{ file.data(), file.size() };
    const std::uint32_t words = r.u32();
    const std::uint8_t* spirv = r.bytes(static_cast<std::size_t>(words) * 4);
    const std::uint32_t glslSize = r.u32();
    const std::uint8_t* glsl = r.bytes(glslSize);
    const std::uint32_t bindingCount = r.u32();
    if (!r.ok || !spirv || !glsl || words < 5) return false;
    c.spirv.resize(words);
    memcpy(c.spirv.data(), spirv, static_cast<std::size_t>(words) * 4);
    c.glsl = ct::String(reinterpret_cast<const char*>(glsl), glslSize);
    c.bindingNames.clear();
    c.bindings.clear();
    for (std::uint32_t i = 0; i < bindingCount; ++i)
    {
        const std::uint32_t kind = r.u32();
        const std::uint32_t slot = r.u32();
        const std::uint32_t length = r.u32();
        const std::uint8_t* name = r.bytes(length);
        if (!r.ok || !name) return false;
        prisma::ShaderBinding binding;
        binding.kind = static_cast<prisma::BindingKind>(kind);
        binding.slot = slot;
        c.bindings.push_back(binding);
        c.bindingNames.push_back(ct::String(reinterpret_cast<const char*>(name), length));
    }
    if (r.ok) rememberCacheFile(name);
    return r.ok;
}

void storeCached(const Compiled& c, std::uint64_t key)
{
    const ct::String path = cachePath(key);
    if (path.empty()) return;
    ct::Vector<std::uint8_t> out;
    appendU32(out, static_cast<std::uint32_t>(c.spirv.size()));
    append(out, c.spirv.data(), c.spirv.size() * 4);
    appendU32(out, static_cast<std::uint32_t>(c.glsl.size()));
    append(out, c.glsl.c_str(), c.glsl.size());
    appendU32(out, static_cast<std::uint32_t>(c.bindings.size()));
    for (std::size_t i = 0; i < c.bindings.size(); ++i)
    {
        appendU32(out, static_cast<std::uint32_t>(c.bindings[i].kind));
        appendU32(out, c.bindings[i].slot);
        appendU32(out, static_cast<std::uint32_t>(c.bindingNames[i].size()));
        append(out, c.bindingNames[i].c_str(), c.bindingNames[i].size());
    }
    mz_ulong packedSize = mz_compressBound(static_cast<mz_ulong>(out.size()));
    ct::Vector<std::uint8_t> packed;
    packed.resize(static_cast<std::size_t>(packedSize) + 8);
    if (mz_compress2(packed.data() + 8, &packedSize, out.data(), static_cast<mz_ulong>(out.size()),
                MZ_DEFAULT_LEVEL) != MZ_OK)
        return;
    packed.resize(static_cast<std::size_t>(packedSize) + 8);
    const std::uint32_t magic = kCacheMagic;
    const std::uint32_t rawSize = static_cast<std::uint32_t>(out.size());
    memcpy(packed.data(), &magic, 4);
    memcpy(packed.data() + 4, &rawSize, 4);
    if (FileSystem::writeBinary(path, packed.data(), packed.size()))
        rememberCacheFile(cacheName(key));
}

void collectBindings(spirv_cross::Compiler& compiler, Compiled& c)
{
    const spirv_cross::ShaderResources resources = compiler.get_shader_resources();
    const auto add = [&](const spirv_cross::SmallVector<spirv_cross::Resource>& list,
                             prisma::BindingKind kind, bool block)
    {
        for (const spirv_cross::Resource& resource: list)
        {
            std::string name = resource.name;
            if (name.empty())
                name = compiler.get_name(block ? static_cast<std::uint32_t>(resource.base_type_id)
                                               : static_cast<std::uint32_t>(resource.id));
            prisma::ShaderBinding binding;
            binding.kind = kind;
            binding.slot = compiler.get_decoration(resource.id, spv::DecorationBinding);
            c.bindings.push_back(binding);
            c.bindingNames.push_back(ct::String(name.c_str(), name.size()));
        }
    };
    add(resources.uniform_buffers, prisma::BindingKind::UniformBlock, true);
    add(resources.sampled_images, prisma::BindingKind::Texture, false);
    add(resources.storage_buffers, prisma::BindingKind::StorageBuffer, true);
    add(resources.storage_images, prisma::BindingKind::StorageTexture, false);
}

bool crossCompile(Compiled& c, prisma::ShaderStage stage, const char* label, bool gles)
{
    try
    {
        {
            spirv_cross::CompilerGLSL glsl(c.spirv.data(), c.spirv.size());
            collectBindings(glsl, c);
            spirv_cross::CompilerGLSL::Options options;
            options.version = 460;
            options.es = false;
            options.vulkan_semantics = false;
            glsl.set_common_options(options);
            const std::string text = glsl.compile();
            c.glsl = ct::String(text.c_str(), text.size());
        }
        if (gles)
        {
            const int versions[3] = { 300, 310, 320 };
            for (int v = 0; v < 3; ++v)
            {
                spirv_cross::CompilerGLSL essl(c.spirv.data(), c.spirv.size());
                spirv_cross::CompilerGLSL::Options options;
                options.version = static_cast<std::uint32_t>(versions[v]);
                options.es = true;
                options.vulkan_semantics = false;
                options.vertex.fixup_clipspace = stage == prisma::ShaderStage::Vertex ||
                                                 stage == prisma::ShaderStage::TessEval;
                essl.set_common_options(options);
                try
                {
                    const std::string text = essl.compile();
                    c.essl[v] = ct::String(text.c_str(), text.size());
                }
                catch (const std::exception&)
                {
                    c.essl[v] = ct::String();
                }
            }
            if (stage == prisma::ShaderStage::Vertex || stage == prisma::ShaderStage::TessEval)
            {
                spirv_cross::CompilerGLSL inner(c.spirv.data(), c.spirv.size());
                spirv_cross::CompilerGLSL::Options options;
                options.version = 320;
                options.es = true;
                options.vulkan_semantics = false;
                inner.set_common_options(options);
                try
                {
                    const std::string text = inner.compile();
                    c.essl[3] = ct::String(text.c_str(), text.size());
                }
                catch (const std::exception&)
                {
                    c.essl[3] = ct::String();
                }
            }
        }
        return true;
    }
    catch (const std::exception& error)
    {
        log_error("shaders: %s: GLSL generation failed: %s", label, error.what());
        return false;
    }
}

bool compileSpirv(Compiled& c, const ShaderVariant& variant, const char* label)
{
    if (!ensureGlslang()) return false;
    ct::String text;
    std::size_t sourceSize = 0;
    const char* source = nullptr;
    ct::Vector<std::uint8_t> externalBytes;
    {
        const FileSystem& files = FileSystem::getSingleton();
        const ct::String external = ct::String(kSourcePrefix) + variant.file;
        if (files.exists(external))
        {
            externalBytes = files.readBinary(external);
            if (!externalBytes.empty())
            {
                externalBytes.push_back(0);
                source = reinterpret_cast<const char*>(externalBytes.data());
                sourceSize = externalBytes.size() - 1;
            }
        }
        if (!source) source = embeddedSource(variant.file, sourceSize);
    }
    if (source && strncmp(source, "#version", 8) == 0)
    {
        const char* lineEnd = strchr(source, '\n');
        const std::size_t firstLine =
                lineEnd ? static_cast<std::size_t>(lineEnd - source) : sourceSize;
        text = ct::String(source, firstLine);
        text += "\n#extension GL_GOOGLE_include_directive : require\n";
        text += variant.defines;
        text += "#line 2\n";
        text += ct::String(source + firstLine, sourceSize - firstLine);
    }
    else
    {
        text = "#version 450\n#extension GL_GOOGLE_include_directive : require\n";
        text += variant.defines;
        text += "#include \"";
        text += variant.file;
        text += "\"\n";
    }

    const EShLanguage language = languageOf(variant.stage);
    glslang::TShader shader(language);
    const char* strings[1] = { text.c_str() };
    shader.setStrings(strings, 1);
    shader.setEnvInput(glslang::EShSourceGlsl, language, glslang::EShClientVulkan, 100);
    shader.setEnvClient(glslang::EShClientVulkan, glslang::EShTargetVulkan_1_0);
    shader.setEnvTarget(glslang::EShTargetSpv, glslang::EShTargetSpv_1_0);
    const EShMessages messages = static_cast<EShMessages>(EShMsgSpvRules | EShMsgVulkanRules);
    Includer includer;
    if (!shader.parse(GetDefaultResources(), 450, false, messages, includer))
    {
        log_error("shaders: %s did not compile:\n%s", label, shader.getInfoLog());
        return false;
    }
    glslang::TProgram program;
    program.addShader(&shader);
    if (!program.link(messages))
    {
        log_error("shaders: %s did not link:\n%s", label, program.getInfoLog());
        return false;
    }
    glslang::SpvOptions options;
    options.disableOptimizer = true;
    spv::SpvBuildLogger logger;
    std::vector<unsigned int> words;
    glslang::GlslangToSpv(*program.getIntermediate(language), words, &logger, &options);
    if (words.size() < 5)
    {
        log_error("shaders: %s produced no SPIR-V: %s", label, logger.getAllMessages().c_str());
        return false;
    }
    c.spirv.resize(words.size());
    memcpy(c.spirv.data(), words.data(), words.size() * sizeof(std::uint32_t));
    return true;
}

ct::String describe(const ShaderVariant& variant)
{
    ct::String label(variant.file);
    label += " (";
    label += stageName(variant.stage);
    std::size_t count = 0;
    for (std::size_t i = 0; i + 7 < variant.defines.size(); ++i)
        if (variant.defines[i] == '#' && variant.defines.substr(i, 8) == "#define ") ++count;
    if (count)
    {
        label += ", ";
        label.append_number(static_cast<unsigned long long>(count));
        label += " defines";
    }
    label += ")";
    return label;
}

} // namespace

const char* shaderSource(const char* name, std::size_t& size)
{
    return embeddedSource(name, size);
}

unsigned long long shaderSourcesHash()
{
    ensureSources();
    return state().sourcesHash;
}

const prisma::ShaderBlob* compileShader(const ShaderVariant& variant, const prisma::Caps& caps)
{
    State& s = state();
    if (!variant.file || !ensureSources()) return nullptr;

    std::uint64_t key = 14695981039346656037ull;
    key = fnvText(key, kToolchainTag);
    key = fnvText(key, variant.file);
    key = fnv(key, variant.defines.c_str(), variant.defines.size());
    const std::uint32_t stage = static_cast<std::uint32_t>(variant.stage);
    key = fnv(key, &stage, sizeof(stage));
    const unsigned long long sources = s.sourcesHash;
    key = fnv(key, &sources, sizeof(sources));
    const std::uint32_t gles = caps.gles ? 1u : 0u;
    key = fnv(key, &gles, sizeof(gles));

    if (Compiled** found = s.compiled.find(key)) return (*found)->ok ? &(*found)->blob : nullptr;

    Compiled* c = new Compiled();
    s.compiled.put(key, c);
    const ct::String label = describe(variant);
    const std::uint64_t start = time_nanos();
    bool fromCache = false;
    if (s.cacheEnabled && !caps.gles && loadCached(*c, key)) fromCache = true;
    else
    {
        c->bindingNames.clear();
        c->bindings.clear();
        if (!compileSpirv(*c, variant, label.c_str()) ||
                !crossCompile(*c, variant.stage, label.c_str(), caps.gles))
        {
            ++s.stats.failed;
            return nullptr;
        }
        if (s.cacheEnabled && !caps.gles) storeCached(*c, key);
    }
    c->ok = true;
    finishBlob(*c, variant.stage);
    const double milliseconds = static_cast<double>(time_nanos() - start) * 1e-6;
    if (fromCache) ++s.stats.loaded;
    else
    {
        ++s.stats.compiled;
        s.stats.compileMilliseconds += milliseconds;
        log_info("shaders: compiled %s in %.1f ms", label.c_str(), milliseconds);
    }
    return &c->blob;
}

prisma::ShaderDesc shaderDesc(const ShaderVariant& variant, const prisma::Caps& caps)
{
    const prisma::ShaderBlob* blob = compileShader(variant, caps);
    if (!blob)
    {
        prisma::ShaderDesc empty;
        empty.stage = variant.stage;
        return empty;
    }
    return prisma::shaderDesc(*blob, caps);
}

void setShaderCacheDirectory(const ct::String& directory)
{
    State& s = state();
    s.cacheDirectory = directory;
    s.cacheChecked = false;
}

const ct::String& shaderCacheDirectory()
{
    return cacheDirectory();
}

void shaderCacheFiles(ct::Vector<ct::String>& out)
{
    out = state().cacheFiles;
}

void setShaderCacheEnabled(bool enabled)
{
    state().cacheEnabled = enabled;
}

const ShaderCompilerStats& shaderCompilerStats()
{
    return state().stats;
}

void releaseShaderCompiler()
{
    if (!sState) return;
    for (auto it = sState->compiled.begin(); it != sState->compiled.end(); ++it) delete it->value;
    if (sState->glslangReady) glslang::FinalizeProcess();
    delete sState;
    sState = nullptr;
}

} // namespace prisma3d
