#include "miniz.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <stdio.h>
#include <string.h>

namespace
{

enum Slot
{
    kSpirv,
    kGlsl,
    kEssl300,
    kEssl310,
    kEssl320,
    kEssl320Inner,
    kSlotCount
};

const char* const kSlotOptions[kSlotCount] = { "--spirv", "--glsl", "--essl300", "--essl310",
    "--essl320", "--essl320-inner" };

bool readFile(const char* path, ct::Vector<unsigned char>& out)
{
    FILE* file = fopen(path, "rb");
    if (!file) return false;
    unsigned char chunk[65536];
    for (;;)
    {
        const size_t got = fread(chunk, 1, sizeof(chunk), file);
        for (size_t i = 0; i < got; ++i) out.push_back(chunk[i]);
        if (got < sizeof(chunk)) break;
    }
    fclose(file);
    return true;
}

int usage(const char* reason)
{
    fprintf(stderr, "shaderpack: %s\n", reason);
    fprintf(stderr,
            "usage: prisma3d_shaderpack --name NAME --stage STAGE --spirv FILE [--glsl FILE] "
            "[--essl300 FILE] [--essl310 FILE] [--essl320 FILE] [--essl320-inner FILE] "
            "--bindings FILE --binding-count N --output FILE\n"
            "       prisma3d_shaderpack --sources NAME=FILE... --output FILE.cpp\n");
    return 1;
}

} // namespace

int packSources(int argc, char** argv)
{
    const char* output = nullptr;
    ct::Vector<const char*> names;
    ct::Vector<const char*> files;
    for (int i = 2; i < argc; ++i)
    {
        if (strcmp(argv[i], "--output") == 0 && i + 1 < argc)
        {
            output = argv[++i];
            continue;
        }
        const char* equals = strchr(argv[i], '=');
        if (!equals) return usage(argv[i]);
        names.push_back(argv[i]);
        files.push_back(equals + 1);
    }
    if (!output || names.empty()) return usage("--sources needs NAME=FILE pairs and --output");

    ct::Vector<unsigned char> plain;
    ct::Vector<unsigned> offsets;
    ct::Vector<unsigned> sizes;
    for (size_t i = 0; i < files.size(); ++i)
    {
        ct::Vector<unsigned char> bytes;
        if (!readFile(files[i], bytes))
        {
            fprintf(stderr, "shaderpack: cannot read %s\n", files[i]);
            return 1;
        }
        bytes.push_back(0);
        offsets.push_back(static_cast<unsigned>(plain.size()));
        sizes.push_back(static_cast<unsigned>(bytes.size()));
        for (size_t b = 0; b < bytes.size(); ++b) plain.push_back(bytes[b]);
    }

    mz_ulong packedSize = mz_compressBound(static_cast<mz_ulong>(plain.size()));
    ct::Vector<unsigned char> packed;
    packed.resize(static_cast<size_t>(packedSize));
    if (mz_compress2(packed.data(), &packedSize, plain.data(), static_cast<mz_ulong>(plain.size()),
                MZ_UBER_COMPRESSION) != MZ_OK)
    {
        fprintf(stderr, "shaderpack: compression failed\n");
        return 1;
    }
    packed.resize(static_cast<size_t>(packedSize));

    FILE* file = fopen(output, "wb");
    if (!file)
    {
        fprintf(stderr, "shaderpack: cannot write %s\n", output);
        return 1;
    }
    fprintf(file, "#include \"prisma3d/render/ShaderSources.h\"\n\nnamespace prisma3d\n{\n\n");
    fprintf(file, "const unsigned char kShaderSourcesPacked[] = {\n");
    for (size_t i = 0; i < packed.size(); ++i)
    {
        if (i % 24 == 0) fputs("    ", file);
        fprintf(file, "0x%02x,", packed[i]);
        fputc((i % 24 == 23 || i + 1 == packed.size()) ? '\n' : ' ', file);
    }
    fprintf(file, "};\n\nconst unsigned kShaderSourcesPackedSize = %uu;\n",
            static_cast<unsigned>(packed.size()));
    fprintf(file, "const unsigned kShaderSourcesUnpackedSize = %uu;\n\n",
            static_cast<unsigned>(plain.size()));
    fprintf(file, "const ShaderSourceEntry kShaderSources[] = {\n");
    for (size_t i = 0; i < names.size(); ++i)
    {
        const char* equals = strchr(names[i], '=');
        fprintf(file, "    { \"%.*s\", %uu, %uu },\n", static_cast<int>(equals - names[i]),
                names[i], offsets[i], sizes[i]);
    }
    fprintf(file, "};\n\nconst unsigned kShaderSourceCount = %uu;\n\n} // namespace prisma3d\n",
            static_cast<unsigned>(names.size()));
    fclose(file);
    return 0;
}

int main(int argc, char** argv)
{
    if (argc > 1 && strcmp(argv[1], "--sources") == 0) return packSources(argc, argv);
    const char* name = nullptr;
    const char* stage = nullptr;
    const char* output = nullptr;
    const char* bindings = nullptr;
    const char* bindingCount = "0";
    const char* slots[kSlotCount] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };
    for (int i = 1; i + 1 < argc; i += 2)
    {
        const char* option = argv[i];
        const char* value = argv[i + 1];
        bool known = false;
        for (int s = 0; s < kSlotCount && !known; ++s)
        {
            if (strcmp(option, kSlotOptions[s]) != 0) continue;
            slots[s] = value;
            known = true;
        }
        if (known) continue;
        if (strcmp(option, "--name") == 0) name = value;
        else if (strcmp(option, "--stage") == 0)
            stage = value;
        else if (strcmp(option, "--output") == 0)
            output = value;
        else if (strcmp(option, "--bindings") == 0)
            bindings = value;
        else if (strcmp(option, "--binding-count") == 0)
            bindingCount = value;
        else
            return usage(option);
    }
    if (!name || !stage || !output || !slots[kSpirv]) return usage("missing an argument");

    ct::Vector<unsigned char> plain;
    unsigned sizes[kSlotCount] = { 0, 0, 0, 0, 0, 0 };
    for (int s = 0; s < kSlotCount; ++s)
    {
        if (!slots[s]) continue;
        ct::Vector<unsigned char> bytes;
        if (!readFile(slots[s], bytes))
        {
            fprintf(stderr, "shaderpack: cannot read %s\n", slots[s]);
            return 1;
        }
        if (s != kSpirv) bytes.push_back(0);
        for (size_t i = 0; i < bytes.size(); ++i) plain.push_back(bytes[i]);
        sizes[s] = static_cast<unsigned>(bytes.size());
    }
    if (sizes[kSpirv] == 0 || sizes[kSpirv] % 4 != 0)
    {
        fprintf(stderr, "shaderpack: %s is not a SPIR-V module\n", slots[kSpirv]);
        return 1;
    }

    mz_ulong packedSize = mz_compressBound(static_cast<mz_ulong>(plain.size()));
    ct::Vector<unsigned char> packed;
    packed.resize(static_cast<size_t>(packedSize));
    if (mz_compress2(packed.data(), &packedSize, plain.data(), static_cast<mz_ulong>(plain.size()),
                MZ_UBER_COMPRESSION) != MZ_OK)
    {
        fprintf(stderr, "shaderpack: compression failed for %s\n", name);
        return 1;
    }
    packed.resize(static_cast<size_t>(packedSize));

    ct::Vector<unsigned char> bindingText;
    if (bindings && !readFile(bindings, bindingText))
    {
        fprintf(stderr, "shaderpack: cannot read %s\n", bindings);
        return 1;
    }
    bindingText.push_back(0);

    FILE* file = fopen(output, "wb");
    if (!file)
    {
        fprintf(stderr, "shaderpack: cannot write %s\n", output);
        return 1;
    }
    fprintf(file, "#pragma once\n\n#include \"prisma/rhi/ShaderBlob.h\"\n\n");
    fprintf(file, "static const std::uint8_t %s_packed[] = {\n", name);
    for (size_t i = 0; i < packed.size(); ++i)
    {
        if (i % 24 == 0) fputs("    ", file);
        fprintf(file, "0x%02x,", packed[i]);
        fputc((i % 24 == 23 || i + 1 == packed.size()) ? '\n' : ' ', file);
    }
    fprintf(file, "};\n\n");
    const bool hasBindings = strcmp(bindingCount, "0") != 0;
    if (hasBindings)
        fprintf(file, "static const prisma::ShaderBinding %s_bindings[] = {\n%s};\n\n", name,
                reinterpret_cast<const char*>(bindingText.data()));
    fprintf(file,
            "static const prisma::ShaderBlob %s = { prisma::ShaderStage::%s, nullptr, 0, nullptr, "
            "nullptr,\n    nullptr, nullptr, nullptr, %s%s, %s, %s_packed, sizeof(%s_packed),\n"
            "    { %uu, %uu, %uu, %uu, %uu, %uu } };\n",
            name, stage, hasBindings ? name : "nullptr", hasBindings ? "_bindings" : "",
            bindingCount, name, name, sizes[kSpirv], sizes[kGlsl], sizes[kEssl300], sizes[kEssl310],
            sizes[kEssl320], sizes[kEssl320Inner]);
    fclose(file);
    return 0;
}
