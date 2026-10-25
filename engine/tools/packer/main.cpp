#include "prisma3d/core/FilePack.h"

#include "platform.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

struct Root
{
    ct::String path;
    ct::String prefix;
};

struct Options
{
    ct::String output;
    ct::String source;
    ct::String key;
    ct::String stamp;
    ct::Vector<Root> roots;
    int level = 9;
    bool list = false;
    bool verbose = false;
};

void usage()
{
    printf("pack - builds and inspects .rpak archives\n"
           "\n"
           "  pack -o <out.rpak> [options] <path>...\n"
           "  pack -t <pack.rpak | stamped executable> [-k <key>]\n"
           "  pack -s <executable> -o <out> <pack.rpak> [-k <key>]\n"
           "\n"
           "Each <path> is a root: files under it are named relative to it, so\n"
           "  pack -o shaders.rpak assets/shaders\n"
           "stores lit.frag - the same name the engine asks FileSystem for once\n"
           "that folder is a search path. -p puts a folder back in front of the\n"
           "names that follow, for a root packed from deeper than it is asked for:\n"
           "  pack -o d.rpak assets/shaders -p lensflare assets/textures/lensflare\n"
           "\n"
           "  -o <file>   pack to write\n"
           "  -t <file>   list an existing pack, or the pack of a stamped executable\n"
           "  -c <file>   also write the pack as a C++ source to compile in\n"
           "  -s <file>   write a copy of this executable with the pack appended; the\n"
           "              key is stored with it, and -t on the copy lists the pack\n"
           "  -k <key>    encrypt with this key (omit for a plain pack)\n"
           "  -p <name>   prefix the names of every root after this one\n"
           "  -l <0-9>    deflate level, default 9; 0 stores everything raw\n"
           "  -v          print every entry as it is added\n");
}

bool parseArguments(int argc, char** argv, Options& options)
{
    ct::String prefix;
    for (int i = 1; i < argc; ++i)
    {
        const char* argument = argv[i];
        if (strcmp(argument, "-o") == 0 && i + 1 < argc) options.output = argv[++i];
        else if (strcmp(argument, "-t") == 0 && i + 1 < argc)
        {
            options.output = argv[++i];
            options.list = true;
        }
        else if (strcmp(argument, "-c") == 0 && i + 1 < argc)
            options.source = argv[++i];
        else if (strcmp(argument, "-s") == 0 && i + 1 < argc)
            options.stamp = argv[++i];
        else if (strcmp(argument, "-k") == 0 && i + 1 < argc)
            options.key = argv[++i];
        else if (strcmp(argument, "-p") == 0 && i + 1 < argc)
            prefix = argv[++i];
        else if (strcmp(argument, "-l") == 0 && i + 1 < argc)
            options.level = atoi(argv[++i]);
        else if (strcmp(argument, "-v") == 0)
            options.verbose = true;
        else if (strcmp(argument, "-h") == 0 || strcmp(argument, "--help") == 0)
            return false;
        else if (argument[0] == '-')
        {
            printf("pack: unknown option %s\n", argument);
            return false;
        }
        else
        {
            Root root;
            root.path = argument;
            root.prefix = prefix;
            options.roots.push_back(root);
        }
    }

    if (options.output.empty()) return false;
    if (!options.list && options.roots.empty()) return false;
    if (!options.stamp.empty() && (options.list || options.roots.size() != 1)) return false;
    return true;
}

ct::String joinRelative(const ct::String& prefix, const ct::String& name)
{
    if (prefix.empty()) return name;
    return prefix + "/" + name;
}

ct::String stripTrailingSlashes(const ct::String& path)
{
    ct::String out(path);
    while (out.size() > 1 && (out[out.size() - 1] == '/' || out[out.size() - 1] == '\\'))
        out.resize(out.size() - 1);
    return out;
}

struct Walk
{
    FilePackWriter* writer;
    ct::String root;
    ct::String prefix;
    bool verbose;
    std::uint32_t added;
    bool ok;
};

bool addEntry(const char* path, PathType type, void* user)
{
    Walk* walk = static_cast<Walk*>(user);
    if (type != PATH_TYPE_FILE) return true;

    const std::size_t rootLength = walk->root.size();
    if (strncmp(path, walk->root.c_str(), rootLength) != 0 ||
            (path[rootLength] != '/' && path[rootLength] != '\\'))
    {
        printf("pack: %s is not under %s\n", path, walk->root.c_str());
        walk->ok = false;
        return false;
    }

    const ct::String relative(path + rootLength + 1);
    const ct::String name = joinRelative(walk->prefix, relative);
    if (!walk->writer->addFile(name, ct::String(path)))
    {
        walk->ok = false;
        return false;
    }
    ++walk->added;
    if (walk->verbose)
    {
        PathInfo info = {};
        fs_get_path_info(path, &info);
        printf("  %s (%lld bytes)\n", name.c_str(), static_cast<long long>(info.size));
    }
    return true;
}

bool addDirectory(FilePackWriter& writer, const ct::String& root, const ct::String& prefix,
        bool verbose, std::uint32_t& added)
{
    Walk walk;
    walk.writer = &writer;
    walk.root = stripTrailingSlashes(root);
    walk.prefix = prefix;
    walk.verbose = verbose;
    walk.added = 0;
    walk.ok = true;
    if (!fs_enumerate_directory(walk.root.c_str(), true, addEntry, &walk))
    {
        printf("pack: could not list %s: %s\n", walk.root.c_str(), platform_get_error());
        return false;
    }
    added += walk.added;
    return walk.ok;
}

bool writeSource(const ct::String& packPath, const ct::String& sourcePath, const ct::String& key)
{
    std::size_t size = 0;
    std::uint8_t* bytes = file_read(packPath.c_str(), &size);
    if (!bytes)
    {
        printf("pack: could not read back %s\n", packPath.c_str());
        return false;
    }

    IoStream* output = io_open_file(sourcePath.c_str(), "wb");
    if (!output)
    {
        printf("pack: could not create %s\n", sourcePath.c_str());
        fs_free(bytes);
        return false;
    }

    ct::String text;
    text += "// Generated by pack from ";
    text += packPath;
    text += ". Do not edit.\n\n";
    text += "namespace prisma3d\n{\nnamespace Generated\n{\n\n";
    text += "extern const unsigned char kDefaultPackData[];\n";
    text += "extern const unsigned long long kDefaultPackSize;\n";
    text += "extern const char* const kDefaultPackKey;\n\n";
    text += "const unsigned char kDefaultPackData[] = {";

    char buffer[64];
    for (std::size_t i = 0; i < size; ++i)
    {
        if (i % 16 == 0) text += "\n    ";
        snprintf(buffer, sizeof(buffer), "0x%02x,", bytes[i]);
        text += buffer;
    }
    text += "\n};\n\n";

    snprintf(buffer, sizeof(buffer), "const unsigned long long kDefaultPackSize = %lluull;\n",
            static_cast<unsigned long long>(size));
    text += buffer;

    text += "const char* const kDefaultPackKey = \"";
    for (std::size_t i = 0; i < key.size(); ++i)
    {
        snprintf(buffer, sizeof(buffer), "\\x%02x", static_cast<unsigned char>(key[i]));
        text += buffer;
    }
    text += "\";\n\n";
    text += "} // namespace Generated\n} // namespace prisma3d\n";

    const bool ok = io_write(output, text.data(), text.size()) == text.size();
    const bool closed = io_close(output);
    fs_free(bytes);
    if (!ok || !closed)
    {
        printf("pack: could not write %s\n", sourcePath.c_str());
        return false;
    }

    printf("%s: %llu bytes embedded\n", sourcePath.c_str(), static_cast<unsigned long long>(size));
    return true;
}

int listPack(const Options& options)
{
    FilePack pack;
    PackStamp stamp;
    if (FilePack::readStamp(options.output, stamp) && stamp.size > 0)
    {
        printf("stamped executable: pack at %llu, %llu bytes%s\n",
                static_cast<unsigned long long>(stamp.offset),
                static_cast<unsigned long long>(stamp.size),
                stamp.key.empty() ? "" : ", key stored");
        if (!pack.open(options.output, options.key.empty() ? stamp.key : options.key, stamp.offset))
            return 1;
    }
    else if (stamp.size == 0 && stamp.offset > 0)
    {
        const ct::String side = FilePack::sidePackPath(options.output);
        printf("stamped executable%s: the pack is %s next to it\n",
                stamp.key.empty() ? "" : " with a key", FilePack::kSidePackName);
        if (!pack.open(side, options.key.empty() ? stamp.key : options.key)) return 1;
    }
    else if (!pack.open(options.output, options.key))
        return 1;

    std::uint64_t raw = 0;
    std::uint64_t stored = 0;
    for (std::uint32_t i = 0; i < pack.entryCount(); ++i)
    {
        raw += pack.entrySizeRaw(i);
        stored += pack.entrySizeStored(i);
        printf("%10u  %10u  %s\n", pack.entrySizeRaw(i), pack.entrySizeStored(i),
                pack.entryName(i).c_str());
    }

    printf("%u entries, %llu bytes stored from %llu\n", pack.entryCount(),
            static_cast<unsigned long long>(stored), static_cast<unsigned long long>(raw));
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    Options options;
    if (!parseArguments(argc, argv, options))
    {
        usage();
        return 1;
    }

    if (options.list) return listPack(options);

    if (!options.stamp.empty())
    {
        const ct::String& pack = options.roots[0].path;
        if (!FilePack::writeStamp(options.stamp, pack, options.key, options.output)) return 1;
        printf("%s: %s with %s appended (%lld bytes)%s\n", options.output.c_str(),
                options.stamp.c_str(), pack.c_str(),
                static_cast<long long>(file_size(pack.c_str())),
                options.key.empty() ? "" : ", key stored");
        return 0;
    }

    FilePackWriter writer;
    writer.setKey(options.key);
    writer.setCompressionLevel(options.level);

    std::uint32_t added = 0;
    for (std::size_t i = 0; i < options.roots.size(); ++i)
    {
        const Root& root = options.roots[i];
        if (dir_exists(root.path.c_str()))
        {
            if (!addDirectory(writer, root.path, root.prefix, options.verbose, added)) return 1;
        }
        else
        {
            const ct::String name =
                    joinRelative(root.prefix, ct::String(path_filename(root.path.c_str())));
            if (!writer.addFile(name, root.path)) return 1;
            ++added;
            if (options.verbose) printf("  %s\n", name.c_str());
        }
    }

    if (!writer.write(options.output)) return 1;

    if (!options.source.empty() && !writeSource(options.output, options.source, options.key))
        return 1;

    printf("%s: %u entries, %llu bytes from %llu (%.1f%%)%s\n", options.output.c_str(), added,
            static_cast<unsigned long long>(writer.storedBytes()),
            static_cast<unsigned long long>(writer.rawBytes()),
            writer.rawBytes() ? 100.0 * static_cast<double>(writer.storedBytes()) /
                                        static_cast<double>(writer.rawBytes())
                              : 0.0,
            options.key.empty() ? "" : ", encrypted");
    return 0;
}
