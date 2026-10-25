#include "prisma3d/main/Main.h"
#include "prisma3d/Device.h"
#include "prisma3d/core/FilePack.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/ShaderCompiler.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/script/ZenScripting.h"

#include "platform.h"

#include <ct/hashmap.hpp>
#include <ct/json.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

FILE* sLog = nullptr;

void say(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    if (!sLog) return;
    va_start(args, format);
    vfprintf(sLog, format, args);
    va_end(args);
    fflush(sLog);
}

bool globMatch(const char* pattern, const char* text)
{
    while (*pattern)
    {
        if (*pattern == '*')
        {
            while (*pattern == '*') ++pattern;
            if (!*pattern) return true;
            for (const char* rest = text;; ++rest)
            {
                if (globMatch(pattern, rest)) return true;
                if (!*rest) return false;
            }
        }
        if (!*text || (*pattern != '?' && *pattern != *text)) return false;
        ++pattern;
        ++text;
    }
    return !*text;
}

struct Options
{
    const char* project = nullptr;
    ct::String output;
    ct::String runner;
    ct::String platform;
    ct::String key;
    ct::String log;
    int level = 9;
    bool keepDebug = false;
    bool shaderCache = true;
    bool embed = false;
    bool verbose = false;
};

void usage()
{
    say("export - packs a project into a copy of the runner\n"
        "\n"
        "  export <projecto.json> -o <game> [options]\n"
        "\n"
        "The asset folder, the folders of the listed scenes and the search paths go\n"
        "into the pack; scripts go as bytecode, loaded at start in the packed order.\n"
        "\n"
        "  -o <file>      the executable to write; the pack goes next to it as\n"
        "                 data.rpak\n"
        "  --embed        append the pack to the executable instead (one file)\n"
        "  --platform <p> linux or windows; default: windows when the output ends in .exe,\n"
        "                 otherwise this machine's\n"
        "  --runner <f>   the runner to stamp; default: templates/<platform>/ next to this\n"
        "                 tool, then, for this machine's platform, the runner next to it or\n"
        "                 in ../editor\n"
        "  -k <key>       encrypt the pack with this key (default: the project's\n"
        "                 export/pack_key setting)\n"
        "  -l <0-9>       deflate level, default 9; 0 stores everything raw\n"
        "  --keep-debug   keep line information in the script bytecode (default: the\n"
        "                 project's export/keep_debug setting)\n"
        "  --no-shader-cache  do not run the scenes once to pack their compiled shaders\n"
        "  --log <file>   also write these messages to a file (the editor reads it)\n"
        "\n"
        "Files matching the project's export/exclude patterns (name or path, * and ?)\n"
        "are left out.\n"
        "  -v             print every file as it is added\n");
}

bool parseArguments(int argc, char** argv, Options& options)
{
    for (int i = 1; i < argc; ++i)
    {
        const char* argument = argv[i];
        if (strcmp(argument, "-o") == 0 && i + 1 < argc) options.output = argv[++i];
        else if (strcmp(argument, "--runner") == 0 && i + 1 < argc)
            options.runner = argv[++i];
        else if (strcmp(argument, "--platform") == 0 && i + 1 < argc)
            options.platform = argv[++i];
        else if (strcmp(argument, "-k") == 0 && i + 1 < argc)
            options.key = argv[++i];
        else if (strcmp(argument, "-l") == 0 && i + 1 < argc)
            options.level = atoi(argv[++i]);
        else if (strcmp(argument, "--keep-debug") == 0)
            options.keepDebug = true;
        else if (strcmp(argument, "--no-shader-cache") == 0)
            options.shaderCache = false;
        else if (strcmp(argument, "--embed") == 0)
            options.embed = true;
        else if (strcmp(argument, "--log") == 0 && i + 1 < argc)
            options.log = argv[++i];
        else if (strcmp(argument, "-v") == 0)
            options.verbose = true;
        else if (strcmp(argument, "-h") == 0 || strcmp(argument, "--help") == 0)
            return false;
        else if (argument[0] == '-')
        {
            say("export: unknown option %s\n", argument);
            return false;
        }
        else if (!options.project)
            options.project = argument;
        else
            return false;
    }
    return options.project && !options.output.empty();
}

const char* hostPlatform()
{
#if defined(_WIN32)
    return "windows";
#else
    return "linux";
#endif
}

bool endsWith(const ct::String& text, const char* suffix)
{
    const size_t tail = strlen(suffix);
    return text.size() >= tail && strcmp(text.c_str() + text.size() - tail, suffix) == 0;
}

ct::String findRunner(const ct::String& platform)
{
    char base[4096];
    if (!fs_get_base_path(base, sizeof base)) return ct::String();
    const ct::String app(base);
    const ct::String name = platform == "windows" ? ct::String("runner.exe")
                                                  : ct::String("runner");
    ct::Vector<ct::String> candidates;
    candidates.push_back(app + "templates/" + platform + "/" + name);
    if (platform == hostPlatform())
    {
        candidates.push_back(app + name);
        candidates.push_back(app + "../editor/" + name);
    }
    for (size_t i = 0; i < candidates.size(); ++i)
        if (file_exists(candidates[i].c_str())) return candidates[i];
    return ct::String();
}

ct::String parentDirectory(const ct::String& path)
{
    const size_t slash = path.rfind('/');
    if (slash == ct::String::npos) return ct::String(".");
    return slash == 0 ? ct::String("/") : path.substr(0, slash);
}

bool hiddenComponent(const ct::String& name)
{
    if (name.empty()) return false;
    if (name[0] == '.') return true;
    for (size_t i = 1; i < name.size(); ++i)
        if (name[i - 1] == '/' && name[i] == '.') return true;
    return false;
}

struct Entry
{
    ct::String name;
    ct::String path;
};

struct Collector
{
    const Project* project;
    ct::Vector<Entry> files;
    ct::Vector<Entry> scripts;
    ct::Vector<ct::String> excludes;
    ct::HashMap<ct::String, char> seen;
    std::uint32_t excluded;
    bool verbose;
};

bool excluded(const Collector& collector, const ct::String& name)
{
    const size_t slash = name.rfind('/');
    const char* base = slash == ct::String::npos ? name.c_str() : name.c_str() + slash + 1;
    for (size_t i = 0; i < collector.excludes.size(); ++i)
    {
        const char* pattern = collector.excludes[i].c_str();
        if (globMatch(pattern, base) || globMatch(pattern, name.c_str())) return true;
    }
    return false;
}

bool collectEntry(const char* path, PathType type, void* user)
{
    Collector* collector = static_cast<Collector*>(user);
    if (type != PATH_TYPE_FILE) return true;
    ct::String name;
    if (!collector->project->relativePath(ct::String(path), name))
    {
        say("export: %s is outside the project, skipped\n", path);
        return true;
    }
    if (hiddenComponent(name) || collector->seen.contains(name)) return true;
    collector->seen[name] = 1;
    if (excluded(*collector, name))
    {
        ++collector->excluded;
        if (collector->verbose) say("  %s (excluded)\n", name.c_str());
        return true;
    }
    Entry entry;
    entry.name = name;
    entry.path = path;
    if (endsWith(name, ".py")) collector->scripts.push_back(entry);
    else
        collector->files.push_back(entry);
    return true;
}

bool collectRoot(Collector& collector, const ct::String& root)
{
    if (!dir_exists(root.c_str())) return true;
    if (!fs_enumerate_directory(root.c_str(), true, collectEntry, &collector))
    {
        say("export: could not list %s: %s\n", root.c_str(), platform_get_error());
        return false;
    }
    return true;
}

void sortByName(ct::Vector<Entry>& entries)
{
    for (size_t i = 1; i < entries.size(); ++i)
    {
        Entry pending = static_cast<Entry&&>(entries[i]);
        size_t j = i;
        while (j > 0 && strcmp(entries[j - 1].name.c_str(), pending.name.c_str()) > 0)
        {
            entries[j] = static_cast<Entry&&>(entries[j - 1]);
            --j;
        }
        entries[j] = static_cast<Entry&&>(pending);
    }
}

bool collect(const Project& project, Collector& collector)
{
    collector.project = &project;
    const ct::String patterns = project.get("export/exclude").asString();
    ct::String current;
    for (size_t i = 0; i <= patterns.size(); ++i)
    {
        const char c = i < patterns.size() ? patterns[i] : ';';
        if (c == ';')
        {
            if (!current.empty()) collector.excludes.push_back(current);
            current = ct::String();
        }
        else if (c != ' ' || !current.empty())
            current += c;
    }
    if (!collectRoot(collector, project.assetPath())) return false;
    const ct::Vector<ct::String>& scenes = project.scenes();
    for (size_t i = 0; i < scenes.size(); ++i)
        if (!collectRoot(collector, parentDirectory(project.absolutePath(scenes[i])))) return false;
    const ct::Vector<ct::String> extra = project.resolvedSearchPaths();
    for (size_t i = 0; i < extra.size(); ++i)
        if (!collectRoot(collector, extra[i])) return false;
    sortByName(collector.scripts);
    return true;
}

void warmShaders(const Project& project, int frames)
{
    SceneTree tree;
    if (!Main::start(&tree) || !tree.getRoot())
    {
        say("export: the scene tree could not start; shaders are not pre-compiled\n");
        tree.finalize();
        return;
    }
    const ct::Vector<ct::String>& scenes = project.scenes();
    bool quit = false;
    for (size_t i = 0; i < scenes.size() && !quit; ++i)
    {
        tree.changeSceneToFile(project.absolutePath(scenes[i]));
        for (int frame = 0; frame < frames && !quit; ++frame) quit = Main::iteration();
    }
    tree.finalize();
}

int exportProject(const Options& options, const ct::String& runner)
{
    Project project;
    if (!project.open(ct::String(options.project)))
    {
        say("export: %s\n", project.error().c_str());
        return 1;
    }
    for (size_t i = 0; i < project.warnings().size(); ++i)
        say("export: project: %s\n", project.warnings()[i].c_str());

    DeviceDesc desc;
    desc.headless = true;
    if (!Main::setupProject(project, desc))
    {
        say("export: the engine could not start\n");
        return 1;
    }
    ZenScripting* scripting = ZenScripting::instance();
    if (!scripting)
    {
        say("export: scripting is not available\n");
        Main::cleanup();
        return 1;
    }
    scripting->setHotReload(false);

    const ct::String key =
            options.key.empty() ? project.get("export/pack_key").asString() : options.key;
    const bool stripDebug = !options.keepDebug && !project.get("export/keep_debug").asBool();

    int status = 0;
    Collector collector;
    collector.excluded = 0;
    collector.verbose = options.verbose;
    if (!collect(project, collector)) status = 1;

    FilePackWriter writer;
    writer.setKey(key);
    writer.setCompressionLevel(options.level);
    if (status == 0 && !writer.addFile(ct::String(Project::kManifestName), project.manifestPath()))
        status = 1;
    for (size_t i = 0; status == 0 && i < collector.files.size(); ++i)
    {
        const Entry& entry = collector.files[i];
        if (!writer.addFile(entry.name, entry.path)) status = 1;
        else if (options.verbose)
            say("  %s\n", entry.name.c_str());
    }

    const ct::String temp = options.output + ".script.tmp";
    ct::Json list = ct::Json::array();
    std::uint32_t modules = 0;
    for (size_t i = 0; status == 0 && i < collector.scripts.size(); ++i)
    {
        const Entry& entry = collector.scripts[i];
        ct::String error;
        const ZenScripting::ExportResult result =
                scripting->exportBytecode(entry.name, temp, stripDebug, error);
        if (result == ZenScripting::kExportFailed)
        {
            say("export: %s: %s\n", entry.name.c_str(), error.c_str());
            status = 1;
            break;
        }
        if (result == ZenScripting::kNotNodeScript)
        {
            scripting->clearErrors();
            if (!writer.addFile(entry.name, entry.path)) status = 1;
            ++modules;
            if (options.verbose) say("  %s (module, source)\n", entry.name.c_str());
            continue;
        }
        if (!writer.addFile(entry.name, temp)) status = 1;
        fs_remove_path(temp.c_str());
        list.push_back(ct::Json(entry.name));
        if (options.verbose) say("  %s (bytecode)\n", entry.name.c_str());
    }
    std::uint32_t cached = 0;
    if (status == 0 && options.shaderCache)
    {
        warmShaders(project, 4);
        ct::Vector<ct::String> names;
        shaderCacheFiles(names);
        const ct::String& directory = shaderCacheDirectory();
        for (size_t i = 0; status == 0 && i < names.size(); ++i)
        {
            if (!writer.addFile(ct::String(kShaderCachePackFolder) + "/" + names[i],
                        directory + "/" + names[i]))
                status = 1;
            else
                ++cached;
        }
    }

    if (status == 0)
    {
        char signature[32];
        snprintf(signature, sizeof signature, "%016llx", scripting->getBindingsSignature());
        ct::Json manifest = ct::Json::object();
        manifest["engine"] = ct::Json(ct::String(signature));
        manifest["scripts"] = list;
        const ct::String text = manifest.dump(2);
        if (!writer.addData(ct::String(ZenScripting::kScriptList),
                    reinterpret_cast<const std::uint8_t*>(text.data()), text.size()))
            status = 1;
    }

    const ct::String packPath =
            options.embed ? options.output + ".rpak" : FilePack::sidePackPath(options.output);
    if (status == 0 && !writer.write(packPath)) status = 1;
    if (status == 0)
    {
        if (options.embed)
        {
            if (!FilePack::writeStamp(runner, packPath, key, options.output)) status = 1;
        }
        else if (!FilePack::writeStamp(runner, ct::String(), key, options.output))
            status = 1;
    }
    if ((options.embed || status != 0) && file_exists(packPath.c_str()))
        fs_remove_path(packPath.c_str());

    if (status == 0)
        say("%s: %u files, %llu bytes from %llu, %u scripts as bytecode, %u modules as "
            "source, %u compiled shaders, %u excluded%s%s\n",
                options.embed ? options.output.c_str() : packPath.c_str(), writer.entryCount(),
                static_cast<unsigned long long>(writer.storedBytes()),
                static_cast<unsigned long long>(writer.rawBytes()),
                static_cast<unsigned>(collector.scripts.size()) - modules, modules, cached,
                collector.excluded, key.empty() ? "" : ", encrypted",
                options.embed ? ", embedded" : "");

    project.close();
    Main::cleanup();
    return status;
}

} // namespace

int main(int argc, char** argv)
{
    Options options;
    if (!parseArguments(argc, argv, options))
    {
        usage();
        return 2;
    }

    if (!options.log.empty()) sLog = fopen(options.log.c_str(), "w");

    ct::String platform = options.platform;
    if (platform.empty())
        platform = endsWith(options.output, ".exe") ? ct::String("windows")
                                                    : ct::String(hostPlatform());
    if (platform != "linux" && platform != "windows")
    {
        say("export: unknown platform '%s' (linux or windows)\n", platform.c_str());
        return 2;
    }
    const ct::String runner = options.runner.empty() ? findRunner(platform) : options.runner;
    if (runner.empty() || !file_exists(runner.c_str()))
    {
        say("export: no %s runner to stamp; build it on that platform and put it in "
            "templates/%s/ next to this tool, or pass --runner <file>\n",
                platform.c_str(), platform.c_str());
        return 1;
    }

    say("export: %s runner %s\n", platform.c_str(), runner.c_str());
    const int status = exportProject(options, runner);
    if (sLog) fclose(sLog);
    return status;
}
