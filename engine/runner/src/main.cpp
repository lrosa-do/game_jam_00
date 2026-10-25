#include "prisma3d/main/Main.h"
#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/core/FilePack.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/gui/Gui.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/RenderProfiler.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/script/ZenScripting.h"

#include "platform.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const char* const kSceneSuffix = ".scene.json";

struct Options
{
    const char* path = nullptr;
    int frames = 0;
    bool headless = false;
    bool opengl = false;
    const char* shot = nullptr;
    const char* record = nullptr;
    int fps = 0;
    int width = 0;
    int height = 0;
    int screen = -1;
    bool profile = false;
    float spikes = 0.0f;
    bool physicsDebug = false;
    bool windowed = false;
};

void usage()
{
    fprintf(stderr,
            "Usage: runner <project.json | scene.scene.json> [--frames N] [--headless] "
            "[--size WxH] [--windowed] [--screen N] [--shot file.png] [--record DIR] [--fps N] "
            "[--profile] [--spikes MS] "
            "[--physics-debug] [vulkan | "
            "opengl]\n"
            "  a copy stamped with a pack (pack -s), or with data.rpak next to it, "
            "runs that pack's project when no path is given\n"
            "  Ctrl+X quits at once; Shift+F9 shows the profiler overlay; --profile starts with it "
            "visible and prints "
            "the "
            "averages at exit\n"
            "  --spikes MS logs every frame slower than MS milliseconds with its sections\n"
            "  --record DIR saves every frame as DIR/NNNNN.png (a video with ffmpeg: -framerate "
            "N -i DIR/%%05d.png)\n"
            "  --fps N steps the game by exactly 1/N s a frame whatever the time it takes (with "
            "--record, a video at N fps)\n"
            "  --size and --windowed open a window whatever the project's window mode\n"
            "  --physics-debug starts with the physics shapes and contacts drawn (Shift+F2 toggles "
            "it)\n");
}

bool endsWith(const char* text, const char* suffix)
{
    const size_t length = strlen(text);
    const size_t tail = strlen(suffix);
    return length >= tail && strcmp(text + length - tail, suffix) == 0;
}

bool parseOptions(int argc, char** argv, Options& options, bool stamped)
{
    for (int i = 1; i < argc; ++i)
    {
        const char* argument = argv[i];
        if (strcmp(argument, "--headless") == 0) options.headless = true;
        else if (strcmp(argument, "--windowed") == 0)
            options.windowed = true;
        else if (strcmp(argument, "--profile") == 0)
            options.profile = true;
        else if (strcmp(argument, "vulkan") == 0)
            options.opengl = false;
        else if (strcmp(argument, "opengl") == 0)
            options.opengl = true;
        else if (strcmp(argument, "--frames") == 0)
        {
            if (i + 1 >= argc) return false;
            options.frames = atoi(argv[++i]);
            if (options.frames < 0) return false;
        }
        else if (strcmp(argument, "--physics-debug") == 0)
            options.physicsDebug = true;
        else if (strcmp(argument, "--spikes") == 0)
        {
            if (i + 1 >= argc) return false;
            options.spikes = static_cast<float>(atof(argv[++i]));
            if (options.spikes <= 0.0f) return false;
        }
        else if (strcmp(argument, "--fps") == 0)
        {
            if (i + 1 >= argc) return false;
            options.fps = atoi(argv[++i]);
            if (options.fps <= 0) return false;
        }
        else if (strcmp(argument, "--record") == 0)
        {
            if (i + 1 >= argc) return false;
            options.record = argv[++i];
        }
        else if (strcmp(argument, "--shot") == 0)
        {
            if (i + 1 >= argc) return false;
            options.shot = argv[++i];
        }
        else if (strcmp(argument, "--screen") == 0)
        {
            if (i + 1 >= argc) return false;
            options.screen = atoi(argv[++i]);
            if (options.screen < 0) return false;
        }
        else if (strcmp(argument, "--size") == 0)
        {
            if (i + 1 >= argc) return false;
            if (sscanf(argv[++i], "%dx%d", &options.width, &options.height) != 2 ||
                    options.width <= 0 || options.height <= 0)
                return false;
        }
        else if (argument[0] == '-')
        {
            fprintf(stderr, "runner: unknown option '%s'\n", argument);
            return false;
        }
        else if (!options.path)
            options.path = argument;
        else
            return false;
    }
    return options.path != nullptr || stamped;
}

ct::String parentDirectory(const ct::String& path)
{
    const size_t slash = path.rfind('/');
    if (slash == ct::String::npos) return ct::String(".");
    return slash == 0 ? ct::String("/") : path.substr(0, slash);
}

ct::String absolutePath(const char* path)
{
    char buffer[4096];
    if (path_absolute(buffer, sizeof buffer, path)) return ct::String(buffer);
    return ct::String(path);
}

bool fileContains(const ct::String& path, const char* needle)
{
    FILE* file = fopen(path.c_str(), "rb");
    if (!file) return false;
    char buffer[512];
    const size_t got = fread(buffer, 1, sizeof buffer - 1, file);
    fclose(file);
    buffer[got] = 0;
    return strstr(buffer, needle) != nullptr;
}

ct::String findProject(const ct::String& scenePath)
{
    ct::String directory = parentDirectory(scenePath);
    for (;;)
    {
        DirList list;
        ct::String found;
        if (dir_list(directory.c_str(), &list))
        {
            for (int i = 0; i < list.count && found.empty(); ++i)
            {
                const char* entry = list.paths[i];
                if (endsWith(entry, ".json") && !endsWith(entry, kSceneSuffix) &&
                        fileContains(ct::String(entry), "prisma3d-project"))
                    found = ct::String(entry);
            }
            dir_list_free(&list);
        }
        if (!found.empty()) return found;
        const ct::String parent = parentDirectory(directory);
        if (parent == directory || directory == "/" || directory == ".") break;
        directory = parent;
    }
    return ct::String();
}

void report(const SceneResult& result)
{
    if (!result.diagnostics.empty()) log_warn("runner: %s", result.describe().c_str());
}

class ProfilerOverlay : public Node
{
public:
    static ProfilerOverlay* create(bool visible)
    {
        return new ProfilerOverlay(visible);
    }

protected:
    void notificationHandler(int what) override
    {
        Node::notificationHandler(what);
        if (what != kNotificationProcess) return;
        Device* device = Main::device();
        if (device && device->input().isKeyJustPressed(Key::F9) &&
                (device->input().isKeyPressed(Key::LeftShift) ||
                        device->input().isKeyPressed(Key::RightShift)))
        {
            mVisible = !mVisible;
        }
        if (device && device->input().isKeyJustPressed(Key::X) &&
                (device->input().isKeyPressed(Key::LeftControl) ||
                        device->input().isKeyPressed(Key::RightControl)))
            device->requestClose();
        if (mVisible) draw();
    }

private:
    static void line(Gui* gui, float x, float& y, const char* text, const ig::Color& color)
    {
        gui->hudText(ig::StringView(text, strlen(text)), x, y, color, 15.0f);
        y += 17.0f;
    }

    static void samples(Gui* gui, float x, float& y, const ProfileSample* items,
            std::uint32_t count, const ig::Color& color)
    {
        char text[128];
        for (std::uint32_t i = 0; i < count; ++i)
        {
            const ProfileSample& sample = items[i];
            snprintf(text, sizeof text, "%*s%-20s %6.2f  max %6.2f",
                    static_cast<int>(sample.depth * 2), "", sample.name ? sample.name : "?",
                    sample.display, sample.maximum);
            line(gui, x, y, text, color);
        }
    }

    void draw() const
    {
        Gui* gui = Main::gui();
        RenderProfiler* profiler = Main::profiler();
        if (!gui || !gui->frameOpen() || !profiler) return;
        profiler->request();
        const std::uint32_t rows = 2 + profiler->cpuSampleCount() +
                                   (profiler->gpuAvailable() ? profiler->gpuSampleCount() : 1);
        const float x = 10.0f;
        float y = 10.0f;
        gui->hudRectFilled(ig::Rect(x - 6.0f, y - 6.0f, 400.0f, rows * 17.0f + 12.0f),
                ig::Color(0, 0, 0, 160));
        char text[128];
        snprintf(text, sizeof text, "%.0f fps   cpu %.2f ms   gpu %.2f ms", profiler->fps(),
                profiler->cpuFrameMilliseconds(), profiler->gpuFrameMilliseconds());
        line(gui, x, y, text, ig::Color(255, 220, 120, 255));
        line(gui, x, y, "cpu", ig::Color(180, 180, 180, 255));
        samples(gui, x, y, profiler->cpuSamples(), profiler->cpuSampleCount(),
                ig::Color(255, 255, 255, 255));
        if (profiler->gpuAvailable())
            samples(gui, x, y, profiler->gpuSamples(), profiler->gpuSampleCount(),
                    ig::Color(140, 220, 255, 255));
        else
            line(gui, x, y, "gpu timers are not available", ig::Color(255, 120, 120, 255));
    }

    explicit ProfilerOverlay(bool visible)
            : mVisible(visible)
    {
        setName("ProfilerOverlay");
        setProcess(true);
    }

    bool mVisible;
};

void printSamples(const char* kind, const ProfileSample* samples, std::uint32_t count)
{
    for (std::uint32_t i = 0; i < count; ++i)
    {
        const ProfileSample& sample = samples[i];
        fprintf(stderr, "  %s %*s%-28s %7.2f ms  max %7.2f\n", kind,
                static_cast<int>(sample.depth * 2), "", sample.name ? sample.name : "?",
                sample.average, sample.maximum);
    }
}

void printProfile()
{
    RenderProfiler* profiler = Main::profiler();
    if (!profiler) return;
    fprintf(stderr, "profile: frame cpu %.2f ms, gpu %.2f ms, %.0f fps\n",
            profiler->cpuFrameMilliseconds(), profiler->gpuFrameMilliseconds(), profiler->fps());
    if (RenderingServer* rendering = Main::rendering())
    {
        const RenderStats& stats = rendering->getStats();
        fprintf(stderr,
                "  draws %u, instances %u, triangles %u, culled %u, shadow draws %u, "
                "batch draws %u\n",
                stats.drawCalls, stats.instancesDrawn, stats.triangles, stats.instancesCulled,
                stats.shadowDrawCalls, stats.batchDrawCalls);
    }
    if (PhysicsServer3D* physics = Main::physics())
    {
        const PhysicsStats& stats = physics->getStats();
        fprintf(stderr,
                "  physics: %u bodies (%u awake, %u static, %u kinematic), %u shapes, "
                "%u joints, %u contacts, step %.2f ms (avg %.2f, max %.2f)\n",
                stats.bodies, stats.awakeBodies, stats.staticBodies, stats.kinematicBodies,
                stats.shapes, stats.joints, stats.contacts, stats.stepMs, stats.averageStepMs,
                stats.maxStepMs);
    }
    printSamples("cpu", profiler->cpuSamples(), profiler->cpuSampleCount());
    if (profiler->gpuAvailable())
        printSamples("gpu", profiler->gpuSamples(), profiler->gpuSampleCount());
    else
        fprintf(stderr, "  gpu timers are not available\n");
}

} // namespace

int main(int argc, char** argv)
{
    char executableBuffer[4096];
    ct::String executable;
    PackStamp stamp;
    if (fs_get_executable_path(executableBuffer, sizeof executableBuffer))
        executable = ct::String(executableBuffer);
    const bool stamped = !executable.empty() && FilePack::readStamp(executable, stamp);
    const ct::String sidePack =
            executable.empty() ? ct::String() : FilePack::sidePackPath(executable);
    const bool embedded = stamped && stamp.size > 0;
    const bool beside = !embedded && !sidePack.empty() && file_exists(sidePack.c_str());

    Options options;
    if (!parseOptions(argc, argv, options, stamped || beside))
    {
        usage();
        return 2;
    }
    const bool player = (embedded || beside) && !options.path;
    if (stamped && !embedded && !beside && !options.path)
    {
        fprintf(stderr, "runner: %s was not found next to %s\n", FilePack::kSidePackName,
                executable.c_str());
        return 1;
    }

    DeviceDesc base;
    base.driver = options.opengl ? prisma::DriverType::OpenGL : prisma::DriverType::Vulkan;
    base.headless = options.headless;
    if (options.width > 0)
    {
        base.width = options.width;
        base.height = options.height;
    }
    if (options.screen >= 0) base.screen = options.screen;

    const bool sceneOnly = !player && endsWith(options.path, kSceneSuffix);
    Project project;
    ct::String scenePath;
    ct::String manifest;
    if (player)
    {
        FileSystem& files = FileSystem::getSingleton();
        const bool mounted = embedded ? files.mountPack(executable, stamp.key, stamp.offset)
                                      : files.mountPack(sidePack, stamp.key);
        if (!mounted) return 1;
        if (!project.openPacked(ct::String(Project::kManifestName)))
        {
            fprintf(stderr, "runner: %s\n", project.error().c_str());
            return 1;
        }
    }
    else
    {
        manifest = absolutePath(options.path);
        if (sceneOnly)
        {
            scenePath = manifest;
            if (!file_exists(scenePath.c_str()))
            {
                fprintf(stderr, "runner: the scene '%s' does not exist\n", scenePath.c_str());
                return 1;
            }
            manifest = findProject(scenePath);
        }
        if (!manifest.empty() && !project.open(manifest))
        {
            fprintf(stderr, "runner: %s\n", project.error().c_str());
            return 1;
        }
    }
    if (!project.isOpen())
    {
        base.title = "prisma3d runner";
        if (!Main::setup(base)) return 1;
    }
    else
    {
        for (size_t i = 0; i < project.warnings().size(); ++i)
            log_warn("runner: project: %s", project.warnings()[i].c_str());
        // A project set to full screen (an exported game) still opens a window when asked to.
        if (options.windowed || options.width > 0)
            project.set("display/window/size/mode", PropertyValue::fromInt(0));
        if (options.width > 0)
        {
            project.set("display/window/size/window_width_override",
                    PropertyValue::fromInt(options.width));
            project.set("display/window/size/window_height_override",
                    PropertyValue::fromInt(options.height));
        }
        if (options.screen >= 0)
        {
            project.set("display/window/size/initial_position_type", PropertyValue::fromInt(3));
            project.set("display/window/size/initial_screen",
                    PropertyValue::fromInt(options.screen));
        }
        if (!Main::setupProject(project, base)) return 1;
        if (sceneOnly) log_info("runner: project %s", manifest.c_str());
        if (player && ZenScripting::instance())
            ZenScripting::instance()->preloadList(ct::String(ZenScripting::kScriptList));
    }
    if (sceneOnly) FileSystem::getSingleton().addSearchPath(parentDirectory(scenePath));

    if (Engine* engine = Engine::singleton()) engine->setEditorHint(false);
    if (options.profile && Main::profiler()) Main::profiler()->setEnabled(true);
    if (options.physicsDebug) Main::setPhysicsDebug(true);
    if (options.spikes > 0.0f && Main::profiler())
    {
        Main::profiler()->setEnabled(true);
        Main::profiler()->setSpikeThreshold(options.spikes);
    }

    SceneTree tree;
    if (!Main::start(&tree) || !tree.getRoot())
    {
        tree.finalize();
        project.close();
        Main::cleanup();
        return 1;
    }

    SceneResult result;
    Node* scene = nullptr;
    if (sceneOnly)
    {
        if (Window* window = tree.getRoot()->castTo<Window>())
            window->setClearColor(Math::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
        scene = SceneSerializer::loadFile(scenePath, tree.getRoot(), result);
        if (scene) tree.setCurrentScene(scene, scenePath);
    }
    else
        scene = Main::loadMainScene(result);

    if (!scene || !result.success())
    {
        fprintf(stderr, "runner: could not load the scene:\n%s\n", result.describe().c_str());
        tree.finalize();
        project.close();
        Main::cleanup();
        return 1;
    }
    report(result);

    if (Window* window = tree.getRoot()->castTo<Window>())
        if (!window->getCamera3D()) log_warn("runner: the scene has no camera");
    tree.getRoot()->addChild(ProfilerOverlay::create(options.profile));

    if (options.fps > 0 && Main::engine()) Main::engine()->setFixedFps(options.fps);
    int frame = 0;
    int status = 0;
    for (;;)
    {
        const bool last = options.frames > 0 && frame + 1 >= options.frames;
        if (last && options.shot) Main::device()->requestScreenshot(options.shot);
        char recordPath[4096];
        if (options.record && !(last && options.shot))
        {
            snprintf(recordPath, sizeof recordPath, "%s/%05d.png", options.record, frame);
            Main::device()->requestScreenshot(recordPath);
        }
        if (Main::iteration()) break;
        if (options.frames > 0 && ++frame >= options.frames)
        {
            if (options.shot && !Main::device()->screenshotSaved())
            {
                fprintf(stderr, "runner: could not save '%s'\n", options.shot);
                status = 1;
            }
            break;
        }
    }

    if (options.profile) printProfile();
    if (status == 0) status = tree.getExitCode();
    tree.finalize();
    project.close();
    Main::cleanup();
    return status;
}
