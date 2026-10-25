#include "prisma3d/main/Main.h"

#include "prisma3d/render/ShaderCompiler.h"
#include "prisma3d/render/ShaderVariants.h"

#include "prisma3d/audio/AudioServer.h"
#include "prisma3d/gui/Gui.h"
#include "prisma3d/import/B3DImporter.h"
#include "prisma3d/import/C3DSImporter.h"
#include "prisma3d/import/FbxImporter.h"
#include "prisma3d/import/GltfImporter.h"
#include "prisma3d/import/MS3DImporter.h"
#include "prisma3d/import/ModelImporter.h"
#include "prisma3d/import/ObjImporter.h"
#include "prisma3d/import/OgreMeshImporter.h"
#include "prisma3d/import/PMeshImporter.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/main/StatsOverlay.h"
#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/render/Batch.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderProfiler.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/SceneTypes.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/script/ZenScripting.h"

#include "prisma3d/core/FileSystem.h"

#include "platform.h"

namespace prisma3d
{

namespace
{

Device* sDevice = nullptr;
Engine* sEngine = nullptr;
SceneTree* sTree = nullptr;
MeshStore* sMeshes = nullptr;
TextureStore* sTextures = nullptr;
MaterialStore* sMaterials = nullptr;
RenderingServer* sRendering = nullptr;
AssetLoader* sAssets = nullptr;
AudioServer* sAudio = nullptr;
PhysicsServer3D* sPhysics = nullptr;
ModelLoader* sModels = nullptr;
RenderProfiler* sProfiler = nullptr;
Gui* sGui = nullptr;
Project* sProject = nullptr;
StatsOverlay* sStats = nullptr;
ZenScripting* sScripting = nullptr;

TextureId sTransitionImage;

void drawSceneTransition(Gui* gui, SceneTree* tree);
bool sPhysicsDebug = false;
int sOverlayWidth = 0;
int sOverlayHeight = 0;
std::uint64_t sLastTicks = 0;
std::uint64_t sSetupTicks = 0;
std::uint64_t sFramesPresented = 0;
bool sStarted = false;
MainListener* sListener = nullptr;

void dispatchInput(const InputEvent& event, void* user)
{
    (void) user;
    if (event.type == InputEventType::Key && event.key.key == Key::F1 &&
            (event.key.modifiers & kModShift))
    {
        if (event.key.pressed && !event.key.echo && sStats) sStats->toggle();
        return;
    }
    if (event.type == InputEventType::Key && event.key.key == Key::F2 &&
            (event.key.modifiers & kModShift))
    {
        if (event.key.pressed && !event.key.echo) sPhysicsDebug = !sPhysicsDebug;
        return;
    }
    if (event.type == InputEventType::Key && event.key.key == Key::F12)
    {
        if (event.key.pressed && !event.key.echo && sDevice)
        {
            if (event.key.modifiers & kModControl)
            {
                if (sDevice->isGifRecording()) sDevice->gifRecordingStop();
                else
                    sDevice->gifRecordingStart();
            }
            else
            {
                sDevice->takeScreenshot();
            }
        }
        return;
    }
    if (sGui) sGui->handleEvent(event);
    if (sTree && (!sListener || sListener->forwardsToScene(event))) sTree->inputEvent(event);
    if (sListener) sListener->inputEvent(event);
}

} // namespace

bool Main::setup(const DeviceDesc& desc)
{
    if (sDevice) return false;

    const std::uint64_t setupStart = time_nanos();
    sDevice = Device::create(desc);
    if (!sDevice) return false;
    // platform_init rebases the clock, so a stamp taken before it is only valid when headless.
    sSetupTicks = time_nanos() < setupStart ? 0 : setupStart;
    sFramesPresented = 0;

    std::uint64_t stamp = time_nanos();
    const auto note = [&stamp](const char* what)
    {
        const std::uint64_t now = time_nanos();
        const double milliseconds = static_cast<double>(now - stamp) * 1e-6;
        stamp = now;
        if (milliseconds >= 1.0) log_info("main: %s ready in %.1f ms", what, milliseconds);
    };
    note("device");

    registerSceneTypes();
    note("scene types");

    sAudio = new AudioServer();
    AudioServerDesc audioDesc;
    audioDesc.nullBackend = desc.headless;
    sAudio->init(audioDesc);
    note("audio");

    sPhysics = new PhysicsServer3D();
    note("physics");

    sScripting = new ZenScripting();
    note("scripting");

    prisma::Driver* driver = sDevice->driver();
    sProfiler = new RenderProfiler(driver);
    sMeshes = new MeshStore(driver);
    sTextures = new TextureStore(driver);
    sMaterials = new MaterialStore(*sTextures);
    sRendering = new RenderingServer(driver, *sMeshes, *sTextures, *sMaterials);
    note("rendering server");
    sModels = new ModelLoader();
    sModels->addImporter(new GltfImporter());
    sModels->addImporter(new OgreMeshImporter());
    sModels->addImporter(new B3DImporter());
    sModels->addImporter(new MS3DImporter());
    sModels->addImporter(new ObjImporter());
    sModels->addImporter(new C3DSImporter());
    sModels->addImporter(new FbxImporter());
    sModels->addImporter(new PMeshImporter());
    sAssets = new AssetLoader(*sTextures, *sMaterials, *sMeshes, *sModels, 0);
    note("asset loader");

    sStats = new StatsOverlay();
    sGui = new Gui(*sDevice, *sTextures);
    if (!sGui->init())
    {
        log_error("gui: initialisation failed, running without the interface");
        delete sGui;
        sGui = nullptr;
    }
    note("gui");
    sDevice->input().setEventDispatchFunction(&dispatchInput, nullptr);
    sEngine = new Engine();
    sEngine->setDrawFunction(&Main::drawFrame, nullptr);
    note("engine");
    return true;
}

bool Main::setupProject(Project& project, const DeviceDesc& base)
{
    if (sDevice || !project.isOpen()) return false;

    DeviceDesc desc = base;
    project.fillDeviceDesc(desc);
    if (!setup(desc)) return false;

    sProject = &project;
    project.registerSearchPaths();
    char userBuffer[4096];
    if (fs_get_pref_path(userBuffer, sizeof userBuffer, "prisma3d", project.name().c_str()))
    {
        ct::String folder(userBuffer);
        while (folder.size() > 1 && folder[folder.size() - 1] == '/')
            folder.resize(folder.size() - 1);
        FileSystem::getSingleton().setUserDirectory(folder);
        log_info("main: user folder %s", folder.c_str());
    }
    else
        log_warn("main: no user folder for '%s': %s", project.name().c_str(), platform_get_error());
    project.applyToEngine(*sEngine);
    project.applyToPhysics(*sPhysics);
    project.applyToInputMap(sDevice->input().map());
    project.applyToRendering(*sRendering);
    return true;
}

bool Main::start(SceneTree* tree)
{
    if (!sDevice || !sEngine || !tree || sStarted) return false;

    sTree = tree;
    sEngine->setMainLoop(tree);
    tree->setProcessHook(
            []
            {
                if (sGui && sGui->frameOpen() && sTree) drawSceneTransition(sGui, sTree);
            });
    sLastTicks = sDevice->getTicksUsec();
    sEngine->start(sLastTicks);
    if (sProject && tree->getRoot())
        if (Window* window = tree->getRoot()->castTo<Window>()) sProject->applyToViewport(*window);
    sStarted = true;
    return true;
}

Node* Main::loadMainScene(SceneResult& result)
{
    if (!sProject)
    {
        result.addError("", "the engine was not set up from a project");
        return nullptr;
    }
    if (!sTree || !sTree->getRoot())
    {
        result.addError("", "the scene tree is not running");
        return nullptr;
    }
    Node* scene = sProject->loadMainScene(sTree->getRoot(), result);
    if (scene) sTree->setCurrentScene(scene, sProject->absolutePath(sProject->mainScene()));
    return scene;
}

namespace
{

void drawSceneTransition(Gui* gui, SceneTree* tree)
{
    const SceneTree::Transition effect = tree->getTransition();
    if (effect == SceneTree::Transition::None)
    {
        if (sTransitionImage.valid() && sTextures) sTextures->destroy(sTransitionImage);
        sTransitionImage = TextureId();
        return;
    }
    ct::Vector<std::uint8_t> pixels;
    int width = 0;
    int height = 0;
    if (tree->takeTransitionPixels(pixels, width, height) && sTextures && width > 0 && height > 0)
    {
        if (sTransitionImage.valid()) sTextures->destroy(sTransitionImage);
        sTransitionImage = sTextures->createEmpty(static_cast<std::uint32_t>(width),
                static_cast<std::uint32_t>(height), true, false, "scene transition");
        if (sTransitionImage.valid())
        {
            sTextures->uploadRows(sTransitionImage, 0, static_cast<std::uint32_t>(height),
                    pixels.data());
            sTextures->finish(sTransitionImage);
        }
    }
    if (!sTransitionImage.valid()) return;
    const float progress = static_cast<float>(tree->getTransitionProgress());
    const float t = progress * progress * (3.0f - 2.0f * progress);
    const ig::Rect hud = gui->hudRect();
    const float hudWidth = hud.width;
    const float hudHeight = hud.height;
    float x = 0.0f;
    float y = 0.0f;
    float drawWidth = hudWidth;
    float drawHeight = hudHeight;
    float u0 = 0.0f;
    float u1 = 1.0f;
    float alpha = 1.0f;
    switch (effect)
    {
        case SceneTree::Transition::Fade:
            alpha = 1.0f - t;
            break;
        case SceneTree::Transition::SlideLeft:
            x -= hudWidth * t;
            break;
        case SceneTree::Transition::SlideRight:
            x += hudWidth * t;
            break;
        case SceneTree::Transition::SlideUp:
            y -= hudHeight * t;
            break;
        case SceneTree::Transition::SlideDown:
            y += hudHeight * t;
            break;
        case SceneTree::Transition::ZoomIn:
        case SceneTree::Transition::ZoomOut:
        {
            const float scale =
                    effect == SceneTree::Transition::ZoomIn ? 1.0f + 0.8f * t : 1.0f - 0.8f * t;
            drawWidth = hudWidth * scale;
            drawHeight = hudHeight * scale;
            x = (hudWidth - drawWidth) * 0.5f;
            y = (hudHeight - drawHeight) * 0.5f;
            alpha = 1.0f - t;
            break;
        }
        case SceneTree::Transition::WipeLeft:
            x = hudWidth * t;
            drawWidth = hudWidth * (1.0f - t);
            u0 = t;
            break;
        case SceneTree::Transition::WipeRight:
            drawWidth = hudWidth * (1.0f - t);
            u1 = 1.0f - t;
            break;
        case SceneTree::Transition::None:
            break;
    }
    if (drawWidth <= 0.0f || drawHeight <= 0.0f || alpha <= 0.0f) return;
    gui->hudImage(sTransitionImage, ig::Rect(x, y, drawWidth, drawHeight), 0.0f, ig::Vec2(u0, 0.0f),
            ig::Vec2(u1, 1.0f), ig::Color(255, 255, 255, static_cast<int>(alpha * 255.0f)));
}

} // namespace

bool Main::iteration()
{
    if (!sDevice || !sEngine) return true;

    if (sProfiler) sProfiler->beginFrame();
    {
        ProfileScope scope("Events");
        sDevice->processEvents();
    }
    if (sGui)
    {
        const std::uint64_t now = sDevice->getTicksUsec();
        const float delta = sLastTicks != 0 && now > sLastTicks
                                    ? static_cast<float>(now - sLastTicks) * 1e-6f
                                    : 0.0f;
        sLastTicks = now;
        ProfileScope scope("GuiBegin");
        sGui->beginFrame(delta);
    }
    {
        ProfileScope scope("Assets");
        if (sAssets) sAssets->pump();
    }
    bool exit = false;
    {
        ProfileScope scope("Engine");
        exit = sEngine->iteration();
    }

    {
        ProfileScope scope("Audio");
        if (sAudio) sAudio->update();
    }
    if (sGui && sGui->frameOpen())
    {
        ProfileScope scope("GuiEnd");
        sGui->endFrame(false);
    }
    if (sProfiler) sProfiler->endFrame();
    return exit;
}

void Main::setListener(MainListener* listener)
{
    sListener = listener;
}

MainListener* Main::listener()
{
    return sListener;
}

void Main::cleanup()
{
    sListener = nullptr;
    if (sProject) sProject->unregisterSearchPaths();
    sProject = nullptr;
    if (sEngine) sEngine->setMainLoop(nullptr);
    if (sDevice) sDevice->input().setEventDispatchFunction(nullptr, nullptr);

    delete sScripting;
    sScripting = nullptr;
    delete sEngine;
    sEngine = nullptr;
    delete sAudio;
    sAudio = nullptr;
    delete sPhysics;
    sPhysics = nullptr;
    delete sGui;
    sGui = nullptr;
    delete sStats;
    sStats = nullptr;
    delete sAssets;
    sAssets = nullptr;
    delete sModels;
    sModels = nullptr;
    delete sRendering;
    sRendering = nullptr;
    delete sMaterials;
    sMaterials = nullptr;
    delete sTextures;
    sTextures = nullptr;
    delete sMeshes;
    sMeshes = nullptr;
    delete sProfiler;
    sProfiler = nullptr;

    if (sDevice) Device::destroy(sDevice);
    sDevice = nullptr;

    sTree = nullptr;
    sStarted = false;
    releaseShaderCompiler();
}

Device* Main::device()
{
    return sDevice;
}

Engine* Main::engine()
{
    return sEngine;
}

SceneTree* Main::tree()
{
    return sTree;
}

MeshStore* Main::meshes()
{
    return sMeshes;
}

TextureStore* Main::textures()
{
    return sTextures;
}

MaterialStore* Main::materials()
{
    return sMaterials;
}

RenderingServer* Main::rendering()
{
    return sRendering;
}

AssetLoader* Main::assets()
{
    return sAssets;
}

AudioServer* Main::audio()
{
    return sAudio;
}

PhysicsServer3D* Main::physics()
{
    return sPhysics;
}

ModelLoader* Main::models()
{
    return sModels;
}

Gui* Main::gui()
{
    return sGui;
}

RenderProfiler* Main::profiler()
{
    return sProfiler;
}

Project* Main::project()
{
    return sProject;
}

void Main::setPhysicsDebug(bool enabled)
{
    sPhysicsDebug = enabled;
}

bool Main::isPhysicsDebug()
{
    return sPhysicsDebug;
}

void Main::setOverlaySize(int width, int height)
{
    sOverlayWidth = width > 0 && height > 0 ? width : 0;
    sOverlayHeight = width > 0 && height > 0 ? height : 0;
}

void Main::overlaySize(int& width, int& height)
{
    width = sOverlayWidth;
    height = sOverlayHeight;
    if (width > 0 && height > 0) return;
    width = 0;
    height = 0;
    if (sDevice) sDevice->windowGetSize(&width, &height);
}

bool Main::isStarted()
{
    return sStarted;
}

void Main::drawFrame(bool wantsPresent, double scaledStep, void* user)
{
    (void) user;
    (void) scaledStep;
    if (!sRendering) return;
    if (sStats)
    {
        sStats->frame(sDevice->getTicksUsec());
        if (sStats->visible())
        {
            int width = 0;
            int height = 0;
            overlaySize(width, height);
            if (sProfiler) sProfiler->request();
            sStats->draw(sRendering->batch(), sRendering->getStats(), sProfiler, width, height);
        }
    }
    if (sDevice && sDevice->isGifRecording() && ((sDevice->gifRecordingFrames() / 15) % 2) == 1)
    {
        int width = 0;
        int height = 0;
        sDevice->windowGetSize(&width, &height);
        BatchRenderer& batch = sRendering->batch();
        batch.setSpace(BatchRenderer::Space::Screen);
        batch.setColor(static_cast<unsigned char>(190), static_cast<unsigned char>(33),
                static_cast<unsigned char>(55));
        batch.drawCircle(30.0f, static_cast<float>(height) - 20.0f, 10.0f);
        batch.setColor(static_cast<unsigned char>(230), static_cast<unsigned char>(41),
                static_cast<unsigned char>(55));
        batch.drawText(50.0f, static_cast<float>(height) - 30.0f, 14.0f, "GIF RECORDING");
        batch.setColor(static_cast<unsigned char>(255), static_cast<unsigned char>(255),
                static_cast<unsigned char>(255));
    }
    if (sPhysicsDebug && sPhysics)
    {
        Engine* engine = Engine::singleton();
        const float fraction =
                engine ? static_cast<float>(engine->getPhysicsInterpolationFraction()) : 1.0f;
        sPhysics->debugDraw(sRendering->batch(), fraction);
    }
    {
        ProfileScope scope("Viewports");
        sRendering->renderViewports();
    }
    {
        ProfileScope scope("GUI");
        if (sListener)
        {
            ProfileScope overlayScope("GuiOverlay");
            sListener->drawOverlay();
        }
        if (sGui && sGui->frameOpen()) sGui->endFrame(true);
    }
    if (sDevice) sDevice->gifRecordingFrame();
    {
        ProfileScope scope("Present");
        sRendering->endFrame(wantsPresent);
    }
    if (wantsPresent && sFramesPresented < 2)
    {
        ++sFramesPresented;
        log_info("main: frame %llu presented %.1f ms after setup",
                static_cast<unsigned long long>(sFramesPresented),
                static_cast<double>(time_nanos() - sSetupTicks) * 1e-6);
    }
}

} // namespace prisma3d
