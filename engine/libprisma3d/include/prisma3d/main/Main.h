#pragma once

#include "prisma3d/Device.h"

namespace prisma3d
{

class Engine;
class SceneTree;
class MeshStore;
class TextureStore;
class MaterialStore;
class RenderingServer;
class AssetLoader;
class AudioServer;
class PhysicsServer3D;
class ModelLoader;
class Gui;
class RenderProfiler;
class Project;
class Node;

struct SceneResult;

struct InputEvent;

class MainListener
{
public:
    virtual ~MainListener()
    {
    }
    virtual void inputEvent(const InputEvent& event)
    {
        (void) event;
    }
    virtual void drawOverlay()
    {
    }
    virtual bool forwardsToScene(const InputEvent& event)
    {
        (void) event;
        return true;
    }
};

class Main
{
public:
    static bool setup(const DeviceDesc& desc);
    static bool setupProject(Project& project, const DeviceDesc& base = DeviceDesc());
    static bool start(SceneTree* tree);
    static Node* loadMainScene(SceneResult& result);
    static bool iteration();
    static void cleanup();

    static Device* device();
    static Engine* engine();
    static SceneTree* tree();
    static MeshStore* meshes();
    static TextureStore* textures();
    static MaterialStore* materials();
    static RenderingServer* rendering();
    static AssetLoader* assets();
    static AudioServer* audio();
    static PhysicsServer3D* physics();
    static ModelLoader* models();
    static Gui* gui();
    static RenderProfiler* profiler();
    static Project* project();
    static void setPhysicsDebug(bool enabled);
    static bool isPhysicsDebug();
    static void setOverlaySize(int width, int height);
    static void overlaySize(int& width, int& height);
    static bool isStarted();
    static void setListener(MainListener* listener);
    static MainListener* listener();

private:
    static void drawFrame(bool wantsPresent, double scaledStep, void* user);
};

} // namespace prisma3d
