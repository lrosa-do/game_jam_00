#include "Check.h"
#include "TestFiles.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/editor/DefaultProject.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorPlay.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/ScriptInstance.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/script/ZenScripting.h"

#include "platform.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kWidth = 1280;
const int kHeight = 720;

bool useVulkan = false;

ct::String testRoot()
{
    ct::String root = PRISMA3D_TEST_DIR;
    root += useVulkan ? "/editor_default_project_test_vulkan" : "/editor_default_project_test";
    return root;
}

ct::Vector<ct::String> printed;

void collectPrint(const char* line, void*)
{
    printed.push_back(ct::String(line));
}

ct::String printedLines()
{
    ct::String text;
    for (std::size_t i = 0; i < printed.size(); ++i)
    {
        if (i > 0) text += ",";
        text += printed[i];
    }
    return text;
}

bool near(float a, float b, float tolerance = 0.001f)
{
    return fabsf(a - b) < tolerance;
}

struct Rig
{
    SceneTree& tree;
    WidgetHost& host;
    EditorDocument& document;
    EditorSettings& settings;
    EditorPlay& play;
};

void frame(Rig& rig)
{
    rig.tree.process(1.0 / 60.0);
    rig.tree.physicsProcess(1.0 / 60.0);
    rig.play.update();
    Main::rendering()->renderViewports();
    rig.host.frame(1.0f / 60.0f, true);
    Main::rendering()->endFrame(true);
}

void frames(Rig& rig, int count)
{
    for (int i = 0; i < count; ++i) frame(rig);
}

ct::String readText(const ct::String& path)
{
    char* text = file_read_text(path.c_str());
    ct::String result = text ? ct::String(text) : ct::String();
    free(text);
    return result;
}

MeshInstance3D* mesh(Rig& rig, const char* name)
{
    Node* node = rig.document.scene() ? rig.document.scene()->getNode(name) : nullptr;
    return node ? node->castTo<MeshInstance3D>() : nullptr;
}

void checkPrimitive(MeshInstance3D* node, PrimitiveKind kind, float size)
{
    CHECK(node != nullptr);
    if (!node) return;
    PrimitiveRecipe recipe;
    ct::String error;
    CHECK(parsePrimitiveRecipe(sceneres::pathOf(node->getMesh()), recipe, error));
    CHECK(recipe.kind == kind);
    CHECK(near(recipe.values[0], size));
    CHECK(near(recipe.values[1], size));
}

void checkScene(Rig& rig, const ct::String& appFolder)
{
    EditorDocument& document = rig.document;
    const Project& project = document.project();
    const ct::String manifest = DefaultProject::manifestPath(appFolder);
    CHECK(project.isOpen());
    CHECK(project.manifestPath() == manifest);
    CHECK(project.name() == DefaultProject::kName);
    CHECK(project.mainScene() == "Scenes/main.scene.json");
    CHECK(project.hasScene("Scenes/main.scene.json"));
    CHECK(document.scenePath() == "Scenes/main.scene.json");
    CHECK(document.isMainScene());
    CHECK(!document.dirty());
    CHECK(rig.settings.lastProjectPath == manifest);

    const ct::String root = project.root();
    CHECK(dir_exists((root + "/Scenes").c_str()));
    CHECK(dir_exists((root + "/Assets/Scripts").c_str()));
    CHECK(dir_exists((root + "/Assets/Models").c_str()));
    CHECK(dir_exists((root + "/Assets/Textures").c_str()));
    CHECK(dir_exists((root + "/Assets/Materials").c_str()));
    CHECK(file_exists((root + "/Assets/Materials/Orange.mat").c_str()));
    CHECK(file_exists((root + "/Assets/Materials/Ground.mat").c_str()));
    CHECK(readText(root + "/Assets/Scripts/rotate.py") == DefaultProject::kScriptText);

    Node* scene = document.scene();
    CHECK(scene != nullptr && scene->getName() == "Main");
    CHECK(scene && scene->castTo<Node3D>() != nullptr);
    CHECK(scene && scene->getChildCount() == 4);

    MeshInstance3D* floor = mesh(rig, "Floor");
    checkPrimitive(floor, PrimitiveKind::Plane, 20.0f);
    CHECK(floor && floor->getPosition() == Math::Vec3(0.0f, 0.0f, 0.0f));

    MeshInstance3D* box = mesh(rig, "Box");
    checkPrimitive(box, PrimitiveKind::Box, 1.0f);
    CHECK(box && near(box->getPosition().y, 0.5f));
    CHECK(box && box->getScript() == DefaultProject::kScriptPath);
    CHECK(box && box->getScriptInstance() && box->getScriptInstance()->hasScript());
    const MaterialId material = box ? box->getSurfaceMaterial(0) : MaterialId();
    CHECK(material.valid());
    const MaterialInfo* info = material.valid() ? Main::materials()->get(material) : nullptr;
    CHECK(info != nullptr);
    if (info)
    {
        CHECK(info->albedo.x > 0.9f && info->albedo.y > 0.3f && info->albedo.y < 0.6f &&
                info->albedo.z < 0.2f);
    }

    DirectionalLight3D* sun = scene ? scene->getNode("Sun")->castTo<DirectionalLight3D>() : nullptr;
    CHECK(sun != nullptr);
    CHECK(sun && sun->hasShadow());
    if (sun)
    {
        const Math::Vec3 down = sun->getBasis() * Math::Vec3(0.0f, 0.0f, -1.0f);
        CHECK(down.y < -0.5f);
    }

    Camera3D* camera = scene ? scene->getNode("MainCamera")->castTo<Camera3D>() : nullptr;
    CHECK(camera != nullptr);
    CHECK(camera && camera->isCurrent());
    if (camera)
    {
        CHECK(camera->getPosition() == Math::Vec3(0.0f, 3.0f, 6.0f));
        const Math::Vec3 forward = camera->getBasis() * Math::Vec3(0.0f, 0.0f, -1.0f);
        const Math::Vec3 toBox = Math::Vec3(0.0f, 0.5f, 0.0f) - camera->getPosition();
        CHECK(forward.Normalized().Dot(toBox.Normalized()) > 0.999f);
    }
}

void testCreated(Rig& rig, const ct::String& appFolder)
{
    CHECK(DefaultProject::wanted(nullptr, ct::String()));
    CHECK(DefaultProject::wanted(nullptr, appFolder + "/gone/projecto.json"));
    CHECK(!DefaultProject::wanted("some/project.json", ct::String()));
    CHECK(!DefaultProject::wanted("some/project.json", appFolder + "/gone/projecto.json"));

    CHECK(DefaultProject::manifestPath(appFolder) == appFolder + "/projects/Default/projecto.json");
    CHECK(DefaultProject::manifestPath(appFolder + "/") == DefaultProject::manifestPath(appFolder));
    CHECK(!DefaultProject::exists(appFolder));
    CHECK(DefaultProject::open(rig.document, appFolder));
    CHECK(rig.document.error().empty());
    CHECK(DefaultProject::exists(appFolder));
    CHECK(!DefaultProject::wanted(nullptr, DefaultProject::manifestPath(appFolder)));
    checkScene(rig, appFolder);

    const EditorSettings& stored = rig.settings;
    EditorSettings reread;
    CHECK(reread.load(stored.path()));
    CHECK(reread.lastProjectPath == DefaultProject::manifestPath(appFolder));
}

void testReused(Rig& rig, const ct::String& appFolder)
{
    rig.document.closeProject();
    CHECK(!rig.document.project().isOpen());
    CHECK(rig.settings.lastProjectPath.empty());
    CHECK(DefaultProject::exists(appFolder));
    CHECK(DefaultProject::open(rig.document, appFolder));
    checkScene(rig, appFolder);

    const Project& project = rig.document.project();
    const ct::String scriptFile = project.absolutePath(DefaultProject::kScriptPath);
    const ct::String sceneFile = project.absolutePath("Scenes/main.scene.json");
    const ct::String materialFile = project.absolutePath("Assets/Materials/Orange.mat");

    const ct::String edited = "class Rotate(MeshInstance3D):\n    speed = 1.0\n";
    CHECK(file_write_text(scriptFile.c_str(), edited.c_str()));
    Node* floor = rig.document.scene()->getNode("Floor");
    CHECK(floor != nullptr);
    if (floor) floor->setName("Ground");
    CHECK(rig.document.saveScene());
    const ct::String sceneBefore = readText(sceneFile);
    const ct::String materialBefore = readText(materialFile);
    CHECK(sceneBefore.contains("Ground") && !sceneBefore.contains("Floor"));

    rig.document.closeProject();
    CHECK(DefaultProject::open(rig.document, appFolder));
    CHECK(rig.document.project().isOpen());
    CHECK(rig.document.scene()->getNode("Ground") != nullptr);
    CHECK(rig.document.scene()->getNode("Floor") == nullptr);
    CHECK(rig.settings.lastProjectPath == DefaultProject::manifestPath(appFolder));
    CHECK(readText(scriptFile) == edited);
    CHECK(readText(sceneFile) == sceneBefore);
    CHECK(readText(materialFile) == materialBefore);

    rig.document.closeProject();
    CHECK(!DefaultProject::create(rig.document, appFolder));
    CHECK(readText(scriptFile) == edited);
    CHECK(readText(sceneFile) == sceneBefore);
    CHECK(readText(materialFile) == materialBefore);
}

void testPartialFolder(Rig& rig, const ct::String& appFolder)
{
    const ct::String root = DefaultProject::parentFolder(appFolder) + "/" + DefaultProject::kName;
    CHECK(fs_create_directory((root + "/Assets/Scripts").c_str()));
    const ct::String mine = "class Rotate(MeshInstance3D):\n    speed = 5.0\n";
    CHECK(file_write_text((root + "/" + DefaultProject::kScriptPath).c_str(), mine.c_str()));
    CHECK(!DefaultProject::exists(appFolder));
    CHECK(DefaultProject::open(rig.document, appFolder));
    CHECK(rig.document.project().isOpen());
    CHECK(readText(root + "/" + DefaultProject::kScriptPath) == mine);
    rig.document.closeProject();
}

void testPlay(Rig& rig, const ct::String& appFolder)
{
    CHECK(DefaultProject::open(rig.document, appFolder));
    checkScene(rig, appFolder);
    frames(rig, 3);
    CHECK(printedLines().empty());

    MeshInstance3D* editBox = mesh(rig, "Box");
    CHECK(editBox != nullptr);

    CHECK(rig.play.play());
    CHECK(rig.play.playing());
    CHECK(printedLines() == "rotate.py ready on Box");

    MeshInstance3D* box = mesh(rig, "Box");
    CHECK(box != nullptr && box != editBox);
    CHECK(box && near(box->getRotationDegrees().y, 0.0f, 0.01f));
    frames(rig, 30);
    CHECK(box && near(box->getRotationDegrees().y, 45.0f, 0.5f));
    CHECK(box && near(box->getRotationDegrees().x, 0.0f, 0.01f));
    CHECK(box && near(box->getRotationDegrees().z, 0.0f, 0.01f));
    CHECK(box && box->get("speed").asFloat() == 90.0);
    CHECK(ZenScripting::instance()->getErrorCount() == 0);
    CHECK(printedLines() == "rotate.py ready on Box");

    CHECK(rig.play.stop());
    frames(rig, 2);
    MeshInstance3D* restored = mesh(rig, "Box");
    CHECK(restored != nullptr);
    CHECK(restored && near(restored->getRotationDegrees().y, 0.0f, 0.01f));
    CHECK(!rig.document.dirty());
}

} // namespace

int main(int argc, char** argv)
{
    useVulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    removeTree(testRoot());
    CHECK(fs_create_directory(testRoot().c_str()));

    DeviceDesc desc;
    desc.title = "editor default project test";
    desc.driver = useVulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    CHECK(Main::rendering() != nullptr && Main::gui() != nullptr);
    if (!Main::rendering() || !Main::gui()) return 1;
    Engine::singleton()->setEditorHint(true);
    CHECK(ZenScripting::instance() != nullptr);
    if (ZenScripting::instance()) ZenScripting::instance()->setPrintHandler(&collectPrint, nullptr);

    SceneTree tree;
    CHECK(Main::start(&tree));
    tree.setAutoAcceptQuit(false);
    Window* window = tree.getRoot() ? tree.getRoot()->castTo<Window>() : nullptr;
    CHECK(window != nullptr);
    if (window) Main::rendering()->viewportSetActive(window->getViewportId(), false);

    Node3D* scene = tree.getRoot() ? tree.getRoot()->createChild<Node3D>() : nullptr;
    CHECK(scene != nullptr);
    if (scene && window)
    {
        scene->setName("Scene");
        {
            WidgetHost host(*Main::device(), *Main::gui(), *Main::textures());
            CHECK(host.init());
            EditorShell shell(host);
            CHECK(shell.build());
            {
                EditorSelection selection(tree);
                SceneTreePanel hierarchy(tree, selection);
                CHECK(hierarchy.attach(shell.panel(EditorShell::kHierarchyPanel)));
                hierarchy.setEditedScene(scene);

                const ct::String firstApp = testRoot() + "/first_app";
                const ct::String secondApp = testRoot() + "/second_app";
                const ct::String thirdApp = testRoot() + "/third_app";
                CHECK(fs_create_directory(firstApp.c_str()));

                EditorSettings settings;
                Project project;
                EditorDocument document(tree, selection, hierarchy, project, settings);
                document.setSettingsPath(firstApp + "/" + EditorSettings::kIniName);
                hierarchy.setChangeListener(&document);

                EditorPlay play(tree, document);
                CHECK(play.attach(shell.runBar()));

                Rig rig = { tree, host, document, settings, play };
                frames(rig, 3);
                testCreated(rig, firstApp);
                testReused(rig, firstApp);
                testPartialFolder(rig, thirdApp);
                testPlay(rig, secondApp);
            }
            host.shutdown();
        }
    }

    tree.finalize();
    CHECK(tree.getRoot() == nullptr);
    Main::cleanup();
    CHECK(Main::device() == nullptr);
    removeTree(testRoot());

    if (failures != 0)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    return 0;
}
