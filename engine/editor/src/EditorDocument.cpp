#include "prisma3d/editor/EditorDocument.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneTree.h"

#include "platform.h"

#include <time.h>

namespace prisma3d
{

namespace
{

const char* const kParkedName = "~replaced";
const char* const kUntitledScene = "Scene";
const char* const kPlayingRefusal = "stop the game first";

class HintScope
{
public:
    explicit HintScope(bool edit)
            : mEngine(Engine::singleton()),
              mPrevious(false)
    {
        if (!mEngine) return;
        mPrevious = mEngine->isEditorHint();
        mEngine->setEditorHint(edit);
    }

    ~HintScope()
    {
        if (mEngine) mEngine->setEditorHint(mPrevious);
    }

    HintScope(const HintScope&) = delete;
    HintScope& operator=(const HintScope&) = delete;

private:
    Engine* mEngine;
    bool mPrevious;
};

bool isAbsoluteText(const ct::String& path)
{
    if (path.empty()) return false;
    if (path[0] == '/' || path[0] == '\\') return true;
    return path.size() >= 2 && path[1] == ':';
}

void mix(std::uint64_t& hash, std::uint64_t value)
{
    hash ^= value;
    hash *= 1099511628211ull;
}

void mixText(std::uint64_t& hash, const ct::String& text)
{
    for (std::size_t i = 0; i < text.size(); ++i) mix(hash, static_cast<unsigned char>(text[i]));
    mix(hash, 0xffu);
}

void hashNodes(const Node* node, std::uint64_t& hash)
{
    mixText(hash, node->getName());
    mixText(hash, ct::String(node->getClassName()));
    mix(hash, node->getChildCount());
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
    {
        const Node* child = node->getChild(i);
        if (child->isInternal()) continue;
        hashNodes(child, hash);
    }
}

ct::String fileNameOf(const ct::String& path)
{
    const std::size_t slash = path.rfind('/');
    return slash == ct::String::npos ? path : path.substr(slash + 1);
}

bool forbiddenNameCharacter(char c)
{
    if (static_cast<unsigned char>(c) < 32) return true;
    return c == '/' || c == '\\' || c == ':' || c == '*' || c == '?' || c == '"' || c == '<' ||
           c == '>' || c == '|';
}

} // namespace

EditorDocument::EditorDocument(SceneTree& tree, EditorSelection& selection,
        SceneTreePanel& hierarchy, Project& project, EditorSettings& settings)
        : mTree(tree),
          mSelection(selection),
          mHierarchy(hierarchy),
          mProject(project),
          mSettings(settings),
          mScene(nullptr),
          mParked(nullptr),
          mOrderHash(0),
          mSerial(0),
          mSavedAt(0),
          mSnapshotSavedAt(0),
          mSuspended(0),
          mDirty(false),
          mOrderPending(false),
          mPlaying(false),
          mSnapshotDirty(false)
{
    mTree.addListener(this);
    mScene = hierarchy.editedScene();
    if (mScene) mOrderHash = orderHash();
}

EditorDocument::~EditorDocument()
{
    mTree.removeListener(this);
    if (mParked) mParked->queueFree();
}

void EditorDocument::setSettingsPath(const ct::String& path)
{
    mSettingsPath = path;
    mSettings.setPath(path);
}

const ct::String& EditorDocument::settingsPath() const
{
    return mSettingsPath;
}

Project& EditorDocument::project()
{
    return mProject;
}

const Project& EditorDocument::project() const
{
    return mProject;
}

EditorSettings& EditorDocument::settings()
{
    return mSettings;
}

const EditorSettings& EditorDocument::settings() const
{
    return mSettings;
}

Node* EditorDocument::scene() const
{
    return mScene;
}

const ct::String& EditorDocument::scenePath() const
{
    return mScenePath;
}

bool EditorDocument::hasScenePath() const
{
    return !mScenePath.empty();
}

bool EditorDocument::sceneInProject() const
{
    return mProject.isOpen() && !mScenePath.empty() && !isAbsoluteText(mScenePath);
}

bool EditorDocument::isMainScene() const
{
    return sceneInProject() && mScenePath == mProject.mainScene();
}

ct::String EditorDocument::sceneName() const
{
    if (mScenePath.empty()) return ct::String(kUntitledName);
    return fileNameOf(mScenePath);
}

std::int64_t EditorDocument::savedAt() const
{
    return mSavedAt;
}

bool EditorDocument::dirty() const
{
    return mDirty || mUndo.isHistoryUnsaved();
}

EditorUndoRedoManager& EditorDocument::undoRedo()
{
    return mUndo;
}

const EditorUndoRedoManager& EditorDocument::undoRedo() const
{
    return mUndo;
}

EditorUndoRedoManager* EditorDocument::undoRedoManager()
{
    return &mUndo;
}

bool EditorDocument::undo()
{
    if (mPlaying || !mScene) return false;
    return mUndo.undo();
}

bool EditorDocument::redo()
{
    if (mPlaying || !mScene) return false;
    return mUndo.redo();
}

void EditorDocument::markDirty()
{
    if (mScene && !mPlaying) mDirty = true;
}

void EditorDocument::sceneEdited()
{
    markDirty();
}

const ct::String& EditorDocument::error() const
{
    return mError;
}

const SceneResult& EditorDocument::lastResult() const
{
    return mResult;
}

unsigned int EditorDocument::sceneSerial() const
{
    return mSerial;
}

bool EditorDocument::fail(const ct::String& message)
{
    mError = message;
    return false;
}

bool EditorDocument::touches(const Node* node, bool includeScene) const
{
    if (mSuspended > 0 || mPlaying || !mScene || !node || mUndo.busy()) return false;
    if (node == mScene) return includeScene;
    if (!mScene->isAncestorOf(node)) return false;
    for (const Node* current = node; current && current != mScene; current = current->getParent())
        if (current->isInternal()) return false;
    return true;
}

void EditorDocument::treeChanged()
{
    if (mSuspended > 0 || mPlaying || !mScene || mUndo.busy()) return;
    mOrderPending = true;
}

void EditorDocument::nodeAdded(Node* node)
{
    if (touches(node, false)) mDirty = true;
}

void EditorDocument::nodeRemoved(Node* node)
{
    if (touches(node, false)) mDirty = true;
}

void EditorDocument::nodeRenamed(Node* node)
{
    if (touches(node, true)) mDirty = true;
}

std::uint64_t EditorDocument::orderHash() const
{
    std::uint64_t hash = 14695981039346656037ull;
    if (mScene) hashNodes(mScene, hash);
    return hash;
}

void EditorDocument::update()
{
    if (!mOrderPending || mPlaying) return;
    mOrderPending = false;
    if (!mDirty && orderHash() != mOrderHash) mDirty = true;
}

void EditorDocument::install(Node* fresh, const ct::String& path)
{
    Node* old = mScene;
    mSelection.clear();
    mUndo.clearHistory();
    mUndo.setHistoryAsSaved();
    mHierarchy.setEditedScene(fresh);
    mScene = fresh;
    mScenePath = path;
    mDirty = false;
    mSavedAt = 0;
    mOrderPending = false;
    mOrderHash = orderHash();
    ++mSerial;
    if (old) old->queueFree();
}

Node* EditorDocument::createBlankScene(const char* name)
{
    Node* root = mTree.getRoot();
    if (!root) return nullptr;
    Node3D* fresh = root->createChild<Node3D>();
    if (fresh) fresh->setName(name);
    return fresh;
}

ct::String EditorDocument::describePath(const ct::String& path, ct::String& absolute) const
{
    ct::String relative;
    if (mProject.isOpen() && mProject.relativePath(path, relative))
    {
        absolute = mProject.absolutePath(relative);
        return relative;
    }
    char buffer[4096];
    if (path_absolute(buffer, sizeof buffer, path.c_str())) absolute = ct::String(buffer);
    else
        absolute = path;
    return absolute;
}

ct::String EditorDocument::sceneAbsolutePath(const ct::String& scenePath) const
{
    if (mProject.isOpen() && !isAbsoluteText(scenePath)) return mProject.absolutePath(scenePath);
    return scenePath;
}

ct::String EditorDocument::sceneFileName(const ct::String& name)
{
    const ct::String trimmed = name.trimmed();
    if (trimmed.ends_with(kSceneSuffix)) return trimmed;
    return trimmed + kSceneSuffix;
}

bool EditorDocument::validSceneName(const ct::String& name, ct::String& reason) const
{
    const ct::String trimmed = name.trimmed();
    if (trimmed.empty())
    {
        reason = "the scene name is empty";
        return false;
    }
    for (std::size_t i = 0; i < trimmed.size(); ++i)
        if (forbiddenNameCharacter(trimmed[i]))
        {
            reason = "the scene name has a character that a file name cannot have";
            return false;
        }
    if (trimmed[0] == '.')
    {
        reason = "the scene name must not start with a dot";
        return false;
    }
    if (mProject.isOpen())
    {
        const ct::String relative =
                ct::String(Project::kScenesFolder) + "/" + sceneFileName(trimmed);
        if (relative != mScenePath && file_exists(mProject.absolutePath(relative).c_str()))
        {
            reason = ct::String("'") + relative + "' already exists";
            return false;
        }
    }
    return true;
}

void EditorDocument::rememberProject()
{
    mSettings.rememberProject(mProject.manifestPath());
    if (!mSettingsPath.empty()) mSettings.save(mSettingsPath);
}

void EditorDocument::setAssetSaver(const AssetSaver& saver)
{
    mAssetSaver = saver;
}

bool EditorDocument::writeAssets()
{
    if (!mAssetSaver) return true;
    ct::String error;
    if (mAssetSaver(error)) return true;
    return fail(ct::String("could not save the materials: ") + error);
}

bool EditorDocument::writeScene(const ct::String& absolute)
{
    if (!writeAssets()) return false;
    mResult = SceneResult();
    if (!SceneSerializer::saveFile(mScene, absolute, &mResult))
        return fail(ct::String("could not save '") + absolute + "':\n" + mResult.describe());
    mDirty = false;
    mUndo.setHistoryAsSaved();
    mSavedAt = static_cast<std::int64_t>(time(nullptr));
    mOrderPending = false;
    mOrderHash = orderHash();
    return true;
}

bool EditorDocument::newScene()
{
    if (mPlaying) return fail(kPlayingRefusal);
    ++mSuspended;
    ct::String previous;
    if (mScene)
    {
        previous = mScene->getName();
        mScene->setName(kParkedName);
    }
    Node* fresh = createBlankScene(kUntitledScene);
    if (!fresh)
    {
        if (mScene) mScene->setName(previous);
        --mSuspended;
        return fail("the scene tree has no root");
    }
    install(fresh, ct::String());
    --mSuspended;
    return true;
}

bool EditorDocument::openScene(const ct::String& path)
{
    Node* root = mTree.getRoot();
    if (!root) return fail("the scene tree has no root");
    if (mPlaying) return fail(kPlayingRefusal);
    if (path.empty()) return fail("the scene path is empty");

    ct::String absolute;
    const ct::String canonical = describePath(path, absolute);
    if (!file_exists(absolute.c_str()))
        return fail(ct::String("the scene '") + absolute + "' does not exist");

    ++mSuspended;
    ct::String previous;
    if (mScene)
    {
        previous = mScene->getName();
        mScene->setName(kParkedName);
    }
    SceneResult result;
    Node* loaded = SceneSerializer::loadFile(absolute, root, result);
    if (!loaded || !result.success())
    {
        if (loaded) loaded->queueFree();
        if (mScene) mScene->setName(previous);
        --mSuspended;
        mResult = result;
        return fail(ct::String("could not open '") + absolute + "':\n" + result.describe());
    }
    install(loaded, canonical);
    mResult = result;
    --mSuspended;
    if (result.warningCount() > 0)
        log_warn("editor: '%s':\n%s", absolute.c_str(), result.describe().c_str());

    if (mProject.isOpen() && !isAbsoluteText(canonical) && !mProject.hasScene(canonical))
    {
        if (mProject.addScene(canonical)) mProject.save();
    }
    return true;
}

bool EditorDocument::saveScene()
{
    if (mPlaying) return fail(kPlayingRefusal);
    if (!mScene) return fail("there is no scene");
    if (mScenePath.empty()) return fail("the scene has no file yet");
    return writeScene(sceneAbsolutePath(mScenePath));
}

bool EditorDocument::saveSceneAs(const ct::String& name)
{
    if (mPlaying) return fail(kPlayingRefusal);
    if (!mScene) return fail("there is no scene");
    if (!mProject.isOpen()) return fail("open a project before saving a scene");
    ct::String reason;
    if (!validSceneName(name, reason)) return fail(reason);

    const ct::String relative = ct::String(Project::kScenesFolder) + "/" + sceneFileName(name);
    if (!writeScene(mProject.absolutePath(relative))) return false;
    if (!mProject.hasScene(relative))
    {
        if (!mProject.addScene(relative)) return fail(mProject.error());
        if (!mProject.save()) return fail(mProject.error());
    }
    mScenePath = relative;
    return true;
}

bool EditorDocument::setMainScene()
{
    if (mPlaying) return fail(kPlayingRefusal);
    if (!sceneInProject()) return fail("save the scene inside the project first");
    if (!mProject.setMainScene(mScenePath)) return fail(mProject.error());
    if (!mProject.save()) return fail(mProject.error());
    return true;
}

bool EditorDocument::saveProject()
{
    if (mPlaying) return fail(kPlayingRefusal);
    if (!mProject.isOpen()) return fail("no project is open");
    if (!writeAssets()) return false;
    if (!mProject.save()) return fail(mProject.error());
    return true;
}

void EditorDocument::setProjectLeavingHandler(const ProjectHandler& handler)
{
    mProjectLeaving = handler;
}

void EditorDocument::projectLeaving()
{
    if (mProject.isOpen() && mProjectLeaving) mProjectLeaving();
}

bool EditorDocument::createProject(const ct::String& parentDirectory, const ct::String& name)
{
    if (mPlaying) return fail(kPlayingRefusal);
    projectLeaving();
    if (!mProject.create(parentDirectory, name)) return fail(mProject.error());
    mProject.registerSearchPaths();
    rememberProject();

    ++mSuspended;
    ct::String previous;
    if (mScene)
    {
        previous = mScene->getName();
        mScene->setName(kParkedName);
    }
    Node* fresh = createBlankScene("Main");
    if (!fresh)
    {
        if (mScene) mScene->setName(previous);
        --mSuspended;
        return fail("the scene tree has no root");
    }
    install(fresh, ct::String());
    --mSuspended;

    if (!saveSceneAs(kInitialSceneName)) return false;
    return setMainScene();
}

bool EditorDocument::openProject(const ct::String& manifestPath)
{
    if (mPlaying) return fail(kPlayingRefusal);
    projectLeaving();
    if (!mProject.open(manifestPath)) return fail(mProject.error());
    mProject.registerSearchPaths();
    rememberProject();

    ct::String sceneError;
    if (!mProject.mainScene().empty())
    {
        if (openScene(mProject.mainScene()))
        {
            mError = ct::String();
            return true;
        }
        sceneError = mError;
    }
    newScene();
    mError = sceneError;
    return true;
}

void EditorDocument::closeProject()
{
    if (mPlaying) return;
    projectLeaving();
    mProject.close();
    mSettings.clearLastProject();
    if (!mSettingsPath.empty()) mSettings.save(mSettingsPath);
    newScene();
}

bool EditorDocument::playing() const
{
    return mPlaying;
}

const ct::String& EditorDocument::snapshot() const
{
    return mSnapshot;
}

ct::Vector<ct::String> EditorDocument::selectedPaths() const
{
    ct::Vector<ct::String> paths;
    if (!mScene) return paths;
    const ct::Vector<Node*>& selected = mSelection.selectedNodes();
    for (std::size_t i = 0; i < selected.size(); ++i)
    {
        if (!selected[i]->isInsideTree()) continue;
        const ct::String path = mScene->getPathTo(selected[i]);
        if (!path.empty()) paths.push_back(path);
    }
    return paths;
}

void EditorDocument::selectPaths(const ct::Vector<ct::String>& paths)
{
    if (!mScene) return;
    for (std::size_t i = 0; i < paths.size(); ++i)
        if (Node* node = mScene->getNode(paths[i])) mSelection.add(node);
}

bool EditorDocument::swapScene(const ct::String& text, bool keepOriginal)
{
    Node* root = mTree.getRoot();
    if (!root || !mScene) return fail("the scene tree has no root");

    ++mSuspended;
    const ct::String previous = mScene->getName();
    mScene->setName(kParkedName);
    SceneResult result;
    Node* fresh = SceneSerializer::load(text, root, result);
    if (!fresh || !result.success())
    {
        if (fresh) fresh->queueFree();
        mScene->setName(previous);
        --mSuspended;
        mResult = result;
        return fail(ct::String("could not rebuild the scene:\n") + result.describe());
    }
    Node* old = mScene;
    const ct::Vector<ct::String> paths = selectedPaths();
    mSelection.clear();
    mHierarchy.setEditedScene(fresh);
    mScene = fresh;
    ++mSerial;
    if (keepOriginal)
    {
        {
            HintScope silent(true);
            root->removeChild(old);
        }
        mParked = old;
        mParkedName = previous;
    }
    else
        old->queueFree();
    selectPaths(paths);
    mResult = result;
    --mSuspended;
    return true;
}

bool EditorDocument::restoreParked()
{
    Node* root = mTree.getRoot();
    if (!root || !mParked || !mScene) return fail("the scene tree has no root");

    ++mSuspended;
    Node* played = mScene;
    const ct::Vector<ct::String> paths = selectedPaths();
    mSelection.clear();
    {
        HintScope live(false);
        root->removeChild(played);
    }
    played->queueFree();
    {
        HintScope silent(true);
        mParked->setName(mParkedName);
        root->addChild(mParked, false);
    }
    mHierarchy.setEditedScene(mParked);
    mScene = mParked;
    mParked = nullptr;
    mParkedName = ct::String();
    ++mSerial;
    selectPaths(paths);
    --mSuspended;
    return true;
}

bool EditorDocument::beginPlay()
{
    if (mPlaying) return swapScene(mSnapshot, false);
    if (!mScene) return fail("there is no scene");

    update();
    SceneResult result;
    ct::String text;
    if (!SceneSerializer::save(mScene, text, &result))
        return fail(ct::String("could not snapshot the scene:\n") + result.describe());

    mSnapshot = text;
    mSnapshotDirty = mDirty;
    mSnapshotSavedAt = mSavedAt;
    if (!swapScene(mSnapshot, true))
    {
        mSnapshot = ct::String();
        return false;
    }
    mUndo.setFrozen(true);
    mPlaying = true;
    return true;
}

bool EditorDocument::endPlay()
{
    if (!mPlaying) return fail("the game is not playing");
    if (!restoreParked()) return false;

    mUndo.setFrozen(false);
    mPlaying = false;
    mSnapshot = ct::String();
    mDirty = mSnapshotDirty;
    mSavedAt = mSnapshotSavedAt;
    mOrderPending = false;
    mOrderHash = orderHash();
    return true;
}

} // namespace prisma3d
