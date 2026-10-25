#pragma once

#include "prisma3d/editor/EditorChange.h"
#include "prisma3d/editor/EditorUndoRedoManager.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTreeListener.h"

#include <ct/function.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class EditorSelection;
class EditorSettings;
class Node;
class Project;
class SceneTree;
class SceneTreePanel;

class EditorDocument : private SceneTreeListener, public EditorChangeListener
{
public:
    static constexpr const char* kSceneSuffix = ".scene.json";
    static constexpr const char* kInitialSceneName = "main";
    static constexpr const char* kUntitledName = "Untitled";

    EditorDocument(SceneTree& tree, EditorSelection& selection, SceneTreePanel& hierarchy,
            Project& project, EditorSettings& settings);
    ~EditorDocument() override;
    EditorDocument(const EditorDocument&) = delete;
    EditorDocument& operator=(const EditorDocument&) = delete;

    using AssetSaver = ct::Function<bool(ct::String&)>;
    using ProjectHandler = ct::Function<void()>;

    void setAssetSaver(const AssetSaver& saver);
    void setProjectLeavingHandler(const ProjectHandler& handler);
    void setSettingsPath(const ct::String& path);
    const ct::String& settingsPath() const;

    Project& project();
    const Project& project() const;
    EditorSettings& settings();
    const EditorSettings& settings() const;

    bool createProject(const ct::String& parentDirectory, const ct::String& name);
    bool openProject(const ct::String& manifestPath);
    bool saveProject();
    void closeProject();

    bool newScene();
    bool openScene(const ct::String& path);
    bool saveScene();
    bool saveSceneAs(const ct::String& name);
    bool setMainScene();

    bool playing() const;
    bool beginPlay();
    bool endPlay();
    const ct::String& snapshot() const;

    bool validSceneName(const ct::String& name, ct::String& reason) const;
    static ct::String sceneFileName(const ct::String& name);
    ct::String sceneAbsolutePath(const ct::String& scenePath) const;

    Node* scene() const;
    const ct::String& scenePath() const;
    bool hasScenePath() const;
    bool sceneInProject() const;
    bool isMainScene() const;
    ct::String sceneName() const;

    bool dirty() const;
    std::int64_t savedAt() const;
    void markDirty();
    void update();

    EditorUndoRedoManager& undoRedo();
    const EditorUndoRedoManager& undoRedo() const;
    bool undo();
    bool redo();

    const ct::String& error() const;
    const SceneResult& lastResult() const;
    unsigned int sceneSerial() const;

    void sceneEdited() override;
    EditorUndoRedoManager* undoRedoManager() override;

private:
    void treeChanged() override;
    void nodeAdded(Node* node) override;
    void nodeRemoved(Node* node) override;
    void nodeRenamed(Node* node) override;

    bool fail(const ct::String& message);
    bool touches(const Node* node, bool includeScene) const;
    std::uint64_t orderHash() const;
    void install(Node* fresh, const ct::String& path);
    Node* createBlankScene(const char* name);
    void projectLeaving();
    ct::String describePath(const ct::String& path, ct::String& absolute) const;
    void rememberProject();
    bool writeScene(const ct::String& absolute);
    bool writeAssets();
    bool swapScene(const ct::String& text, bool keepOriginal);
    bool restoreParked();
    ct::Vector<ct::String> selectedPaths() const;
    void selectPaths(const ct::Vector<ct::String>& paths);

    SceneTree& mTree;
    EditorSelection& mSelection;
    SceneTreePanel& mHierarchy;
    Project& mProject;
    EditorSettings& mSettings;
    ct::String mSettingsPath;
    AssetSaver mAssetSaver;
    ProjectHandler mProjectLeaving;
    EditorUndoRedoManager mUndo;
    Node* mScene;
    Node* mParked;
    ct::String mParkedName;
    ct::String mScenePath;
    ct::String mError;
    SceneResult mResult;
    std::uint64_t mOrderHash;
    unsigned int mSerial;
    std::int64_t mSavedAt;
    ct::String mSnapshot;
    std::int64_t mSnapshotSavedAt;
    int mSuspended;
    bool mDirty;
    bool mOrderPending;
    bool mPlaying;
    bool mSnapshotDirty;
};

} // namespace prisma3d
