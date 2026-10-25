#pragma once

#include "prisma3d/editor/SectionedInspector.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/scene/NodeClassDB.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Signal.hpp>
#include <igui/widgets/TextInputWidgets.hpp>
#include <igui/widgets/ViewWidgets.hpp>

#include <ct/ptr.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class EditorDocument;
class EditorViewportPanel;
class GamePanel;
class RenderingServer;

class ProjectSettingsWindow : public ig::retained::FloatWindow
{
public:
    IG_WIDGET_CLASS(ProjectSettingsWindow, ig::retained::FloatWindow);

    static constexpr const char* kTitle = "Project Settings (projecto.json)";
    static constexpr const char* kSearchPlaceholder = "Filter Settings";
    static constexpr const char* kRestartText =
            "Changed settings will be applied after restarting.";
    static constexpr float kWidth = 1200.0f;
    static constexpr float kHeight = 700.0f;
    static constexpr float kScreenShare = 0.8f;

    ProjectSettingsWindow();

    SectionedInspector* inspector() const;
    ig::retained::TextInput* searchBox() const;
    ig::retained::Button* closeButton() const;
    ig::retained::BoxLayout* restartBar() const;
    ig::retained::Label* restartLabel() const;
    ig::retained::Button* restartCloseButton() const;

    void showRestart(bool shown);
    bool restartShown() const;

    ig::retained::Signal<> closeRequested;

    void onKeyPress(ig::retained::KeyEvent& event) override;
    ig::retained::Widget* focusOnOpen() override;

private:
    SectionedInspector* mInspector;
    ig::retained::TextInput* mSearch;
    ig::retained::BoxLayout* mRestartBar;
    ig::retained::Label* mRestartLabel;
    ig::retained::Button* mRestartClose;
    ig::retained::Button* mClose;
};

class ProjectSettingsEditor
{
public:
    static constexpr float kSaveDelay = 1.5f;
    static constexpr const char* kClearColorSetting =
            "rendering/environment/defaults/default_clear_color";

    explicit ProjectSettingsEditor(EditorDocument& document);
    ~ProjectSettingsEditor();
    ProjectSettingsEditor(const ProjectSettingsEditor&) = delete;
    ProjectSettingsEditor& operator=(const ProjectSettingsEditor&) = delete;

    void bindRendering(RenderingServer& server, EditorViewportPanel* scene, GamePanel* game);

    bool canOpen() const;
    void open();
    void close();
    bool isOpen() const;
    ProjectSettingsWindow* window() const;

    void update(float delta);
    bool flush();
    bool applyPending() const;
    void queueSave();
    bool savePending() const;
    unsigned int saveCount() const;
    unsigned int applyCount() const;
    const ct::String& error() const;

    void applyLive();

    static bool isLive(const ct::String& name);
    static PropertyInfo describe(const ProjectSettingInfo& setting);
    static ct::Vector<PropertyInfo> properties();

private:
    struct Link
    {
        ProjectSettingsEditor* owner;
    };

    SectionedInspector::Source source();
    bool setValue(const ct::String& name, const PropertyValue& value);
    void resetValue(const ct::String& name);
    void changed(const ct::String& name);
    void syncProject();

    EditorDocument& mDocument;
    ct::Rc<Link> mLink;
    RenderingServer* mServer;
    EditorViewportPanel* mScene;
    GamePanel* mGame;
    ProjectSettingsWindow* mWindow;
    ct::String mSection;
    ct::String mProjectPath;
    ct::String mError;
    float mTimer;
    unsigned int mSaves;
    unsigned int mApplies;
    bool mPending;
    bool mApplyPending;
    bool mApplied;
};

} // namespace prisma3d
