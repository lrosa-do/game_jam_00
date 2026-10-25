#include "prisma3d/editor/ProjectSettingsEditor.h"

#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/GamePanel.h"
#include "prisma3d/render/RenderingServer.h"

#include <igui/widgets/Theme.hpp>
#include <igui/widgets/WidgetApp.hpp>

#include "platform.h"

#include <math.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const char* const kShadowQualities = "Hard (Fastest),Soft Very Low (Faster),Soft Low (Fast),Soft "
                                     "Medium (Average),Soft High (Slow),Soft Ultra (Slowest)";
const char* const kSubdivisions =
        "Disabled,1 Shadow,4 Shadows,16 Shadows,64 Shadows,256 Shadows,1024 Shadows";

struct EnumHint
{
    const char* name;
    const char* hint;
};

const EnumHint kEnums[] = {
    { "display/window/size/mode", "Windowed,Minimized,Maximized,Fullscreen,Exclusive Fullscreen" },
    { "display/window/size/initial_position_type",
        "Center of Primary Screen:1,Center of Other Screen:3,Center of Screen With Mouse "
        "Pointer:4,Center of Screen With Keyboard Focus:5" },
    { "display/window/vsync/vsync_mode", "Disabled,Enabled,Mailbox" },
    { "rendering/anti_aliasing/quality/msaa_3d",
        "Disabled (Fastest),2x (Average),4x (Slow),8x (Slowest)" },
    { "rendering/scaling_3d/aspect", "Stretch,Keep,Crop,Expand" },
    { "rendering/scaling_3d/mode", "Bilinear (Fastest),FSR 1.0 (Fast)" },
    { "rendering/anti_aliasing/quality/screen_space_aa",
        "Disabled (Fastest),FXAA (Fast),SMAA (Average)" },
    { "rendering/camera/depth_of_field/depth_of_field_bokeh_shape",
        "Box (Fast),Hexagon (Average),Circle (Slowest)" },
    { "rendering/camera/depth_of_field/depth_of_field_bokeh_quality",
        "Very Low (Fastest),Low (Fast),Medium (Average),High (Slow)" },
    { "rendering/lights_and_shadows/directional_shadow/soft_shadow_filter_quality",
        kShadowQualities },
    { "rendering/lights_and_shadows/positional_shadow/soft_shadow_filter_quality",
        kShadowQualities },
    { "rendering/lights_and_shadows/positional_shadow/atlas_quadrant_0_subdiv", kSubdivisions },
    { "rendering/lights_and_shadows/positional_shadow/atlas_quadrant_1_subdiv", kSubdivisions },
    { "rendering/lights_and_shadows/positional_shadow/atlas_quadrant_2_subdiv", kSubdivisions },
    { "rendering/lights_and_shadows/positional_shadow/atlas_quadrant_3_subdiv", kSubdivisions },
};

struct RangeHint
{
    const char* name;
    double minimum;
    double maximum;
    double step;
    bool orLess;
    bool orGreater;
};

const RangeHint kRanges[] = {
    { "application/run/max_fps", 0.0, 1000.0, 1.0, false, false },
    { "application/run/frame_delay_msec", 0.0, 100.0, 1.0, false, true },
    { "application/run/low_processor_mode_sleep_usec", 0.0, 33200.0, 1.0, false, true },
    { "display/window/size/viewport_width", 1.0, 7680.0, 1.0, false, true },
    { "display/window/size/viewport_height", 1.0, 4320.0, 1.0, false, true },
    { "display/window/size/initial_screen", 0.0, 64.0, 1.0, false, true },
    { "display/window/size/window_width_override", 0.0, 7680.0, 1.0, false, true },
    { "display/window/size/window_height_override", 0.0, 4320.0, 1.0, false, true },
    { "physics/common/physics_ticks_per_second", 1.0, 1000.0, 1.0, false, false },
    { "physics/common/max_physics_steps_per_frame", 1.0, 100.0, 1.0, false, false },
    { "physics/common/physics_jitter_fix", 0.0, 2.0, 0.001, false, true },
    { "physics/3d/default_gravity", -32.0, 32.0, 0.001, true, true },
    { "rendering/anti_aliasing/screen_space_roughness_limiter/amount", 0.01, 4.0, 0.01, false,
        false },
    { "rendering/anti_aliasing/screen_space_roughness_limiter/limit", 0.01, 1.0, 0.01, false,
        false },
    { "rendering/scaling_3d/scale", 0.1, 2.0, 0.0001, false, false },
    { "rendering/scaling_3d/fsr_sharpness", 0.0, 2.0, 0.01, false, false },
    { "rendering/anti_aliasing/quality/smaa_edge_detection_threshold", 0.01, 0.2, 0.01, false,
        false },
    { "rendering/mesh_lod/lod_change/threshold_pixels", 0.0, 1024.0, 0.1, false, false },
    { "rendering/lights_and_shadows/directional_shadow/size", 256.0, 16384.0, 1.0, false, false },
    { "rendering/lights_and_shadows/positional_shadow/atlas_size", 256.0, 16384.0, 1.0, false,
        false },
};

const RangeHint* findRange(const ct::String& name)
{
    for (const RangeHint& hint: kRanges)
        if (name == hint.name) return &hint;
    return nullptr;
}

ct::String valueList(const ProjectSettingInfo& setting)
{
    ct::String list;
    for (int value = 0; value < 64; ++value)
    {
        if ((setting.allowedMask & (static_cast<std::uint64_t>(1) << value)) == 0) continue;
        if (!list.empty()) list += ',';
        list.append_number(value);
        list += ':';
        list.append_number(value);
    }
    return list;
}

} // namespace

ProjectSettingsWindow::ProjectSettingsWindow()
        : rt::FloatWindow(kTitle),
          mInspector(nullptr),
          mSearch(nullptr),
          mRestartBar(nullptr),
          mRestartLabel(nullptr),
          mRestartClose(nullptr),
          mClose(nullptr)
{
    setMinimizable(false);

    rt::BoxLayout* main = setContent<rt::BoxLayout>(rt::LayoutDir::Vertical);
    main->setSpacing(8.0f);
    main->setPadding(10.0f);

    mSearch = main->createChild<rt::TextInput>();
    mSearch->setPlaceholder(kSearchPlaceholder);

    mInspector = main->createChild<SectionedInspector>();
    mInspector->setStretch(1.0f);

    mRestartBar = main->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    mRestartBar->setPadding(4.0f);
    mRestartBar->setSpacing(8.0f);
    mRestartBar->setVisible(false);
    mRestartLabel = mRestartBar->createChild<rt::Label>(kRestartText);
    mRestartLabel->setStretch(1.0f);
    mRestartLabel->setColor(rt::Color(255, 204, 0, 255));
    mRestartClose = mRestartBar->createChild<rt::Button>();
    mRestartClose->setIconId(EditorIcons::get("Close"));
    mRestartClose->setSize(34.0f, 0.0f);
    mRestartClose->setTooltip("Close");

    rt::BoxLayout* buttons = main->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    buttons->setPadding(0.0f);
    buttons->setSpacing(8.0f);
    buttons->setMainAlign(rt::MainAlign::End);
    mClose = buttons->createChild<rt::Button>("Close");
    mClose->setSize(90.0f, 0.0f);

    mSearch->textChanged.connect([this](const ct::String& text) { mInspector->setFilter(text); });
    mRestartClose->clicked.connect([this] { showRestart(false); });
    mClose->clicked.connect([this] { closeRequested.emit(); });
    closed.connect([this] { closeRequested.emit(); });

    const rt::IO& io = rt::GetIO();
    float width = kWidth;
    float height = kHeight;
    if (io.displayWidth > 0.0f && width > io.displayWidth * kScreenShare)
        width = io.displayWidth * kScreenShare;
    if (io.displayHeight > 0.0f && height > io.displayHeight * kScreenShare)
        height = io.displayHeight * kScreenShare;
    setFloatSize(width, height);
    setFloatPos((io.displayWidth - width) * 0.5f, (io.displayHeight - height) * 0.5f);
}

SectionedInspector* ProjectSettingsWindow::inspector() const
{
    return mInspector;
}

rt::TextInput* ProjectSettingsWindow::searchBox() const
{
    return mSearch;
}

rt::Button* ProjectSettingsWindow::closeButton() const
{
    return mClose;
}

rt::BoxLayout* ProjectSettingsWindow::restartBar() const
{
    return mRestartBar;
}

rt::Label* ProjectSettingsWindow::restartLabel() const
{
    return mRestartLabel;
}

rt::Button* ProjectSettingsWindow::restartCloseButton() const
{
    return mRestartClose;
}

void ProjectSettingsWindow::showRestart(bool shown)
{
    if (mRestartBar->isVisible() == shown) return;
    mRestartBar->setVisible(shown);
    if (rt::WidgetApp* app = rt::WidgetApp::existing()) app->requestLayout();
}

bool ProjectSettingsWindow::restartShown() const
{
    return mRestartBar->isVisible();
}

void ProjectSettingsWindow::onKeyPress(rt::KeyEvent& event)
{
    if (event.key == rt::Key::Escape)
    {
        event.consumed = true;
        closeRequested.emit();
        return;
    }
    rt::FloatWindow::onKeyPress(event);
}

rt::Widget* ProjectSettingsWindow::focusOnOpen()
{
    return mSearch;
}

ProjectSettingsEditor::ProjectSettingsEditor(EditorDocument& document)
        : mDocument(document),
          mLink(ct::make_rc<Link>(Link{ this })),
          mServer(nullptr),
          mScene(nullptr),
          mGame(nullptr),
          mWindow(nullptr),
          mTimer(0.0f),
          mSaves(0),
          mApplies(0),
          mPending(false),
          mApplyPending(false),
          mApplied(false)
{
    const ct::Rc<Link> link = mLink;
    document.setProjectLeavingHandler(
            [link]
            {
                if (link->owner) link->owner->flush();
            });
}

ProjectSettingsEditor::~ProjectSettingsEditor()
{
    mDocument.setProjectLeavingHandler(EditorDocument::ProjectHandler());
    mLink->owner = nullptr;
    flush();
    close();
}

void ProjectSettingsEditor::bindRendering(RenderingServer& server, EditorViewportPanel* scene,
        GamePanel* game)
{
    mServer = &server;
    mScene = scene;
    mGame = game;
}

bool ProjectSettingsEditor::isLive(const ct::String& name)
{
    return name.starts_with("rendering/");
}

PropertyInfo ProjectSettingsEditor::describe(const ProjectSettingInfo& setting)
{
    const char* const name = setting.name.c_str();
    switch (setting.type)
    {
        case PropertyType::Bool:
            return PropertyInfo(PropertyType::Bool, name);
        case PropertyType::Vec3:
            return PropertyInfo(PropertyType::Vec3, name);
        case PropertyType::Color:
            return PropertyInfo::color(name);
        case PropertyType::Int:
        case PropertyType::Float:
        {
            if (setting.allowedMask != 0)
            {
                ct::String hint = valueList(setting);
                for (const EnumHint& known: kEnums)
                    if (setting.name == known.name) hint = known.hint;
                return PropertyInfo::enumeration(name, hint.c_str());
            }
            const bool integer = setting.type == PropertyType::Int;
            double minimum = setting.minimum;
            double maximum = setting.maximum;
            double step = integer ? 1.0 : 0.001;
            bool orLess = false;
            bool orGreater = false;
            if (const RangeHint* hint = findRange(setting.name))
            {
                minimum = hint->minimum;
                maximum = hint->maximum;
                step = hint->step;
                orLess = hint->orLess;
                orGreater = hint->orGreater;
            }
            if (setting.limited)
            {
                if (isfinite(setting.minimum))
                {
                    if (minimum < setting.minimum) minimum = setting.minimum;
                    orLess = false;
                }
                if (isfinite(setting.maximum))
                {
                    if (maximum > setting.maximum) maximum = setting.maximum;
                    orGreater = false;
                }
            }
            if (!isfinite(minimum) || !isfinite(maximum))
            {
                minimum = 0.0;
                maximum = 0.0;
            }
            return PropertyInfo::range(setting.type, name, minimum, maximum, step, orGreater,
                    orLess);
        }
        default:
            return PropertyInfo(setting.type, name);
    }
}

ct::Vector<PropertyInfo> ProjectSettingsEditor::properties()
{
    ct::Vector<PropertyInfo> list;
    for (std::size_t i = 0; i < Project::settingCount(); ++i)
        list.push_back(describe(Project::settingInfo(i)));
    return list;
}

SectionedInspector::Source ProjectSettingsEditor::source()
{
    const ct::Rc<Link> link = mLink;
    SectionedInspector::Source result;
    result.get = [link](const ct::String& name)
    { return link->owner ? link->owner->mDocument.project().get(name) : PropertyValue(); };
    result.set = [link](const ct::String& name, const PropertyValue& value)
    { return link->owner && link->owner->setValue(name, value); };
    result.isDefault = [link](const ct::String& name)
    { return !link->owner || link->owner->mDocument.project().isDefault(name); };
    result.revert = [link](const ct::String& name)
    {
        if (link->owner) link->owner->resetValue(name);
    };
    return result;
}

bool ProjectSettingsEditor::canOpen() const
{
    return mDocument.project().isOpen();
}

void ProjectSettingsEditor::open()
{
    if (!canOpen()) return;
    rt::WidgetApp& app = rt::WidgetApp::instance();
    if (mWindow)
    {
        mWindow->setVisible(true);
        app.bringToFront(mWindow);
        return;
    }

    mWindow = app.addFloat<ProjectSettingsWindow>();
    SectionedInspector* inspector = mWindow->inspector();
    inspector->setSource(source());
    inspector->setProperties(properties());
    if (!mSection.empty()) inspector->setCurrentSection(mSection);
    const ct::Rc<Link> link = mLink;
    inspector->categoryChanged.connect(
            [link](const ct::String& section)
            {
                if (link->owner && !section.empty()) link->owner->mSection = section;
            });
    if (!inspector->currentSection().empty()) mSection = inspector->currentSection();
    mWindow->closeRequested.connect(
            [link]
            {
                if (link->owner) link->owner->close();
            });
}

void ProjectSettingsEditor::close()
{
    if (!mWindow) return;
    flush();
    ProjectSettingsWindow* window = mWindow;
    mWindow = nullptr;
    if (rt::WidgetApp* app = rt::WidgetApp::existing()) app->removeFloat(window);
}

bool ProjectSettingsEditor::isOpen() const
{
    return mWindow != nullptr;
}

ProjectSettingsWindow* ProjectSettingsEditor::window() const
{
    return mWindow;
}

bool ProjectSettingsEditor::setValue(const ct::String& name, const PropertyValue& value)
{
    Project& project = mDocument.project();
    if (!project.isOpen()) return false;
    if (!project.set(name, value))
    {
        mError = project.error();
        return false;
    }
    mError = ct::String();
    changed(name);
    return true;
}

void ProjectSettingsEditor::resetValue(const ct::String& name)
{
    Project& project = mDocument.project();
    if (!project.isOpen()) return;
    project.reset(name);
    changed(name);
}

void ProjectSettingsEditor::changed(const ct::String& name)
{
    queueSave();
    if (isLive(name))
    {
        if (mWindow && mWindow->inspector()->anyActive()) mApplyPending = true;
        else
            applyLive();
    }
    else if (mWindow)
        mWindow->showRestart(true);
}

void ProjectSettingsEditor::queueSave()
{
    mPending = true;
    mTimer = kSaveDelay;
}

bool ProjectSettingsEditor::savePending() const
{
    return mPending;
}

bool ProjectSettingsEditor::applyPending() const
{
    return mApplyPending;
}

unsigned int ProjectSettingsEditor::saveCount() const
{
    return mSaves;
}

unsigned int ProjectSettingsEditor::applyCount() const
{
    return mApplies;
}

const ct::String& ProjectSettingsEditor::error() const
{
    return mError;
}

bool ProjectSettingsEditor::flush()
{
    if (!mPending) return true;
    mPending = false;
    Project& project = mDocument.project();
    if (!project.isOpen()) return false;
    if (!project.save())
    {
        mError = project.error();
        log_error("project settings: %s", mError.c_str());
        return false;
    }
    ++mSaves;
    return true;
}

void ProjectSettingsEditor::applyLive()
{
    mApplyPending = false;
    if (!mServer) return;
    const Project& project = mDocument.project();
    project.applyToRendering(*mServer);
    if (mScene && mScene->viewportId().valid())
        project.applyToViewport(*mServer, mScene->viewportId(), false);
    if (mGame && mGame->viewportId().valid())
    {
        project.applyToViewport(*mServer, mGame->viewportId(), false);
        if (project.isOpen()) mGame->setClearColor(project.get(kClearColorSetting).asColor());
    }
    ++mApplies;
}

void ProjectSettingsEditor::syncProject()
{
    const Project& project = mDocument.project();
    const ct::String path = project.isOpen() ? project.manifestPath() : ct::String();
    if (mApplied && path == mProjectPath) return;
    mApplied = true;
    mProjectPath = path;
    applyLive();
}

void ProjectSettingsEditor::update(float delta)
{
    syncProject();
    if (mWindow && !canOpen())
    {
        close();
        return;
    }
    if (mWindow)
    {
        mWindow->inspector()->update();
        if (mApplyPending && !mWindow->inspector()->anyActive()) applyLive();
    }
    else if (mApplyPending)
        applyLive();
    if (mPending)
    {
        mTimer -= delta;
        if (mTimer <= 0.0f) flush();
    }
}

} // namespace prisma3d
