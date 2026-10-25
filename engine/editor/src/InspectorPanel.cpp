#include "prisma3d/editor/InspectorPanel.h"

#include "prisma3d/editor/EditorTreeEdits.h"
#include "prisma3d/editor/EditorUndoRedoManager.h"
#include "prisma3d/editor/NodeNaming.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneTree.h"

#include <igui/widgets/DialogWidgets.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Theme.hpp>
#include <igui/widgets/WidgetApp.hpp>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const char* const kRowNames[InspectorPanel::kRowCount] = { "Position", "Rotation", "Scale" };
const char* const kAxisNames[InspectorPanel::kAxisCount] = { "X", "Y", "Z" };
const float kFieldStart[InspectorPanel::kRowCount] = { 0.0f, 0.0f, 1.0f };
const float kSensitivity[InspectorPanel::kRowCount] = { 0.05f, 0.5f, 0.01f };
const char* const kResetTooltips[InspectorPanel::kRowCount] = { "Reset position to 0, 0, 0",
    "Reset rotation to 0, 0, 0", "Reset scale to 1, 1, 1" };
const float kHeaderHeight = 36.0f;
const float kHeaderIcon = 24.0f;

rt::Color axisColor(int axis)
{
    switch (axis)
    {
        case 0:
            return rt::Color(226, 86, 86, 255);
        case 1:
            return rt::Color(124, 200, 92, 255);
        default:
            return rt::Color(88, 142, 236, 255);
    }
}

float component(const Math::Vec3& vector, int axis)
{
    return axis == 0 ? vector.x : (axis == 1 ? vector.y : vector.z);
}

ct::String labelOf(const Node* node)
{
    return node->getName().empty() ? ct::String(node->getClassName()) : node->getName();
}

const char* const kRenameAction = "Rename Node";
const char* const kVisibleAction = "Set visible";
const char* const kActiveAction = "Set active";
const char* const kTagAction = "Set tag";
const char* const kSetPosition = "Set position";
const char* const kSetRotation = "Set rotation";
const char* const kSetScale = "Set scale";

void requestLayout()
{
    if (rt::WidgetApp* app = rt::WidgetApp::existing()) app->requestLayout();
}

} // namespace

class InspectorHeader : public rt::Widget
{
public:
    IG_WIDGET_CLASS(InspectorHeader, rt::Widget);

    void setInfo(const ct::String& name, const ct::String& className, rt::IconId icon)
    {
        mName = name;
        mClass = className;
        mIcon = icon;
        markDirty();
    }

    const ct::String& name() const
    {
        return mName;
    }

    const ct::String& className() const
    {
        return mClass;
    }

    rt::IconId icon() const
    {
        return mIcon;
    }

    Vec2f sizeHint() const override
    {
        return { 160.0f, kHeaderHeight };
    }

    void paint(rt::PaintContext& ctx) override
    {
        if (!visible_) return;
        const rt::Rect abs = absoluteRect();
        if (ctx.isClipped(abs)) return;

        const rt::Theme& theme = rt::Theme::instance();
        ctx.drawIcon(mIcon, abs.x + 6.0f, abs.y + (abs.h - kHeaderIcon) * 0.5f, kHeaderIcon);

        if (overrideFont_) ctx.pushFont(overrideFont_);
        ctx.pushClip(abs);
        ctx.font.SetFontSize(theme.fontSize);
        ctx.font.SetBatch(&ctx.text);
        const float ascender = ctx.font.GetAscender();
        const float left = abs.x + kHeaderIcon + 14.0f;
        ctx.font.SetColor(theme.textColor);
        ctx.font.Print(mClass.c_str(), left, abs.y + (abs.h - theme.fontSize) * 0.5f + ascender);
        ctx.popClip();
        if (overrideFont_) ctx.popFont();
    }

private:
    ct::String mName;
    ct::String mClass;
    rt::IconId mIcon = rt::IconId::None;
};

InspectorPanel::InspectorPanel(SceneTree& tree, EditorSelection& selection)
        : mTree(tree),
          mSelection(selection),
          mLink(ct::make_rc<Link>(Link{ this })),
          mScroll(nullptr),
          mRoot(nullptr),
          mEmpty(nullptr),
          mHeader(nullptr),
          mIdentity(nullptr),
          mName(nullptr),
          mVisible(nullptr),
          mActive(nullptr),
          mTag(nullptr),
          mTransform(nullptr),
          mFields{},
          mResets{},
          mListener(nullptr),
          mNode(nullptr),
          mWrites(0),
          mDirty(true),
          mSyncing(false)
{
    mTree.addListener(this);
    mSelection.addListener(this);
}

InspectorPanel::~InspectorPanel()
{
    mSelection.removeListener(this);
    mTree.removeListener(this);
    mLink->owner = nullptr;
}

bool InspectorPanel::attach(rt::Widget* parent)
{
    if (mRoot || !parent) return false;

    mScroll = parent->createChild<rt::ScrollView>();
    mRoot = mScroll->setContent<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mRoot->setPadding(8.0f);
    mRoot->setSpacing(6.0f);

    mEmpty = mRoot->createChild<rt::Label>("No selection");
    mHeader = mRoot->createChild<InspectorHeader>();
    mHeader->setVisible(false);

    const ct::Rc<Link> identityLink = mLink;
    mIdentity = mRoot->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mIdentity->setPadding(0.0f);
    mIdentity->setSpacing(4.0f);
    mIdentity->setVisible(false);

    rt::BoxLayout* nameRow = mIdentity->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    nameRow->setPadding(0.0f);
    nameRow->setSpacing(4.0f);
    nameRow->createChild<NameLabel>("Name")->setStretch(3.0f);
    mName = nameRow->createChild<CommitInput>();
    mName->setStretch(7.0f);
    mName->committed.connect(
            [identityLink](const ct::String& text)
            {
                if (identityLink->owner) identityLink->owner->nameCommitted(text);
            });

    rt::BoxLayout* flagRow = mIdentity->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    flagRow->setPadding(0.0f);
    flagRow->setSpacing(12.0f);
    mVisible = flagRow->createChild<rt::CheckBox>("Visible");
    mVisible->toggled.connect(
            [identityLink](bool checked)
            {
                if (identityLink->owner) identityLink->owner->visibleToggled(checked);
            });
    mActive = flagRow->createChild<rt::CheckBox>("Active");
    mActive->setTooltip(kActiveTooltip);
    mActive->toggled.connect(
            [identityLink](bool checked)
            {
                if (identityLink->owner) identityLink->owner->activeToggled(checked);
            });

    rt::BoxLayout* tagRow = mIdentity->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    tagRow->setPadding(0.0f);
    tagRow->setSpacing(4.0f);
    NameLabel* tagLabel = tagRow->createChild<NameLabel>("Tag");
    tagLabel->setStretch(3.0f);
    tagLabel->setTooltip(kTagTooltip);
    mTag = tagRow->createChild<CommitInput>();
    mTag->setStretch(7.0f);
    mTag->setTooltip(kTagTooltip);
    mTag->committed.connect(
            [identityLink](const ct::String& text)
            {
                if (identityLink->owner) identityLink->owner->tagCommitted(text);
            });

    mTransform = mRoot->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mTransform->setPadding(0.0f);
    mTransform->setSpacing(4.0f);
    mTransform->setVisible(false);
    mTransform->createChild<rt::Label>("Transform");
    mTransform->createChild<rt::Line>();

    const ct::Rc<Link> link = mLink;
    for (int row = 0; row < kRowCount; ++row)
    {
        rt::BoxLayout* box = mTransform->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
        box->setPadding(0.0f);
        box->setSpacing(2.0f);
        rt::BoxLayout* header = box->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
        header->setPadding(0.0f);
        header->setSpacing(4.0f);
        rt::Label* title = header->createChild<rt::Label>(kRowNames[row]);
        title->setStretch(1.0f);
        RevertButton* reset = header->createChild<RevertButton>();
        reset->setActive(true);
        reset->setTooltip(kResetTooltips[row]);
        reset->reverted.connect(
                [link, row]
                {
                    if (link->owner) link->owner->resetRow(row);
                });
        mResets[row] = reset;

        rt::BoxLayout* fields = box->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
        fields->setPadding(0.0f);
        fields->setSpacing(4.0f);

        for (int axis = 0; axis < kAxisCount; ++axis)
        {
            NumberField* field = fields->createChild<NumberField>(kAxisNames[axis]);
            field->setStretch(1.0f);
            field->setTagColor(axisColor(axis));
            field->setDragSensitivity(kSensitivity[row]);
            field->setValue(kFieldStart[row]);
            field->valueChanged.connect(
                    [link, row](float)
                    {
                        if (link->owner) link->owner->fieldChanged(row);
                    });
            mFields[row][axis] = field;
        }
    }

    mProperties.attach(mRoot);
    mProperties.setClassIcons(&mIcons);
    mProperties.setChangeListener(mListener);

    mDirty = true;
    return true;
}

void InspectorPanel::setChangeListener(EditorChangeListener* listener)
{
    mListener = listener;
    mProperties.setChangeListener(listener);
}

EditorUndoRedoManager* InspectorPanel::undoRedo() const
{
    return mListener ? mListener->undoRedoManager() : nullptr;
}

void InspectorPanel::setRenameHandler(const RenameHandler& handler)
{
    mRename = handler;
}

void InspectorPanel::setSettings(EditorSettings* settings)
{
    mProperties.setSettings(settings);
}

void InspectorPanel::setProject(const Project* project)
{
    mProperties.setProject(project);
}

PropertyInspector& InspectorPanel::properties()
{
    return mProperties;
}

const PropertyInspector& InspectorPanel::properties() const
{
    return mProperties;
}

rt::ScrollView* InspectorPanel::scrollView() const
{
    return mScroll;
}

bool InspectorPanel::attached() const
{
    return mRoot != nullptr;
}

void InspectorPanel::nodeRemoved(Node* node)
{
    mProperties.forget(node);
    if (node != mNode) return;
    mNode = nullptr;
    mDirty = true;
}

void InspectorPanel::selectionChanged()
{
    mDirty = true;
}

void InspectorPanel::update()
{
    if (!mRoot) return;

    const ct::Vector<Node*>& selected = mSelection.selectedNodes();
    Node* wanted = selected.empty() ? nullptr : selected[0];
    const bool changed = wanted != mNode || (mDirty && !mNode);
    mDirty = false;
    if (changed)
    {
        show(wanted);
        return;
    }
    if (!mNode) return;

    refreshHeader(false);
    refreshIdentity();
    if (const Node3D* spatial = mNode->castTo<Node3D>()) refreshTransform(spatial, false);
    mProperties.update();
}

void InspectorPanel::show(Node* node)
{
    releaseEditors();
    mNode = node;

    const Node3D* spatial = node ? node->castTo<Node3D>() : nullptr;
    mEmpty->setVisible(node == nullptr);
    mHeader->setVisible(node != nullptr);
    mIdentity->setVisible(node != nullptr);
    mVisible->setVisible(spatial != nullptr);
    mTransform->setVisible(spatial != nullptr);
    mProperties.show(node);
    if (mScroll) mScroll->setScrollY(0.0f);
    requestLayout();

    if (!node) return;
    refreshHeader(true);
    refreshIdentity();
    if (spatial) refreshTransform(spatial, true);
}

ct::String InspectorPanel::tagOf(const Node* node)
{
    ct::Vector<Node::GroupInfo> groups;
    node->getGroups(groups);
    ct::String text;
    for (std::size_t i = 0; i < groups.size(); ++i)
    {
        if (!groups[i].persistent) continue;
        if (!text.empty()) text += ", ";
        text += groups[i].name;
    }
    return text;
}

void InspectorPanel::refreshIdentity()
{
    if (!mNode) return;
    const bool wasSyncing = mSyncing;
    mSyncing = true;
    if (!mName->isFocused() && mName->committedText() != mNode->getName())
        mName->setCommittedText(mNode->getName());
    const ct::String tag = tagOf(mNode);
    if (!mTag->isFocused() && mTag->committedText() != tag) mTag->setCommittedText(tag);
    if (const Node3D* spatial = mNode->castTo<Node3D>())
        if (mVisible->isChecked() != spatial->isVisible())
            mVisible->setChecked(spatial->isVisible());
    const bool active = mNode->getProcessMode() != Node::ProcessMode::Disabled;
    if (mActive->isChecked() != active) mActive->setChecked(active);
    mSyncing = wasSyncing;
}

void InspectorPanel::nameCommitted(const ct::String& text)
{
    if (mSyncing || !mNode || text == mNode->getName()) return;
    ct::String error;
    if (mRename) mRename(mNode, text, error);
    else
    {
        const ct::String name = NodeNaming::resolve(mNode, text, error);
        if (name != mNode->getName())
            EditorTreeEdits::setProperty(undoRedo(), kRenameAction, mNode, "name",
                    PropertyValue::fromString(mNode->getName()), PropertyValue::fromString(name));
    }
    const bool wasSyncing = mSyncing;
    mSyncing = true;
    mName->setCommittedText(mNode->getName());
    mSyncing = wasSyncing;
    if (!undoRedo() && mListener) mListener->sceneEdited();
    if (!error.empty()) rt::AlertDialog::show("Error", error);
}

void InspectorPanel::visibleToggled(bool visible)
{
    if (mSyncing || !mNode) return;
    Node3D* spatial = mNode->castTo<Node3D>();
    if (!spatial || spatial->isVisible() == visible) return;
    EditorTreeEdits::setProperty(undoRedo(), kVisibleAction, spatial, "visible",
            PropertyValue::fromBool(!visible), PropertyValue::fromBool(visible));
    if (!undoRedo() && mListener) mListener->sceneEdited();
}

void InspectorPanel::activeToggled(bool active)
{
    if (mSyncing || !mNode) return;
    const Node::ProcessMode mode =
            active ? Node::ProcessMode::Inherit : Node::ProcessMode::Disabled;
    const Node::ProcessMode previous = mNode->getProcessMode();
    if (previous == mode) return;
    EditorUndoRedoManager* undo = undoRedo();
    if (!undo)
    {
        mNode->setProcessMode(mode);
        if (mListener) mListener->sceneEdited();
        return;
    }
    Node* node = mNode;
    undo->createAction(kActiveAction);
    undo->addDoMethod([node, mode] { node->setProcessMode(mode); }, node);
    undo->addUndoMethod([node, previous] { node->setProcessMode(previous); }, node);
    undo->commitAction();
}

void InspectorPanel::tagCommitted(const ct::String& text)
{
    if (mSyncing || !mNode) return;
    ct::Vector<ct::String> wanted;
    std::size_t start = 0;
    while (start <= text.size())
    {
        std::size_t end = text.find(',', start);
        if (end == ct::String::npos) end = text.size();
        const ct::String name = text.substr(start, end - start).trimmed();
        bool known = false;
        for (std::size_t i = 0; i < wanted.size(); ++i) known = known || wanted[i] == name;
        if (!name.empty() && !known) wanted.push_back(name);
        start = end + 1;
    }

    const ct::Vector<ct::String> previous = EditorTreeEdits::persistentGroups(mNode);
    bool changed = previous.size() != wanted.size();
    for (std::size_t i = 0; i < wanted.size() && !changed; ++i)
    {
        bool known = false;
        for (std::size_t j = 0; j < previous.size(); ++j) known = known || previous[j] == wanted[i];
        changed = !known;
    }
    if (changed)
    {
        EditorUndoRedoManager* undo = undoRedo();
        Node* node = mNode;
        if (undo)
        {
            undo->createAction(kTagAction);
            undo->addDoMethod([node, wanted]
                    { EditorTreeEdits::setPersistentGroups(node, wanted); }, node);
            undo->addUndoMethod([node, previous]
                    { EditorTreeEdits::setPersistentGroups(node, previous); }, node);
            undo->commitAction();
        }
        else
            EditorTreeEdits::setPersistentGroups(node, wanted);
    }

    const bool wasSyncing = mSyncing;
    mSyncing = true;
    mTag->setCommittedText(tagOf(mNode));
    mSyncing = wasSyncing;
    if (changed && !undoRedo() && mListener) mListener->sceneEdited();
}

void InspectorPanel::refreshHeader(bool force)
{
    if (!mNode) return;
    const ct::String name = labelOf(mNode);
    const ct::String className(mNode->getClassName());
    const rt::IconId icon = mIcons.icon(mNode);
    if (!force && name == mHeader->name() && className == mHeader->className() &&
            icon == mHeader->icon())
        return;
    mHeader->setInfo(name, className, icon);
}

void InspectorPanel::refreshTransform(const Node3D* spatial, bool force)
{
    const Math::Vec3 values[kRowCount] = { spatial->getPosition(), spatial->getRotationDegrees(),
        spatial->getScale() };

    const bool wasSyncing = mSyncing;
    mSyncing = true;
    for (int row = 0; row < kRowCount; ++row)
    {
        for (int axis = 0; axis < kAxisCount; ++axis)
        {
            const float value = component(values[row], axis);
            NumberField* field = mFields[row][axis];
            if (force || !field->isActive()) field->setValue(value);
        }
    }
    mSyncing = wasSyncing;
}

void InspectorPanel::fieldChanged(int row)
{
    if (mSyncing || !mNode || !mNode->castTo<Node3D>()) return;
    applyRow(row);
}

void InspectorPanel::applyRow(int row)
{
    Node3D* spatial = mNode->castTo<Node3D>();
    const Math::Vec3 value(mFields[row][0]->value(), mFields[row][1]->value(),
            mFields[row][2]->value());
    EditorUndoRedoManager* undo = undoRedo();
    switch (row)
    {
        case kPosition:
            if (undo)
                EditorTreeEdits::setProperty(undo, kSetPosition, spatial, "position",
                        PropertyValue::fromVec3(spatial->getPosition()),
                        PropertyValue::fromVec3(value));
            else
                spatial->setPosition(value);
            break;
        case kRotation:
            if (undo)
            {
                undo->createPropertyAction(kSetRotation, spatial);
                undo->addDoProperty(spatial, "rotation_degrees", PropertyValue::fromVec3(value));
                undo->addUndoProperty(spatial, "rotation",
                        PropertyValue::fromVec3(spatial->getRotation()));
                undo->commitAction();
            }
            else
                spatial->setRotationDegrees(value);
            break;
        default:
            if (undo)
                EditorTreeEdits::setProperty(undo, kSetScale, spatial, "scale",
                        PropertyValue::fromVec3(spatial->getScale()),
                        PropertyValue::fromVec3(value));
            else
                spatial->setScale(value);
            break;
    }
    ++mWrites;
    refreshTransform(spatial, false);
    if (!undo && mListener) mListener->sceneEdited();
}

void InspectorPanel::resetRow(int row)
{
    if (!mNode || !mNode->castTo<Node3D>()) return;
    const float value = row == kScale ? 1.0f : 0.0f;
    const bool wasSyncing = mSyncing;
    mSyncing = true;
    for (int axis = 0; axis < kAxisCount; ++axis)
    {
        mFields[row][axis]->setValue(value);
    }
    mSyncing = wasSyncing;
    applyRow(row);
}

void InspectorPanel::releaseEditors()
{
    mProperties.releaseEditors();
    for (int row = 0; row < kRowCount; ++row)
        for (int axis = 0; axis < kAxisCount; ++axis)
            if (mFields[row][axis] && mFields[row][axis]->isEditing())
                mFields[row][axis]->cancelEdit();
}

Node* InspectorPanel::inspected() const
{
    return mNode;
}

ClassIcons& InspectorPanel::icons()
{
    return mIcons;
}

const ClassIcons& InspectorPanel::icons() const
{
    return mIcons;
}

bool InspectorPanel::showsNoSelection() const
{
    return mEmpty && mEmpty->isVisible();
}

bool InspectorPanel::showsHeader() const
{
    return mHeader && mHeader->isVisible();
}

bool InspectorPanel::showsIdentity() const
{
    return mIdentity && mIdentity->isVisible();
}

CommitInput* InspectorPanel::nameInput() const
{
    return mName;
}

rt::CheckBox* InspectorPanel::visibleBox() const
{
    return mVisible;
}

rt::CheckBox* InspectorPanel::activeBox() const
{
    return mActive;
}

CommitInput* InspectorPanel::tagInput() const
{
    return mTag;
}

bool InspectorPanel::showsTransform() const
{
    return mTransform && mTransform->isVisible();
}

const ct::String& InspectorPanel::headerName() const
{
    return mHeader->name();
}

const ct::String& InspectorPanel::headerClass() const
{
    return mHeader->className();
}

rt::IconId InspectorPanel::headerIcon() const
{
    return mHeader->icon();
}

const rt::Widget* InspectorPanel::headerWidget() const
{
    return mHeader;
}

NumberField* InspectorPanel::field(int row, int axis) const
{
    if (row < 0 || row >= kRowCount || axis < 0 || axis >= kAxisCount) return nullptr;
    return mFields[row][axis];
}

RevertButton* InspectorPanel::resetButton(int row) const
{
    if (row < 0 || row >= kRowCount) return nullptr;
    return mResets[row];
}

const char* InspectorPanel::resetTooltip(int row)
{
    return row >= 0 && row < kRowCount ? kResetTooltips[row] : "";
}

float InspectorPanel::dragSensitivity(int row)
{
    return kSensitivity[row];
}

unsigned int InspectorPanel::writeCount() const
{
    return mWrites;
}

} // namespace prisma3d
