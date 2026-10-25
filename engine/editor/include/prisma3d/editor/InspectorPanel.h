#pragma once

#include "prisma3d/editor/ClassIcons.h"
#include "prisma3d/editor/EditorChange.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/NumberField.h"
#include "prisma3d/editor/PropertyInspector.h"
#include "prisma3d/editor/PropertyWidgets.h"
#include "prisma3d/scene/SceneTreeListener.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/InputWidgets.hpp>
#include <igui/widgets/ScrollWidgets.hpp>

#include <ct/function.hpp>
#include <ct/ptr.hpp>
#include <ct/string.hpp>

namespace prisma3d
{

class InspectorHeader;
class Node;
class Node3D;
class Project;
class SceneTree;
class EditorUndoRedoManager;

class InspectorPanel : private SceneTreeListener, private EditorSelectionListener
{
public:
    static constexpr int kRowCount = 3;
    static constexpr int kAxisCount = 3;
    enum Row : int
    {
        kPosition = 0,
        kRotation = 1,
        kScale = 2
    };

    InspectorPanel(SceneTree& tree, EditorSelection& selection);
    ~InspectorPanel() override;
    InspectorPanel(const InspectorPanel&) = delete;
    InspectorPanel& operator=(const InspectorPanel&) = delete;

    bool attach(ig::retained::Widget* parent);
    bool attached() const;

    using RenameHandler = ct::Function<bool(Node*, const ct::String&, ct::String&)>;

    static constexpr const char* kActiveTooltip =
            "Inactive nodes and their children stop processing, and physics bodies leave the "
            "simulation.";
    static constexpr const char* kTagTooltip = "Groups this node belongs to, separated by commas.";

    void setChangeListener(EditorChangeListener* listener);
    void setProject(const Project* project);
    void setSettings(EditorSettings* settings);
    void setRenameHandler(const RenameHandler& handler);

    void update();

    Node* inspected() const;
    ClassIcons& icons();
    const ClassIcons& icons() const;

    bool showsNoSelection() const;
    bool showsHeader() const;
    bool showsTransform() const;
    bool showsIdentity() const;
    static ct::String tagOf(const Node* node);
    CommitInput* nameInput() const;
    ig::retained::CheckBox* visibleBox() const;
    ig::retained::CheckBox* activeBox() const;
    CommitInput* tagInput() const;
    const ct::String& headerName() const;
    const ct::String& headerClass() const;
    ig::retained::IconId headerIcon() const;
    const ig::retained::Widget* headerWidget() const;

    PropertyInspector& properties();
    const PropertyInspector& properties() const;
    ig::retained::ScrollView* scrollView() const;

    NumberField* field(int row, int axis) const;
    RevertButton* resetButton(int row) const;
    static const char* resetTooltip(int row);

    static float dragSensitivity(int row);

    unsigned int writeCount() const;

private:
    struct Link
    {
        InspectorPanel* owner;
    };

    void nodeRemoved(Node* node) override;
    void selectionChanged() override;

    void show(Node* node);
    void refreshHeader(bool force);
    void refreshTransform(const Node3D* spatial, bool force);
    void fieldChanged(int row);
    void applyRow(int row);
    void resetRow(int row);
    void releaseEditors();
    void refreshIdentity();
    void nameCommitted(const ct::String& text);
    void visibleToggled(bool visible);
    void activeToggled(bool active);
    void tagCommitted(const ct::String& text);
    EditorUndoRedoManager* undoRedo() const;

    SceneTree& mTree;
    EditorSelection& mSelection;
    ClassIcons mIcons;
    ct::Rc<Link> mLink;
    ig::retained::ScrollView* mScroll;
    ig::retained::BoxLayout* mRoot;
    ig::retained::Label* mEmpty;
    InspectorHeader* mHeader;
    ig::retained::BoxLayout* mIdentity;
    CommitInput* mName;
    ig::retained::CheckBox* mVisible;
    ig::retained::CheckBox* mActive;
    CommitInput* mTag;
    RenameHandler mRename;
    ig::retained::BoxLayout* mTransform;
    NumberField* mFields[kRowCount][kAxisCount];
    RevertButton* mResets[kRowCount];
    PropertyInspector mProperties;
    EditorChangeListener* mListener;
    Node* mNode;
    unsigned int mWrites;
    bool mDirty;
    bool mSyncing;
};

} // namespace prisma3d
