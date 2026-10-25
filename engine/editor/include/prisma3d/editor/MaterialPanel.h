#pragma once

#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/MaterialPreview.h"
#include "prisma3d/editor/MaterialProperties.h"
#include "prisma3d/editor/MaterialTextureRow.h"
#include "prisma3d/editor/PropertyRow.h"
#include "prisma3d/render/MaterialLibrary.h"
#include "prisma3d/scene/SceneTreeListener.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/ComboBox.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/ScrollWidgets.hpp>

#include <ct/function.hpp>
#include <ct/ptr.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class EditorDocument;
class EditorViewportPanel;
class MaterialPreviewView;
class MeshInstance3D;
class Node;
class SceneTree;

class MaterialPanel : private SceneTreeListener, private EditorSelectionListener
{
public:
    static constexpr int kOverride = -1;
    static constexpr int kNoSlot = -2;
    static constexpr const char* kFolderName = "Materials";
    static constexpr const char* kDefaultName = "NewMaterial";
    static constexpr const char* kHintTitle = "Select a MeshInstance3D";
    static constexpr const char* kHintDetail = "or open a .mat from Assets.";
    static constexpr const char* kSlotHintTitle = "This mesh has no material to edit";
    static constexpr const char* kSlotHintDetail = "Use New on a slot to create one.";
    static constexpr const char* kReadOnlyTitle = "Built-in material of the model.";
    static constexpr const char* kReadOnlyDetail = "Make Unique copies it into a new .mat.";

    enum class Mode : std::uint8_t
    {
        None,
        Mesh,
        Library
    };

    enum SlotButton : int
    {
        kNew,
        kEdit,
        kClear,
        kSlotButtonCount
    };

    MaterialPanel(RenderingServer& server, SceneTree& tree, EditorSelection& selection,
            EditorDocument& document);
    ~MaterialPanel() override;
    MaterialPanel(const MaterialPanel&) = delete;
    MaterialPanel& operator=(const MaterialPanel&) = delete;

    bool attach(ig::retained::Widget* parent);
    bool attached() const;
    void update();

    using RevealHandler = ct::Function<void()>;
    using CreatedHandler = ct::Function<void(const ct::String&)>;
    using RowProbe = ct::Function<Node*(float, float)>;
    void bindViewport(EditorViewportPanel& viewport);
    void setRowProbe(const RowProbe& probe);
    void setRevealHandler(const RevealHandler& handler);
    void setCreatedHandler(const CreatedHandler& handler);

    Mode mode() const;
    MeshInstance3D* node() const;
    MaterialId edited() const;
    int editedSlot() const;
    bool editable() const;
    bool showsHint() const;

    int slotCount() const;
    int slotSurface(int entry) const;
    ct::String slotTitle(int entry) const;
    ct::String slotName(int entry) const;
    ct::String slotSource(int entry) const;
    MaterialId slotMaterial(int entry) const;
    ig::retained::Button* slotButton(int entry, SlotButton button) const;
    int entryOf(int surface) const;

    ct::String materialsFolder() const;
    ct::String uniqueName(const ct::String& folder, const ct::String& stem) const;
    bool createLibrary(const ct::String& folder, const ct::String& name, ct::String& path,
            ct::String& error);
    void requestNew(const ct::String& folder, int surface);
    bool newMaterial(int surface, const ct::String& name, ct::String& error);
    bool createMaterial(const ct::String& folder, int surface, const ct::String& name,
            ct::String& error);
    bool editSlot(int surface);
    bool clearSlot(int surface);
    bool makeUnique(ct::String& error);
    bool openLibrary(const ct::String& path);
    bool selectLibraryMaterial(int index);
    int dropMaterial(const ct::String& path, Node* node);
    static ct::String stripPart(const ct::String& path);

    bool setField(const char* key, const PropertyValue& value);
    PropertyValue fieldValue(const char* key) const;
    bool fieldShown(const char* key) const;
    bool groupShown(const char* group) const;
    PropertyRow* row(const char* key) const;
    MaterialTextureRow* textureRow(const char* key) const;
    ig::retained::Collapsible* group(const char* name) const;
    const MaterialFeatures& features() const;

    bool dirty() const;
    bool isDirty(MaterialId material) const;
    bool save(ct::String& error);
    bool saveMaterial(MaterialId material, ct::String& error);

    MaterialPreview& preview();
    ig::retained::Widget* previewView() const;
    ig::retained::Button* shapeButton(MaterialPreview::Shape shape) const;
    ig::retained::Button* saveButton() const;
    ig::retained::Button* uniqueButton() const;
    ig::retained::ComboBox* libraryCombo() const;
    ig::retained::Label* nameLabel() const;
    ig::retained::Label* sourceLabel() const;
    ig::retained::ScrollView* scrollView() const;
    const ct::String& libraryPath() const;
    unsigned int writeCount() const;

    void recordAssign(const ct::Vector<MeshInstance3D*>& nodes, int surface, MaterialId material);

private:
    struct Link
    {
        MaterialPanel* owner;
    };

    struct Slot
    {
        int surface = kNoSlot;
        MaterialId material;
        bool nodeLevel = false;
        ct::String title;
        ct::String name;
        ct::String source;
        ig::retained::BoxLayout* box = nullptr;
        ig::retained::Label* titleLabel = nullptr;
        ig::retained::Label* nameLabel = nullptr;
        ig::retained::Label* sourceLabel = nullptr;
        ig::retained::Button* buttons[kSlotButtonCount] = {};
    };

    struct FieldRow
    {
        PropertyRow* row = nullptr;
        MaterialTextureRow* texture = nullptr;
        std::size_t group = 0;
    };

    void nodeRemoved(Node* node) override;
    void selectionChanged() override;

    void buildPreview(ig::retained::BoxLayout* parent);
    void buildGroups();
    void connectRow(std::size_t index);
    void showNode(MeshInstance3D* node);
    void showNothing();
    void rebuildSlots();
    void collectSlots(ct::Vector<Slot>& out) const;
    std::uint64_t slotSignature() const;
    void chooseEdited();
    void bind(MaterialId material, int slot);
    void syncAll(bool force);
    void syncHeader();
    void syncButtons();
    void syncLibraryRow();
    void syncVisibility();
    bool fieldEdited(std::size_t index, const PropertyValue& value);
    void fieldReverted(std::size_t index);
    void pickTexture(std::size_t index);
    void textureChosen(std::size_t index, const ct::String& path);
    void markDirty(MaterialId material);
    void prune();
    static constexpr const char* kAssignAction = "Assign Material";

    bool assign(int surface, MaterialId material);
    bool assignPath(int surface, const ct::String& path, ct::String& error);
    MaterialId slotEffective(int surface, bool& nodeLevel) const;
    void applyShape(MaterialPreview::Shape shape);
    void previewDragEnded(bool accepted);
    void report(const ct::String& message, bool error);
    void reveal();
    ct::String absoluteOf(const ct::String& relative) const;
    ct::String relativeOf(const ct::String& path) const;
    bool ensureFolder(const ct::String& relative) const;
    bool libraryNames(const ct::String& path, ct::Vector<ct::String>& names,
            ct::String& error) const;
    int dropOn(Node* node, const ct::String& path, ct::String& error);
    ct::String firstMaterialOf(const ct::String& library, ct::String& error) const;
    bool visibleInTree() const;

    RenderingServer& mServer;
    SceneTree& mTree;
    EditorSelection& mSelection;
    EditorDocument& mDocument;
    ct::Rc<Link> mLink;
    MaterialPreview mPreview;
    RevealHandler mReveal;
    CreatedHandler mCreated;
    RowProbe mRowProbe;

    ig::retained::ScrollView* mScroll;
    ig::retained::BoxLayout* mRoot;
    ig::retained::BoxLayout* mHint;
    ig::retained::Label* mHintTitle;
    ig::retained::Label* mHintDetail;
    ig::retained::BoxLayout* mTop;
    MaterialPreviewView* mView;
    ig::retained::Label* mName;
    ig::retained::Label* mSource;
    ig::retained::Button* mShapes[2];
    ig::retained::Button* mSave;
    ig::retained::Button* mUnique;
    ig::retained::BoxLayout* mReadOnly;
    ig::retained::BoxLayout* mSlotsBox;
    ig::retained::BoxLayout* mLibraryRow;
    ig::retained::ComboBox* mLibraryCombo;
    ig::retained::BoxLayout* mGroupsBox;
    ct::Vector<ig::retained::Collapsible*> mGroups;
    ct::Vector<FieldRow> mRows;
    ct::Vector<Slot> mSlots;

    Mode mMode;
    MeshInstance3D* mNode;
    MaterialId mEdited;
    int mEditedSlot;
    MaterialFeatures mFeatures;
    MaterialInfo mDefaults;
    ct::Vector<MaterialId> mDirty;
    ct::String mLibraryPath;
    ct::Vector<ct::String> mLibraryNames;
    std::uint64_t mSignature;
    std::uint32_t mShownVersion;
    unsigned int mWrites;
    bool mSlotsDirty;
    bool mSelectionDirty;
    bool mResync;
    bool mSyncing;
};

} // namespace prisma3d
