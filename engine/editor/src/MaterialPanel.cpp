#include "prisma3d/editor/MaterialPanel.h"
#include "prisma3d/editor/FileDialogMemory.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/editor/AssetGrid.h"
#include "prisma3d/editor/AssetTypes.h"
#include "prisma3d/editor/AssetsPanel.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/EditorUndoRedoManager.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/LoadedMaterialLibrary.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include "platform.h"

#include <igui/widgets/DialogWidgets.hpp>
#include <igui/widgets/FileDialog.hpp>
#include <igui/widgets/Theme.hpp>
#include <igui/widgets/WidgetApp.hpp>

#include <string.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const float kSpacing = 6.0f;
const float kSectionSpacing = 4.0f;
const float kSectionPadding = 4.0f;
const float kMinPreview = 96.0f;
const float kMaxPreview = 176.0f;
const char* const kTextureFilter = "*.png;*.jpg;*.jpeg;*.tga;*.bmp;*.dds;*.hdr";
const rt::Color kAccent(110, 170, 230, 255);

void requestLayout()
{
    if (rt::WidgetApp* app = rt::WidgetApp::existing()) app->requestLayout();
}

std::uint64_t mix(std::uint64_t hash, std::uint64_t value)
{
    return (hash ^ value) * 1099511628211ull;
}

bool isAbsolute(const ct::String& path)
{
    if (path.empty()) return false;
    if (path[0] == '/' || path[0] == '\\') return true;
    return path.size() > 1 && path[1] == ':';
}

ct::String joined(const ct::String& folder, const ct::String& name)
{
    if (folder.empty()) return name;
    return folder.ends_with("/") ? folder + name : folder + "/" + name;
}

ct::String safeName(const ct::String& name)
{
    ct::String result;
    for (std::size_t i = 0; i < name.size(); ++i)
    {
        const char c = name[i];
        const bool plain = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                           (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.' || c == ' ';
        result += plain ? c : '_';
    }
    result = result.trimmed();
    while (!result.empty() && result[0] == '.') result = result.substr(1);
    return result.empty() ? ct::String("Material") : result;
}

void relativeTextures(MaterialDef& def, const ct::String& libraryPath)
{
    for (std::size_t slot = 0; slot < kMaterialTextureCount; ++slot)
    {
        if (!def.paths[slot].empty())
            def.paths[slot] = relativeMaterialTexturePath(libraryPath, def.paths[slot]);
        ct::Vector<ct::String>& sequence = def.sequences[slot];
        for (std::size_t frame = 0; frame < sequence.size(); ++frame)
            sequence[frame] = relativeMaterialTexturePath(libraryPath, sequence[frame]);
    }
}

} // namespace

class MaterialPreviewView : public rt::Widget
{
public:
    IG_WIDGET_CLASS(MaterialPreviewView, rt::Widget);

    void setTexture(rt::TextureHandle texture)
    {
        if (mTexture.value == texture.value) return;
        mTexture = texture;
        markDirty();
    }

    rt::TextureHandle texture() const
    {
        return mTexture;
    }

    void setDragPath(const ct::String& path)
    {
        mDragPath = path;
    }

    const ct::String& dragPath() const
    {
        return mDragPath;
    }

    Vec2f sizeHint() const override
    {
        float side = rect_.w < kMinPreview ? kMinPreview : rect_.w;
        if (side > kMaxPreview) side = kMaxPreview;
        return { side, side };
    }

    void paint(rt::PaintContext& ctx) override
    {
        if (!visible_) return;
        const rt::Rect abs = absoluteRect();
        if (ctx.isClipped(abs)) return;
        const rt::Theme& theme = rt::Theme::instance();
        const float side = abs.w < abs.h ? abs.w : abs.h;
        const rt::Rect square{ abs.x + (abs.w - side) * 0.5f, abs.y, side, side };
        ctx.fill.SetColor(theme.inputBg);
        ctx.fill.Rectangle(square.x, square.y, square.w, square.h, true);
        if (mTexture) ctx.drawImage(mTexture, square, rt::Rect{ 0.0f, 1.0f, 1.0f, -1.0f });
        ctx.line.SetColor(theme.borderColor);
        ctx.line.Rectangle(square.x, square.y, square.w, square.h, false);
    }

    void setDragEndHandler(const ct::Function<void(bool)>& handler)
    {
        mDragEnd = handler;
    }

    bool isDragSource() const override
    {
        return !mDragPath.empty();
    }

    void onDragEnd(bool accepted) override
    {
        if (mDragEnd) mDragEnd(accepted);
    }

    rt::DragPayload onDragBegin() override
    {
        rt::DragPayload payload;
        if (mDragPath.empty()) return payload;
        payload.type = AssetGrid::kDragType;
        payload.data = mDragPath;
        return payload;
    }

private:
    rt::TextureHandle mTexture;
    ct::String mDragPath;
    ct::Function<void(bool)> mDragEnd;
};

MaterialPanel::MaterialPanel(RenderingServer& server, SceneTree& tree, EditorSelection& selection,
        EditorDocument& document)
        : mServer(server),
          mTree(tree),
          mSelection(selection),
          mDocument(document),
          mLink(ct::make_rc<Link>(Link{ this })),
          mPreview(server),
          mScroll(nullptr),
          mRoot(nullptr),
          mHint(nullptr),
          mHintTitle(nullptr),
          mHintDetail(nullptr),
          mTop(nullptr),
          mView(nullptr),
          mName(nullptr),
          mSource(nullptr),
          mShapes{},
          mSave(nullptr),
          mUnique(nullptr),
          mReadOnly(nullptr),
          mSlotsBox(nullptr),
          mLibraryRow(nullptr),
          mLibraryCombo(nullptr),
          mGroupsBox(nullptr),
          mMode(Mode::None),
          mNode(nullptr),
          mEditedSlot(kNoSlot),
          mSignature(0),
          mShownVersion(0),
          mWrites(0),
          mSlotsDirty(true),
          mSelectionDirty(true),
          mResync(false),
          mSyncing(false)
{
    mTree.addListener(this);
    mSelection.addListener(this);
    const ct::Rc<Link> link = mLink;
    mDocument.setAssetSaver(
            [link](ct::String& error)
            {
                if (!link->owner) return true;
                return link->owner->save(error);
            });
}

MaterialPanel::~MaterialPanel()
{
    mDocument.setAssetSaver(EditorDocument::AssetSaver());
    mSelection.removeListener(this);
    mTree.removeListener(this);
    mLink->owner = nullptr;
}

void MaterialPanel::bindViewport(EditorViewportPanel& viewport)
{
    const ct::Rc<Link> link = mLink;
    viewport.setMaterialDropHandler(
            [link](const ct::String& path, Node* node)
            {
                if (link->owner) link->owner->dropMaterial(path, node);
            });
}

void MaterialPanel::setRowProbe(const RowProbe& probe)
{
    mRowProbe = probe;
}

void MaterialPanel::previewDragEnded(bool accepted)
{
    if (accepted || !mRowProbe || !mEdited.valid() || !editable()) return;
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return;
    Node* row = mRowProbe(app->mouseX(), app->mouseY());
    const MaterialInfo* info = mServer.materials().get(mEdited);
    if (row && info) dropMaterial(info->path + "::" + info->name, row);
}

void MaterialPanel::setRevealHandler(const RevealHandler& handler)
{
    mReveal = handler;
}

void MaterialPanel::setCreatedHandler(const CreatedHandler& handler)
{
    mCreated = handler;
}

bool MaterialPanel::attach(rt::Widget* parent)
{
    if (mRoot || !parent) return false;

    mScroll = parent->createChild<rt::ScrollView>();
    mRoot = mScroll->setContent<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mRoot->setPadding(8.0f);
    mRoot->setSpacing(kSpacing);

    mHint = mRoot->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mHint->setPadding(0.0f);
    mHint->setSpacing(2.0f);
    mHintTitle = mHint->createChild<rt::Label>(ct::String(kHintTitle));
    mHintDetail = mHint->createChild<rt::Label>(ct::String(kHintDetail));
    mHintDetail->setColor(rt::Theme::instance().textDisabled);

    buildPreview(mRoot);

    mReadOnly = mRoot->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mReadOnly->setPadding(0.0f);
    mReadOnly->setSpacing(2.0f);
    mReadOnly->createChild<rt::Label>(ct::String(kReadOnlyTitle));
    rt::Label* readOnlyDetail = mReadOnly->createChild<rt::Label>(ct::String(kReadOnlyDetail));
    readOnlyDetail->setColor(rt::Theme::instance().textDisabled);
    mReadOnly->setVisible(false);

    mSlotsBox = mRoot->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mSlotsBox->setPadding(0.0f);
    mSlotsBox->setSpacing(kSpacing);
    mSlotsBox->setVisible(false);

    mLibraryRow = mRoot->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    mLibraryRow->setPadding(0.0f);
    mLibraryRow->setSpacing(kSectionSpacing);
    rt::Label* libraryLabel = mLibraryRow->createChild<rt::Label>(ct::String("Material"));
    libraryLabel->setLayoutAlign(rt::Align::Center);
    mLibraryCombo = mLibraryRow->createChild<rt::ComboBox>();
    mLibraryCombo->setStretch(1.0f);
    const ct::Rc<Link> link = mLink;
    mLibraryCombo->selectionChanged.connect(
            [link](int index)
            {
                if (link->owner && !link->owner->mSyncing)
                    link->owner->selectLibraryMaterial(index);
            });
    mLibraryRow->setVisible(false);

    mGroupsBox = mRoot->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mGroupsBox->setPadding(0.0f);
    mGroupsBox->setSpacing(kSectionSpacing);
    mGroupsBox->setVisible(false);
    buildGroups();

    applyShape(MaterialPreview::Shape::Sphere);
    return true;
}

void MaterialPanel::buildPreview(rt::BoxLayout* parent)
{
    const ct::Rc<Link> link = mLink;
    mTop = parent->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mTop->setPadding(0.0f);
    mTop->setSpacing(kSectionSpacing);
    mTop->setVisible(false);

    mView = mTop->createChild<MaterialPreviewView>();
    mView->setDragEndHandler(
            [link](bool accepted)
            {
                if (link->owner) link->owner->previewDragEnded(accepted);
            });

    mName = mTop->createChild<rt::Label>(ct::String());
    mSource = mTop->createChild<rt::Label>(ct::String());
    mSource->setColor(rt::Theme::instance().textDisabled);

    rt::BoxLayout* shapes = mTop->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    shapes->setPadding(0.0f);
    shapes->setSpacing(kSectionSpacing);
    mShapes[0] = shapes->createChild<rt::Button>(ct::String("Sphere"));
    mShapes[0]->setStretch(1.0f);
    mShapes[0]->setIconId(EditorIcons::get("GPUParticlesCollisionSphere3D"));
    mShapes[0]->setTooltip("Preview on a sphere");
    mShapes[0]->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->applyShape(MaterialPreview::Shape::Sphere);
            });
    mShapes[1] = shapes->createChild<rt::Button>(ct::String("Cube"));
    mShapes[1]->setStretch(1.0f);
    mShapes[1]->setIconId(EditorIcons::get("GPUParticlesCollisionBox3D"));
    mShapes[1]->setTooltip("Preview on a cube");
    mShapes[1]->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->applyShape(MaterialPreview::Shape::Cube);
            });

    rt::BoxLayout* actions = mTop->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    actions->setPadding(0.0f);
    actions->setSpacing(kSectionSpacing);
    mSave = actions->createChild<rt::Button>(ct::String("Save"));
    mSave->setStretch(1.0f);
    mSave->setIconId(EditorIcons::get("Save"));
    mSave->setTooltip("Write the changed materials to their .mat files");
    mSave->clicked.connect(
            [link]
            {
                if (!link->owner) return;
                ct::String error;
                if (link->owner->save(error)) link->owner->report("Saved the materials", false);
                else
                    link->owner->report(error, true);
            });
    mUnique = actions->createChild<rt::Button>(ct::String("Make Unique"));
    mUnique->setStretch(1.0f);
    mUnique->setIconId(EditorIcons::get("Duplicate"));
    mUnique->setTooltip("Copy this built-in material into a new .mat file to edit it");
    mUnique->clicked.connect(
            [link]
            {
                if (!link->owner) return;
                ct::String error;
                if (!link->owner->makeUnique(error)) link->owner->report(error, true);
            });
}

void MaterialPanel::buildGroups()
{
    mGroupsBox->createChild<CategoryHeader>(ct::String("Material"),
            EditorIcons::get("StandardMaterial3D"));
    for (std::size_t g = 0; g < MaterialProperties::groupCount(); ++g)
    {
        rt::Collapsible* section = mGroupsBox->createChild<rt::Collapsible>(
                ct::String(MaterialProperties::groupName(g)),
                MaterialProperties::groupOpenByDefault(g));
        section->setContentPadding(kSectionPadding);
        section->setContentSpacing(kSectionSpacing);
        mGroups.push_back(section);
    }

    mRows.resize(MaterialProperties::count());
    for (std::size_t i = 0; i < MaterialProperties::count(); ++i)
    {
        const MaterialField& field = MaterialProperties::field(i);
        FieldRow& entry = mRows[i];
        entry.group = MaterialProperties::groupOf(i);
        rt::Widget* parent = entry.group < mGroups.size()
                                     ? static_cast<rt::Widget*>(mGroups[entry.group])
                                     : static_cast<rt::Widget*>(mGroupsBox);
        if (field.kind == MaterialFieldKind::Texture)
            entry.texture =
                    parent->createChild<MaterialTextureRow>(ct::String(field.label), field.slot);
        else
            entry.row = parent->createChild<PropertyRow>(MaterialProperties::infoOf(field),
                    ct::String(field.label));
        connectRow(i);
    }
}

void MaterialPanel::connectRow(std::size_t index)
{
    const ct::Rc<Link> link = mLink;
    FieldRow& entry = mRows[index];
    if (entry.row)
    {
        entry.row->edited.connect(
                [link, index](const PropertyValue& value)
                {
                    if (link->owner && !link->owner->mSyncing)
                        link->owner->fieldEdited(index, value);
                });
        entry.row->revertRequested.connect(
                [link, index]
                {
                    if (link->owner) link->owner->fieldReverted(index);
                });
        return;
    }
    entry.texture->edited.connect(
            [link, index](const ct::String& path)
            {
                if (link->owner && !link->owner->mSyncing)
                    link->owner->fieldEdited(index, PropertyValue::fromResourcePath(path));
            });
    entry.texture->browseRequested.connect(
            [link, index]
            {
                if (link->owner) link->owner->pickTexture(index);
            });
    entry.texture->revertRequested.connect(
            [link, index]
            {
                if (link->owner) link->owner->fieldReverted(index);
            });
}

bool MaterialPanel::attached() const
{
    return mRoot != nullptr;
}

void MaterialPanel::nodeRemoved(Node* node)
{
    if (node != mNode) return;
    mNode = nullptr;
    mSelectionDirty = true;
    mSlotsDirty = true;
}

void MaterialPanel::selectionChanged()
{
    mSelectionDirty = true;
}

bool MaterialPanel::visibleInTree() const
{
    for (const rt::Widget* widget = mScroll; widget; widget = widget->parent())
        if (!widget->isVisible()) return false;
    return mScroll != nullptr;
}

void MaterialPanel::update()
{
    if (!mRoot) return;
    prune();

    if (mSelectionDirty)
    {
        mSelectionDirty = false;
        const ct::Vector<Node*>& selected = mSelection.selectedNodes();
        MeshInstance3D* mesh = selected.empty() ? nullptr : selected[0]->castTo<MeshInstance3D>();
        if (mesh && (mesh != mNode || mMode != Mode::Mesh)) showNode(mesh);
        else if (!mesh && mMode == Mode::Mesh)
            showNothing();
    }

    MaterialStore& store = mServer.materials();
    if (mEdited.valid() && !store.valid(mEdited))
    {
        bind(MaterialId(), kNoSlot);
        mSlotsDirty = true;
    }

    if (mMode == Mode::Mesh && mNode && slotSignature() != mSignature) mSlotsDirty = true;
    if (mSlotsDirty)
    {
        rebuildSlots();
        bool present = false;
        for (std::size_t i = 0; i < mSlots.size(); ++i)
            present = present || (mSlots[i].material == mEdited && mEdited.valid());
        if (mMode == Mode::Mesh && !present) chooseEdited();
        syncButtons();
    }

    const MaterialInfo* info = store.get(mEdited);
    if (info && (info->version != mShownVersion || mResync))
    {
        bool active = false;
        for (std::size_t i = 0; i < mRows.size(); ++i)
            active = active || (mRows[i].row && mRows[i].row->isActive());
        if (!active)
        {
            mResync = false;
            syncAll(false);
        }
    }

    mPreview.setVisible(visibleInTree());
    mPreview.update();
    const TextureId texture = mPreview.texture();
    if (texture.valid()) mView->setTexture(WidgetHost::textureHandle(texture));
}

void MaterialPanel::showNode(MeshInstance3D* node)
{
    mMode = Mode::Mesh;
    mNode = node;
    mLibraryPath = ct::String();
    mLibraryNames.clear();
    bind(MaterialId(), kNoSlot);
    rebuildSlots();
    chooseEdited();
    syncLibraryRow();
    syncButtons();
    requestLayout();
}

void MaterialPanel::showNothing()
{
    mMode = Mode::None;
    mNode = nullptr;
    bind(MaterialId(), kNoSlot);
    rebuildSlots();
    syncLibraryRow();
    syncButtons();
    requestLayout();
}

MaterialId MaterialPanel::slotEffective(int surface, bool& nodeLevel) const
{
    nodeLevel = false;
    if (!mNode) return MaterialId();
    MaterialStore& store = mServer.materials();
    if (surface == kOverride)
    {
        const MaterialId id = mNode->getMaterialOverride();
        nodeLevel = store.valid(id);
        return nodeLevel ? id : MaterialId();
    }
    const MaterialId own = mNode->getSurfaceMaterial(static_cast<std::uint32_t>(surface));
    if (store.valid(own))
    {
        nodeLevel = true;
        return own;
    }
    const MeshInfo* mesh = mServer.meshes().get(mNode->getMesh());
    if (mesh && static_cast<std::size_t>(surface) < mesh->surfaces.size())
    {
        const MaterialId builtIn = mesh->surfaces[static_cast<std::size_t>(surface)].material;
        if (store.valid(builtIn)) return builtIn;
    }
    return MaterialId();
}

void MaterialPanel::collectSlots(ct::Vector<Slot>& out) const
{
    out.clear();
    if (mMode != Mode::Mesh || !mNode) return;
    const MeshInfo* mesh = mServer.meshes().get(mNode->getMesh());
    const int surfaces = mesh ? static_cast<int>(mesh->surfaces.size()) : 0;
    for (int i = 0; i <= surfaces; ++i)
    {
        Slot slot;
        slot.surface = i == surfaces ? kOverride : i;
        slot.material = slotEffective(slot.surface, slot.nodeLevel);
        if (slot.surface == kOverride) slot.title = "Override";
        else
        {
            slot.title = "Surface ";
            slot.title.append_number(slot.surface);
        }
        const MaterialInfo* info = mServer.materials().get(slot.material);
        slot.name =
                info && !info->name.empty() ? info->name : ct::String(info ? "Unnamed" : "None");
        slot.source = MaterialProperties::sourceOf(info);
        out.push_back(slot);
    }
}

std::uint64_t MaterialPanel::slotSignature() const
{
    std::uint64_t hash = 1469598103934665603ull;
    if (mMode != Mode::Mesh || !mNode) return hash;
    hash = mix(hash, mNode->getMesh().bits());
    const MeshInfo* mesh = mServer.meshes().get(mNode->getMesh());
    const int surfaces = mesh ? static_cast<int>(mesh->surfaces.size()) : 0;
    for (int i = 0; i <= surfaces; ++i)
    {
        const int surface = i == surfaces ? kOverride : i;
        bool nodeLevel = false;
        const MaterialId material = slotEffective(surface, nodeLevel);
        hash = mix(hash, static_cast<std::uint64_t>(surface + 2));
        hash = mix(hash, material.bits());
        hash = mix(hash, nodeLevel ? 1u : 0u);
    }
    return hash;
}

void MaterialPanel::rebuildSlots()
{
    mSlotsDirty = false;
    const ct::Vector<rt::Widget*> children = mSlotsBox->children();
    for (std::size_t i = 0; i < children.size(); ++i) mSlotsBox->removeChild(children[i]);
    mSlots.clear();
    mSignature = slotSignature();
    if (mMode != Mode::Mesh || !mNode)
    {
        mSlotsBox->setVisible(false);
        requestLayout();
        return;
    }
    collectSlots(mSlots);
    mSlotsBox->setVisible(true);
    mSlotsBox->createChild<CategoryHeader>(ct::String("Material Slots"),
            EditorIcons::get("MeshInstance3D"));

    const ct::Rc<Link> link = mLink;
    const rt::Theme& theme = rt::Theme::instance();
    for (std::size_t i = 0; i < mSlots.size(); ++i)
    {
        Slot& slot = mSlots[i];
        const int surface = slot.surface;
        slot.box = mSlotsBox->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
        slot.box->setPadding(0.0f);
        slot.box->setSpacing(2.0f);

        slot.titleLabel = slot.box->createChild<rt::Label>(slot.title + ": " + slot.name);
        slot.sourceLabel = slot.box->createChild<rt::Label>(slot.source);
        slot.sourceLabel->setColor(theme.textDisabled);
        slot.sourceLabel->setTooltip(slot.source);

        rt::BoxLayout* actions = slot.box->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
        actions->setPadding(0.0f);
        actions->setSpacing(kSectionSpacing);
        static const char* const labels[kSlotButtonCount] = { "New", "Edit", "Clear" };
        static const char* const slotIcons[kSlotButtonCount] = { "Add", "Edit", "Close" };
        static const char* const tips[kSlotButtonCount] = { "Create a new material and use it here",
            "Edit the material of this slot", "Remove the material assigned to this slot" };
        for (int b = 0; b < kSlotButtonCount; ++b)
        {
            rt::Button* button = actions->createChild<rt::Button>(ct::String(labels[b]));
            button->setTooltip(tips[b]);
            button->setIconId(EditorIcons::get(slotIcons[b]));
            button->setStretch(1.0f);
            button->clicked.connect(
                    [link, surface, b]
                    {
                        if (!link->owner) return;
                        if (b == kNew)
                            link->owner->requestNew(link->owner->materialsFolder(), surface);
                        else if (b == kEdit)
                            link->owner->editSlot(surface);
                        else
                            link->owner->clearSlot(surface);
                    });
            slot.buttons[b] = button;
        }
    }
    requestLayout();
}

void MaterialPanel::chooseEdited()
{
    for (std::size_t i = mSlots.size(); i > 0; --i)
    {
        const Slot& slot = mSlots[i - 1];
        if (!slot.material.valid()) continue;
        bind(slot.material, slot.surface);
        return;
    }
    bind(MaterialId(), kNoSlot);
}

void MaterialPanel::bind(MaterialId material, int slot)
{
    MaterialStore& store = mServer.materials();
    const MaterialInfo* info = store.get(material);
    mEdited = info ? material : MaterialId();
    mEditedSlot = info ? slot : kNoSlot;
    mFeatures = info ? MaterialFeatures::of(*info) : MaterialFeatures();
    mPreview.setMaterial(mEdited);
    mShownVersion = 0;
    syncAll(true);
    syncHeader();
    syncButtons();
}

void MaterialPanel::syncAll(bool force)
{
    const MaterialInfo* info = mServer.materials().get(mEdited);
    const bool has = info != nullptr;
    mTop->setVisible(has);
    mGroupsBox->setVisible(has);
    mReadOnly->setVisible(has && !MaterialProperties::editable(*info));
    mHint->setVisible(!has);
    if (!has)
    {
        mHintTitle->setText(ct::String(mMode == Mode::Mesh ? kSlotHintTitle : kHintTitle));
        mHintDetail->setText(ct::String(mMode == Mode::Mesh ? kSlotHintDetail : kHintDetail));
        requestLayout();
        return;
    }
    mGroupsBox->setEnabled(MaterialProperties::editable(*info));

    const MaterialInfo copy = *info;
    mSyncing = true;
    for (std::size_t i = 0; i < mRows.size(); ++i)
    {
        const MaterialField& field = MaterialProperties::field(i);
        FieldRow& entry = mRows[i];
        const PropertyValue value = MaterialProperties::read(copy, mFeatures, field);
        const PropertyValue fallback =
                MaterialProperties::read(mDefaults, MaterialFeatures(), field);
        if (entry.row)
        {
            if (!force && entry.row->isActive())
            {
                mResync = true;
                continue;
            }
            entry.row->setValue(value);
            entry.row->setRevertible(value != fallback);
        }
        else
        {
            const TextureId texture = copy.textures[static_cast<std::size_t>(field.slot)];
            entry.texture->setPath(value.asString(), texture);
            entry.texture->setRevertible(value != fallback);
        }
    }
    mSyncing = false;
    mShownVersion = copy.version;
    syncVisibility();
    requestLayout();
}

void MaterialPanel::syncVisibility()
{
    const MaterialInfo* info = mServer.materials().get(mEdited);
    if (!info) return;
    ct::Vector<bool> shown;
    shown.resize(mGroups.size(), false);
    for (std::size_t i = 0; i < mRows.size(); ++i)
    {
        const bool visible =
                MaterialProperties::visible(*info, mFeatures, MaterialProperties::field(i));
        rt::Widget* widget = mRows[i].row ? static_cast<rt::Widget*>(mRows[i].row)
                                          : static_cast<rt::Widget*>(mRows[i].texture);
        widget->setVisible(visible);
        if (visible && mRows[i].group < shown.size()) shown[mRows[i].group] = true;
    }
    for (std::size_t g = 0; g < mGroups.size(); ++g) mGroups[g]->setVisible(shown[g]);
}

void MaterialPanel::syncHeader()
{
    const MaterialInfo* info = mServer.materials().get(mEdited);
    if (!info)
    {
        mName->setText(ct::String());
        mSource->setText(ct::String());
        mView->setDragPath(ct::String());
        return;
    }
    ct::String name = info->name.empty() ? ct::String("Unnamed") : info->name;
    if (isDirty(mEdited)) name += " *";
    mName->setText(name);
    const ct::String source = MaterialProperties::sourceOf(info);
    mSource->setText(source);
    mSource->setTooltip(source);
    mView->setDragPath(
            MaterialProperties::editable(*info) ? info->path + "::" + info->name : ct::String());
}

void MaterialPanel::syncButtons()
{
    const MaterialInfo* info = mServer.materials().get(mEdited);
    mSave->setEnabled(dirty());
    mUnique->setVisible(info && !MaterialProperties::editable(*info) && mEditedSlot != kNoSlot);
    for (std::size_t i = 0; i < mSlots.size(); ++i)
    {
        Slot& slot = mSlots[i];
        if (!slot.buttons[kEdit]) continue;
        slot.buttons[kEdit]->setEnabled(slot.material.valid());
        slot.buttons[kClear]->setEnabled(slot.nodeLevel);
        const bool current = slot.surface == mEditedSlot && slot.material == mEdited;
        slot.titleLabel->setColor(current ? kAccent : rt::Theme::instance().textColor);
    }
}

void MaterialPanel::syncLibraryRow()
{
    const bool shown = mMode == Mode::Library && mLibraryNames.size() > 1;
    mLibraryRow->setVisible(shown);
    mSyncing = true;
    mLibraryCombo->clear();
    for (std::size_t i = 0; i < mLibraryNames.size(); ++i) mLibraryCombo->addItem(mLibraryNames[i]);
    mSyncing = false;
}

void MaterialPanel::applyShape(MaterialPreview::Shape shape)
{
    mPreview.setShape(shape);
    const rt::Theme& theme = rt::Theme::instance();
    for (int i = 0; i < 2; ++i)
        mShapes[i]->setBgColor(
                i == static_cast<int>(shape) ? theme.dialogBtnPrimary : rt::Color(0, 0, 0, 0));
}

void MaterialPanel::report(const ct::String& message, bool error)
{
    if (error) log_error("editor: %s", message.c_str());
    rt::Toast::show(message, error ? rt::Toast::Type::Error : rt::Toast::Type::Success,
            error ? 6.0f : rt::Toast::kDefaultDuration);
}

void MaterialPanel::reveal()
{
    if (mReveal) mReveal();
}

MaterialPanel::Mode MaterialPanel::mode() const
{
    return mMode;
}

MeshInstance3D* MaterialPanel::node() const
{
    return mNode;
}

MaterialId MaterialPanel::edited() const
{
    return mEdited;
}

int MaterialPanel::editedSlot() const
{
    return mEditedSlot;
}

bool MaterialPanel::editable() const
{
    const MaterialInfo* info = mServer.materials().get(mEdited);
    return info && MaterialProperties::editable(*info);
}

bool MaterialPanel::showsHint() const
{
    return mHint && mHint->isVisible();
}

int MaterialPanel::slotCount() const
{
    return static_cast<int>(mSlots.size());
}

int MaterialPanel::slotSurface(int entry) const
{
    return entry >= 0 && entry < slotCount() ? mSlots[static_cast<std::size_t>(entry)].surface
                                             : kNoSlot;
}

ct::String MaterialPanel::slotTitle(int entry) const
{
    return entry >= 0 && entry < slotCount() ? mSlots[static_cast<std::size_t>(entry)].title
                                             : ct::String();
}

ct::String MaterialPanel::slotName(int entry) const
{
    return entry >= 0 && entry < slotCount() ? mSlots[static_cast<std::size_t>(entry)].name
                                             : ct::String();
}

ct::String MaterialPanel::slotSource(int entry) const
{
    return entry >= 0 && entry < slotCount() ? mSlots[static_cast<std::size_t>(entry)].source
                                             : ct::String();
}

MaterialId MaterialPanel::slotMaterial(int entry) const
{
    return entry >= 0 && entry < slotCount() ? mSlots[static_cast<std::size_t>(entry)].material
                                             : MaterialId();
}

rt::Button* MaterialPanel::slotButton(int entry, SlotButton button) const
{
    if (entry < 0 || entry >= slotCount() || button < 0 || button >= kSlotButtonCount)
        return nullptr;
    return mSlots[static_cast<std::size_t>(entry)].buttons[button];
}

int MaterialPanel::entryOf(int surface) const
{
    for (std::size_t i = 0; i < mSlots.size(); ++i)
        if (mSlots[i].surface == surface) return static_cast<int>(i);
    return -1;
}

ct::String MaterialPanel::absoluteOf(const ct::String& relative) const
{
    if (isAbsolute(relative)) return relative;
    const Project& project = mDocument.project();
    return project.isOpen() ? project.absolutePath(relative) : relative;
}

ct::String MaterialPanel::relativeOf(const ct::String& path) const
{
    ct::String relative;
    const Project& project = mDocument.project();
    if (project.isOpen() && project.relativePath(path, relative)) return relative;
    return path;
}

ct::String MaterialPanel::materialsFolder() const
{
    const Project& project = mDocument.project();
    const ct::String root = project.isOpen() ? project.assetRoot() : ct::String();
    return joined(root, kFolderName);
}

ct::String MaterialPanel::uniqueName(const ct::String& folder, const ct::String& stem) const
{
    ct::String name = stem;
    for (int index = 2; index < 10000; ++index)
    {
        const ct::String file =
                absoluteOf(joined(folder, name + MaterialProperties::kLibraryExtension));
        if (!file_exists(file.c_str())) break;
        name = stem;
        name.append_number(index);
    }
    return name;
}

ct::String MaterialPanel::stripPart(const ct::String& path)
{
    return AssetTypes::filePart(path);
}

bool MaterialPanel::libraryNames(const ct::String& path, ct::Vector<ct::String>& names,
        ct::String& error) const
{
    names.clear();
    MaterialLibraryData data;
    if (!MaterialLibraryFormat::load(absoluteOf(path), data, error)) return false;
    for (std::size_t i = 0; i < data.materials.size(); ++i)
        names.push_back(data.materials[i].info.name);
    if (names.empty())
    {
        error = "the library has no materials";
        return false;
    }
    return true;
}

ct::String MaterialPanel::firstMaterialOf(const ct::String& library, ct::String& error) const
{
    ct::Vector<ct::String> names;
    if (!libraryNames(library, names, error)) return ct::String();
    return library + "::" + names[0];
}

bool MaterialPanel::createLibrary(const ct::String& folder, const ct::String& name,
        ct::String& path, ct::String& error)
{
    const Project& project = mDocument.project();
    if (!project.isOpen())
    {
        error = "Open or create a project first.";
        return false;
    }
    ct::String reason;
    if (!AssetsPanel::validEntryName(name, reason))
    {
        error = reason;
        return false;
    }
    ct::String base;
    if (!folder.empty() && !Project::normalizeRelativePath(folder, base, reason))
    {
        error = reason;
        return false;
    }
    const ct::String relative = joined(base, name + MaterialProperties::kLibraryExtension);
    const ct::String absolute = absoluteOf(relative);
    if (file_exists(absolute.c_str()) || dir_exists(absolute.c_str()))
    {
        error = relative + " already exists.";
        return false;
    }
    if (!base.empty() && !fs_create_directory(absoluteOf(base).c_str()))
    {
        error = ct::String("Could not create the folder ") + base;
        return false;
    }
    MaterialLibraryData data;
    MaterialDef def;
    def.info.name = name;
    data.materials.push_back(def);
    if (!MaterialLibraryFormat::save(absolute, data, error)) return false;
    path = relative + "::" + name;
    return true;
}

void MaterialPanel::requestNew(const ct::String& folder, int surface)
{
    if (!mDocument.project().isOpen())
    {
        report("Open or create a project to create materials.", true);
        return;
    }
    rt::WidgetApp& app = rt::WidgetApp::instance();
    rt::InputBox* box = app.addFloat<rt::InputBox>("New Material",
            ct::String("Material name in ") +
                    (folder.empty() ? ct::String("the project") : folder) + ":");
    box->setText(uniqueName(folder, kDefaultName));
    box->setAcceptLabel("Create");
    const ct::Rc<Link> link = mLink;
    box->setValidator(
            [link, folder](const ct::String& text)
            {
                if (!link->owner) return ct::String();
                const ct::String name = text.trimmed();
                ct::String reason;
                if (!AssetsPanel::validEntryName(name, reason)) return reason;
                const ct::String file = link->owner->absoluteOf(
                        joined(folder, name + MaterialProperties::kLibraryExtension));
                if (file_exists(file.c_str()))
                    return name + MaterialProperties::kLibraryExtension + " already exists.";
                return ct::String();
            });
    box->accepted.connect(
            [link, folder, surface](const ct::String& text)
            {
                if (!link->owner) return;
                ct::String error;
                if (!link->owner->createMaterial(folder, surface, text.trimmed(), error))
                    link->owner->report(error, true);
            });
}

bool MaterialPanel::newMaterial(int surface, const ct::String& name, ct::String& error)
{
    return createMaterial(materialsFolder(), surface, name, error);
}

bool MaterialPanel::createMaterial(const ct::String& folder, int surface, const ct::String& name,
        ct::String& error)
{
    ct::String path;
    if (!createLibrary(folder, name, path, error)) return false;
    sceneres::clearError();
    const MaterialId material = sceneres::loadMaterial(path);
    if (!material.valid())
    {
        error = sceneres::takeError();
        return false;
    }
    if (surface != kNoSlot && mMode == Mode::Mesh && mNode)
    {
        assign(surface, material);
        mSlotsDirty = true;
        bind(material, surface);
        rebuildSlots();
        syncButtons();
    }
    else if (!openLibrary(path))
    {
        error = ct::String("Could not open ") + path;
        return false;
    }
    if (mCreated) mCreated(stripPart(path));
    reveal();
    report(ct::String("Created ") + stripPart(path), false);
    return true;
}

namespace
{

struct SlotChange
{
    MeshInstance3D* node;
    int surface;
    MaterialId before;
    MaterialId after;
};

void applySlots(const ct::Vector<SlotChange>& changes, bool after)
{
    for (std::size_t i = 0; i < changes.size(); ++i)
    {
        const SlotChange& change = changes[i];
        const MaterialId material = after ? change.after : change.before;
        if (change.surface == MaterialPanel::kOverride) change.node->setMaterialOverride(material);
        else
            change.node->setSurfaceMaterial(static_cast<std::uint32_t>(change.surface), material);
    }
}

MaterialId nodeSlotMaterial(const MeshInstance3D* node, int surface)
{
    if (surface == MaterialPanel::kOverride) return node->getMaterialOverride();
    return node->getSurfaceMaterial(static_cast<std::uint32_t>(surface));
}

} // namespace

void MaterialPanel::recordAssign(const ct::Vector<MeshInstance3D*>& nodes, int surface,
        MaterialId material)
{
    ct::Vector<SlotChange> changes;
    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        SlotChange change;
        change.node = nodes[i];
        change.surface = surface;
        change.before = nodeSlotMaterial(nodes[i], surface);
        change.after = material;
        changes.push_back(change);
    }
    EditorUndoRedoManager& undo = mDocument.undoRedo();
    undo.createAction(kAssignAction);
    undo.addDoMethod([changes] { applySlots(changes, true); });
    undo.addUndoMethod([changes] { applySlots(changes, false); });
    undo.commitAction();
}

bool MaterialPanel::assign(int surface, MaterialId material)
{
    if (!mNode) return false;
    if (surface != kOverride && surface < 0) return false;
    ct::Vector<MeshInstance3D*> nodes;
    nodes.push_back(mNode);
    recordAssign(nodes, surface, material);
    return true;
}

bool MaterialPanel::assignPath(int surface, const ct::String& path, ct::String& error)
{
    sceneres::clearError();
    const MaterialId material = sceneres::loadMaterial(path);
    if (!material.valid())
    {
        error = sceneres::takeError();
        return false;
    }
    return assign(surface, material);
}

bool MaterialPanel::editSlot(int surface)
{
    const int entry = entryOf(surface);
    if (entry < 0 || !mSlots[static_cast<std::size_t>(entry)].material.valid()) return false;
    bind(mSlots[static_cast<std::size_t>(entry)].material, surface);
    return true;
}

bool MaterialPanel::clearSlot(int surface)
{
    const int entry = entryOf(surface);
    if (entry < 0 || !mSlots[static_cast<std::size_t>(entry)].nodeLevel) return false;
    assign(surface, MaterialId());
    mSlotsDirty = true;
    if (mEditedSlot == surface) bind(MaterialId(), kNoSlot);
    return true;
}

bool MaterialPanel::makeUnique(ct::String& error)
{
    MaterialStore& store = mServer.materials();
    const MaterialInfo* info = store.get(mEdited);
    if (!info)
    {
        error = "There is no material to copy.";
        return false;
    }
    if (MaterialProperties::editable(*info))
    {
        error = "The material already lives in a .mat file.";
        return false;
    }
    if (!mDocument.project().isOpen())
    {
        error = "Open or create a project first.";
        return false;
    }
    const ct::String folder = materialsFolder();
    const ct::String name = uniqueName(folder, safeName(info->name));
    const ct::String relative = joined(folder, name + MaterialProperties::kLibraryExtension);

    MaterialDef def;
    captureMaterial(store, mEdited, def);
    def.info.name = name;
    relativeTextures(def, relative);
    MaterialLibraryData data;
    data.materials.push_back(def);
    if (!fs_create_directory(absoluteOf(folder).c_str()))
    {
        error = ct::String("Could not create the folder ") + folder;
        return false;
    }
    if (!MaterialLibraryFormat::save(absoluteOf(relative), data, error)) return false;

    const ct::String path = relative + "::" + name;
    sceneres::clearError();
    const MaterialId material = sceneres::loadMaterial(path);
    if (!material.valid())
    {
        error = sceneres::takeError();
        return false;
    }
    const int slot = mEditedSlot;
    if (mNode && slot != kNoSlot) assign(slot, material);
    mSlotsDirty = true;
    bind(material, slot);
    if (mCreated) mCreated(relative);
    report(ct::String("Created ") + relative, false);
    return true;
}

bool MaterialPanel::openLibrary(const ct::String& path)
{
    const ct::String file = relativeOf(stripPart(path));
    const std::size_t at = path.find("::");
    const ct::String wanted = at == ct::String::npos ? ct::String() : path.substr(at + 2);

    ct::Vector<ct::String> names;
    ct::String error;
    if (!libraryNames(file, names, error))
    {
        report(file + ": " + error, true);
        return false;
    }
    mMode = Mode::Library;
    mNode = nullptr;
    mLibraryPath = file;
    mLibraryNames = names;
    int index = 0;
    for (std::size_t i = 0; i < names.size(); ++i)
        if (!wanted.empty() && names[i] == wanted) index = static_cast<int>(i);
    syncLibraryRow();
    mSyncing = true;
    mLibraryCombo->setSelectedIndex(index);
    mSyncing = false;
    mSlotsDirty = true;
    rebuildSlots();
    const bool opened = selectLibraryMaterial(index);
    reveal();
    return opened;
}

bool MaterialPanel::selectLibraryMaterial(int index)
{
    if (mMode != Mode::Library || index < 0 || index >= static_cast<int>(mLibraryNames.size()))
        return false;
    sceneres::clearError();
    const MaterialId material = sceneres::loadMaterial(
            mLibraryPath + "::" + mLibraryNames[static_cast<std::size_t>(index)]);
    if (!material.valid())
    {
        report(sceneres::takeError(), true);
        return false;
    }
    bind(material, kNoSlot);
    return true;
}

namespace
{

void collectMeshes(Node* node, ct::Vector<MeshInstance3D*>& meshes)
{
    if (MeshInstance3D* mesh = node->castTo<MeshInstance3D>())
    {
        meshes.push_back(mesh);
        return;
    }
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        collectMeshes(node->getChild(i), meshes);
}

} // namespace

int MaterialPanel::dropOn(Node* node, const ct::String& path, ct::String& error)
{
    if (!node) return 0;
    ct::Vector<MeshInstance3D*> meshes;
    collectMeshes(node, meshes);
    if (meshes.empty()) return 0;
    sceneres::clearError();
    const MaterialId material = sceneres::loadMaterial(path);
    if (!material.valid())
    {
        error = sceneres::takeError();
        return 0;
    }
    recordAssign(meshes, kOverride, material);
    return static_cast<int>(meshes.size());
}

int MaterialPanel::dropMaterial(const ct::String& path, Node* node)
{
    ct::String error;
    ct::String material = path;
    if (path.find("::") == ct::String::npos)
    {
        material = firstMaterialOf(relativeOf(path), error);
        if (material.empty())
        {
            report(path + ": " + error, true);
            return 0;
        }
    }
    const int count = dropOn(node, material, error);
    if (count == 0)
    {
        report(error.empty() ? ct::String("Drop the material on a mesh.") : error, true);
        return 0;
    }
    mSlotsDirty = true;
    report(ct::String("Assigned ") + material, false);
    return count;
}

void MaterialPanel::markDirty(MaterialId material)
{
    if (!isDirty(material)) mDirty.push_back(material);
    mDocument.sceneEdited();
}

bool MaterialPanel::dirty() const
{
    for (std::size_t i = 0; i < mDirty.size(); ++i)
        if (mServer.materials().valid(mDirty[i])) return true;
    return false;
}

bool MaterialPanel::isDirty(MaterialId material) const
{
    for (std::size_t i = 0; i < mDirty.size(); ++i)
        if (mDirty[i] == material) return true;
    return false;
}

void MaterialPanel::prune()
{
    for (std::size_t i = mDirty.size(); i > 0; --i)
        if (!mServer.materials().valid(mDirty[i - 1])) mDirty.erase(mDirty.begin() + (i - 1));
}

bool MaterialPanel::save(ct::String& error)
{
    prune();
    ct::Vector<MaterialId> pending = mDirty;
    for (std::size_t i = 0; i < pending.size(); ++i)
    {
        if (!saveMaterial(pending[i], error)) return false;
        for (std::size_t d = 0; d < mDirty.size(); ++d)
            if (mDirty[d] == pending[i])
            {
                mDirty.erase(mDirty.begin() + d);
                break;
            }
    }
    syncHeader();
    syncButtons();
    return true;
}

bool MaterialPanel::saveMaterial(MaterialId material, ct::String& error)
{
    MaterialStore& store = mServer.materials();
    const MaterialInfo* info = store.get(material);
    if (!info || !MaterialProperties::editable(*info))
    {
        error = "The material does not belong to a .mat file.";
        return false;
    }
    const ct::String logical = info->path;
    const ct::String absolute = absoluteOf(logical);

    MaterialLibraryData data;
    if (file_exists(absolute.c_str()) && !MaterialLibraryFormat::load(absolute, data, error))
        return false;
    MaterialDef def;
    captureMaterial(store, material, def);
    relativeTextures(def, logical);
    bool replaced = false;
    for (std::size_t i = 0; i < data.materials.size() && !replaced; ++i)
    {
        if (!(data.materials[i].info.name == def.info.name)) continue;
        data.materials[i] = def;
        replaced = true;
    }
    if (!replaced) data.materials.push_back(def);
    return MaterialLibraryFormat::save(absolute, data, error);
}

bool MaterialPanel::fieldEdited(std::size_t index, const PropertyValue& value)
{
    if (index >= mRows.size()) return false;
    MaterialStore& store = mServer.materials();
    if (!editable())
    {
        report("Make the material unique to edit it.", true);
        syncAll(true);
        return false;
    }
    sceneres::clearError();
    const MaterialField& field = MaterialProperties::field(index);
    if (!MaterialProperties::write(store, mEdited, mFeatures, field, value))
    {
        const ct::String error = sceneres::takeError();
        report(error.empty() ? ct::String("The value was refused.") : error, true);
        syncAll(true);
        return false;
    }
    ++mWrites;
    markDirty(mEdited);
    syncAll(false);
    syncHeader();
    syncButtons();
    return true;
}

void MaterialPanel::fieldReverted(std::size_t index)
{
    if (index >= mRows.size()) return;
    const MaterialField& field = MaterialProperties::field(index);
    fieldEdited(index, MaterialProperties::read(mDefaults, MaterialFeatures(), field));
}

void MaterialPanel::pickTexture(std::size_t index)
{
    if (!editable())
    {
        report("Make the material unique to edit it.", true);
        return;
    }
    rt::WidgetApp& app = rt::WidgetApp::instance();
    rt::FileDialog* dialog = app.addFloat<rt::FileDialog>("Choose a texture");
    dialog->setMode(rt::FileDialog::Mode::Open);
    dialog->setFilter(kTextureFilter);
    EditorSettings* settings = &mDocument.settings();
    FileDialogMemory::prepare(*dialog, settings, EditorSettings::kFolderTexture,
            &mDocument.project());
    const ct::Rc<Link> link = mLink;
    dialog->accepted.connect(
            [link, index, settings](const ct::String& path)
            {
                FileDialogMemory::remember(settings, EditorSettings::kFolderTexture, path);
                if (link->owner) link->owner->textureChosen(index, path);
            });
}

void MaterialPanel::textureChosen(std::size_t index, const ct::String& path)
{
    const Project& project = mDocument.project();
    ct::String stored = path;
    if (project.isOpen() && !project.relativePath(path, stored))
    {
        report("Choose an image inside the project folder.", true);
        return;
    }
    if (!MaterialTextureRow::acceptsPath(stored))
    {
        report("Choose an image file.", true);
        return;
    }
    fieldEdited(index, PropertyValue::fromResourcePath(stored));
}

bool MaterialPanel::setField(const char* key, const PropertyValue& value)
{
    const std::size_t index = MaterialProperties::indexOf(key);
    if (index >= mRows.size()) return false;
    return fieldEdited(index, value);
}

PropertyValue MaterialPanel::fieldValue(const char* key) const
{
    const std::size_t index = MaterialProperties::indexOf(key);
    const MaterialInfo* info = mServer.materials().get(mEdited);
    if (index >= mRows.size() || !info) return PropertyValue();
    return MaterialProperties::read(*info, mFeatures, MaterialProperties::field(index));
}

bool MaterialPanel::fieldShown(const char* key) const
{
    const std::size_t index = MaterialProperties::indexOf(key);
    if (index >= mRows.size()) return false;
    const rt::Widget* widget = mRows[index].row
                                       ? static_cast<const rt::Widget*>(mRows[index].row)
                                       : static_cast<const rt::Widget*>(mRows[index].texture);
    return widget->isVisible() && mGroupsBox->isVisible();
}

bool MaterialPanel::groupShown(const char* name) const
{
    const rt::Collapsible* section = group(name);
    return section && section->isVisible() && mGroupsBox->isVisible();
}

PropertyRow* MaterialPanel::row(const char* key) const
{
    const std::size_t index = MaterialProperties::indexOf(key);
    return index < mRows.size() ? mRows[index].row : nullptr;
}

MaterialTextureRow* MaterialPanel::textureRow(const char* key) const
{
    const std::size_t index = MaterialProperties::indexOf(key);
    return index < mRows.size() ? mRows[index].texture : nullptr;
}

rt::Collapsible* MaterialPanel::group(const char* name) const
{
    for (std::size_t i = 0; i < mGroups.size(); ++i)
        if (mGroups[i]->title() == name) return mGroups[i];
    return nullptr;
}

const MaterialFeatures& MaterialPanel::features() const
{
    return mFeatures;
}

MaterialPreview& MaterialPanel::preview()
{
    return mPreview;
}

rt::Widget* MaterialPanel::previewView() const
{
    return mView;
}

rt::Button* MaterialPanel::shapeButton(MaterialPreview::Shape shape) const
{
    return mShapes[static_cast<int>(shape)];
}

rt::Button* MaterialPanel::saveButton() const
{
    return mSave;
}

rt::Button* MaterialPanel::uniqueButton() const
{
    return mUnique;
}

rt::ComboBox* MaterialPanel::libraryCombo() const
{
    return mLibraryCombo;
}

rt::Label* MaterialPanel::nameLabel() const
{
    return mName;
}

rt::Label* MaterialPanel::sourceLabel() const
{
    return mSource;
}

rt::ScrollView* MaterialPanel::scrollView() const
{
    return mScroll;
}

const ct::String& MaterialPanel::libraryPath() const
{
    return mLibraryPath;
}

unsigned int MaterialPanel::writeCount() const
{
    return mWrites;
}

} // namespace prisma3d
