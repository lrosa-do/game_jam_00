#include "prisma3d/editor/CreateMenu.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/editor/AssetTypes.h"
#include "prisma3d/editor/HierarchyActions.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PropertyValue.h"
#include "prisma3d/scene/Terrain3D.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <igui/widgets/WidgetApp.hpp>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

struct ItemDef
{
    const char* label;
    const char* className;
    int primitive;
};

const ItemDef kItems[CreateMenu::kItemCount] = {
    { "Empty", "Node3D", -1 },
    { "Cube", "MeshInstance3D", static_cast<int>(PrimitiveKind::Box) },
    { "Sphere", "MeshInstance3D", static_cast<int>(PrimitiveKind::Sphere) },
    { "Capsule", "MeshInstance3D", static_cast<int>(PrimitiveKind::Capsule) },
    { "Cylinder", "MeshInstance3D", static_cast<int>(PrimitiveKind::Cylinder) },
    { "Cone", "MeshInstance3D", static_cast<int>(PrimitiveKind::Cone) },
    { "Plane", "MeshInstance3D", static_cast<int>(PrimitiveKind::Plane) },
    { "Torus", "MeshInstance3D", static_cast<int>(PrimitiveKind::Torus) },
    { "Directional Light", "DirectionalLight3D", -1 },
    { "Point Light", "OmniLight3D", -1 },
    { "Spot Light", "SpotLight3D", -1 },
    { "Camera", "Camera3D", -1 },
    { "Audio Source", "AudioStreamPlayer3D", -1 },
    { "Audio Listener", "AudioListener3D", -1 },
    { "Static Body", "StaticBody3D", -1 },
    { "Rigid Body", "RigidBody3D", -1 },
    { "Character Body", "CharacterBody3D", -1 },
    { "Particles", "ParticleSystem3D", -1 },
    { "Decal", "Decal3D", -1 },
    { "Reflection Probe", "ReflectionProbe3D", -1 },
    { "World Environment", "WorldEnvironment", -1 },
    { "Terrain...", "Terrain3D", -1 },
    { "Water", "Water3D", -1 },
    { "Ocean", "Ocean3D", -1 },
    { "Tree", "Tree3D", -1 },
    { "Grass", "Grass3D", -1 },
    { "Forest", "Forest3D", -1 },
};

struct EntryDef
{
    const char* label;
    bool submenu;
    CreateMenu::Item first;
    int count;
};

const EntryDef kEntries[CreateMenu::kEntryCount] = {
    { CreateMenu::kAddNodeLabel, false, CreateMenu::kEmpty, 0 },
    { "", false, CreateMenu::kEmpty, 0 },
    { "Empty", false, CreateMenu::kEmpty, 1 },
    { "3D Object", true, CreateMenu::kCube, 7 },
    { "Light", true, CreateMenu::kDirectionalLight, 3 },
    { "Camera", false, CreateMenu::kCamera, 1 },
    { "Audio", true, CreateMenu::kAudioSource, 2 },
    { "Physics", true, CreateMenu::kStaticBody, 3 },
    { "Effects", true, CreateMenu::kParticles, 3 },
    { "Environment", true, CreateMenu::kWorldEnvironment, 4 },
    { "Vegetation", true, CreateMenu::kTree, 3 },
};

const char* const kShapeNode = "CollisionShape3D";
const char* const kMeshNode = "MeshInstance3D";

Math::Vec3 vectorOf(const PrimitiveRecipe& recipe)
{
    return Math::Vec3(recipe.values[0], recipe.values[1], recipe.values[2]);
}

void enableAll(rt::Menu* menu, bool enabled)
{
    if (!menu) return;
    for (int i = 0; i < menu->actionCount(); ++i)
    {
        rt::MenuAction* action = menu->action(i);
        if (action->isSeparator()) continue;
        action->setEnabled(enabled);
        enableAll(action->submenu(), enabled);
    }
}

} // namespace

CreateMenu::CreateMenu(HierarchyActions& owner)
        : mOwner(owner),
          mLink(ct::make_rc<Link>(Link{ this })),
          mDropdown(nullptr)
{
    const std::size_t kinds = primitiveKindCount();
    for (std::size_t i = 0; i < kinds; ++i)
    {
        PrimitiveRecipe recipe;
        primitiveDefaults(static_cast<PrimitiveKind>(i), recipe);
        mLast.push_back(recipe);
    }
}

CreateMenu::~CreateMenu()
{
    mLink->owner = nullptr;
    for (std::size_t i = 0; i < mSubmenus.size(); ++i) delete mSubmenus[i];
}

const char* CreateMenu::label(Item item)
{
    return item >= 0 && item < kItemCount ? kItems[item].label : "";
}

const char* CreateMenu::className(Item item)
{
    return item >= 0 && item < kItemCount ? kItems[item].className : "";
}

bool CreateMenu::isPrimitive(Item item)
{
    return item >= 0 && item < kItemCount && kItems[item].primitive >= 0;
}

PrimitiveKind CreateMenu::primitiveKind(Item item)
{
    return isPrimitive(item) ? static_cast<PrimitiveKind>(kItems[item].primitive)
                             : PrimitiveKind::Box;
}

int CreateMenu::entryItemCount(Entry entry)
{
    return entry >= 0 && entry < kEntryCount ? kEntries[entry].count : 0;
}

CreateMenu::Item CreateMenu::entryItem(Entry entry, int index)
{
    if (index < 0 || index >= entryItemCount(entry)) return kItemCount;
    return static_cast<Item>(static_cast<int>(kEntries[entry].first) + index);
}

const char* CreateMenu::entryLabel(Entry entry)
{
    return entry >= 0 && entry < kEntryCount ? kEntries[entry].label : "";
}

bool CreateMenu::hasSubmenu(Entry entry)
{
    return entry >= 0 && entry < kEntryCount && kEntries[entry].submenu;
}

Math::Vec3 CreateMenu::sunFrom()
{
    return Math::Vec3(4.0f, 6.0f, 3.0f);
}

Math::Vec3 CreateMenu::sunTarget()
{
    return Math::Vec3(0.0f, 0.0f, 0.0f);
}

rt::Menu* CreateMenu::newMenu()
{
    rt::Menu* menu = new rt::Menu();
    mSubmenus.push_back(menu);
    return menu;
}

void CreateMenu::populate(rt::Menu* menu)
{
    if (!menu) return;
    const ct::Rc<Link> link = mLink;
    rt::MenuAction* add = menu->addAction(kAddNodeLabel,
            [link]
            {
                if (link->owner) link->owner->mOwner.run(HierarchyActions::kAddChild);
            });
    add->setShortcut(kAddNodeShortcut);
    menu->addSeparator();
    for (int e = kEntryEmpty; e < kEntryCount; ++e)
    {
        const Entry entry = static_cast<Entry>(e);
        const auto itemAction = [&](rt::Menu* target, Item item, const char* text)
        {
            target->addAction(text,
                    [link, item]
                    {
                        if (link->owner) link->owner->request(item);
                    });
        };
        if (!hasSubmenu(entry))
        {
            itemAction(menu, entryItem(entry, 0), entryLabel(entry));
            continue;
        }
        rt::MenuAction* parent = menu->addAction(entryLabel(entry));
        rt::Menu* submenu = newMenu();
        for (int i = 0; i < entryItemCount(entry); ++i)
        {
            const Item item = entryItem(entry, i);
            itemAction(submenu, item, label(item));
        }
        parent->setSubmenu(submenu);
    }
}

void CreateMenu::refresh(rt::Menu* menu) const
{
    enableAll(menu, mOwner.canAddChild());
}

void CreateMenu::buildDropdown()
{
    if (mDropdown) return;
    mDropdown = newMenu();
    populate(mDropdown);
}

rt::Menu* CreateMenu::dropdown() const
{
    return mDropdown;
}

void CreateMenu::showDropdown(float x, float y)
{
    buildDropdown();
    refresh(mDropdown);
    mDropdown->exec(x, y);
}

const PrimitiveRecipe& CreateMenu::lastRecipe(PrimitiveKind kind) const
{
    return mLast[static_cast<std::size_t>(kind)];
}

CreatePrimitiveDialog* CreateMenu::primitiveDialog() const
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return nullptr;
    const ct::Vector<rt::FloatWindow*>& floats = app->floats();
    for (std::size_t i = floats.size(); i > 0; --i)
        if (CreatePrimitiveDialog* box = rt::widget_cast<CreatePrimitiveDialog>(floats[i - 1]))
            return box;
    return nullptr;
}

void CreateMenu::request(Item item)
{
    if (!mOwner.canAddChild()) return;
    if (isPrimitive(item))
    {
        openPrimitive(item);
        return;
    }
    if (item == kTerrain)
    {
        openTerrain();
        return;
    }
    ct::String error;
    create(item, error);
    if (!error.empty()) mOwner.showError(error);
}

void CreateMenu::openPrimitive(Item item)
{
    if (CreatePrimitiveDialog* existing = primitiveDialog()) existing->cancel();
    const PrimitiveKind kind = primitiveKind(item);
    rt::WidgetApp& app = rt::WidgetApp::instance();
    CreatePrimitiveDialog* box =
            app.addFloat<CreatePrimitiveDialog>(kind, ct::String(label(item)), lastRecipe(kind));
    const ct::Rc<Link> link = mLink;
    box->created.connect(
            [link, item](const ct::String& name, const PrimitiveRecipe& recipe)
            {
                if (link->owner) link->owner->primitiveAccepted(item, name, recipe);
            });
}

void CreateMenu::primitiveAccepted(Item item, const ct::String& name, const PrimitiveRecipe& recipe)
{
    ct::String error;
    if (createPrimitive(primitiveKind(item), name, recipe, error))
        mLast[static_cast<std::size_t>(recipe.kind)] = recipe;
    if (!error.empty()) mOwner.showError(error);
}

CreateTerrainDialog* CreateMenu::terrainDialog() const
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return nullptr;
    const ct::Vector<rt::FloatWindow*>& floats = app->floats();
    for (std::size_t i = floats.size(); i > 0; --i)
        if (CreateTerrainDialog* box = rt::widget_cast<CreateTerrainDialog>(floats[i - 1]))
            return box;
    return nullptr;
}

const TerrainCreateSettings& CreateMenu::lastTerrain() const
{
    return mTerrain;
}

void CreateMenu::openTerrain()
{
    if (CreateTerrainDialog* existing = terrainDialog()) existing->cancel();
    rt::WidgetApp& app = rt::WidgetApp::instance();
    CreateTerrainDialog* box = app.addFloat<CreateTerrainDialog>(mTerrain,
            &mOwner.mDocument.project(), &mOwner.mDocument.settings());
    const ct::Rc<Link> link = mLink;
    box->created.connect(
            [link](const ct::String& name, const TerrainCreateSettings& settings)
            {
                if (link->owner) link->owner->terrainAccepted(name, settings);
            });
}

void CreateMenu::terrainAccepted(const ct::String& name, const TerrainCreateSettings& settings)
{
    ct::String error;
    createTerrain(name, settings, error);
    if (!error.empty()) mOwner.showError(error);
}

bool CreateMenu::createTerrain(const ct::String& name, const TerrainCreateSettings& settings,
        ct::String& error)
{
    error = ct::String();
    mTerrain = settings;
    return place(kTerrain, name, nullptr, error);
}

bool CreateMenu::create(Item item, ct::String& error)
{
    error = ct::String();
    if (item < 0 || item >= kItemCount) return false;
    if (isPrimitive(item))
        return createPrimitive(primitiveKind(item), ct::String(label(item)),
                lastRecipe(primitiveKind(item)), error);
    return place(item, ct::String(), nullptr, error);
}

bool CreateMenu::createUnder(Item item, Node* parent, ct::String& error)
{
    error = ct::String();
    if (item < 0 || item >= kItemCount || isPrimitive(item) || !parent) return false;
    return place(item, ct::String(), nullptr, error, parent);
}

bool CreateMenu::createPrimitive(PrimitiveKind kind, const ct::String& name,
        const PrimitiveRecipe& recipe, ct::String& error)
{
    error = ct::String();
    Item item = kItemCount;
    for (int i = 0; i < kItemCount; ++i)
        if (kItems[i].primitive == static_cast<int>(kind)) item = static_cast<Item>(i);
    if (item == kItemCount)
    {
        error = ct::String("Can't create a primitive of the kind '") + primitiveKindName(kind) +
                "'.";
        return false;
    }
    PrimitiveRecipe values = recipe;
    values.kind = kind;
    return place(item, name, &values, error);
}

bool CreateMenu::place(Item item, const ct::String& name, const PrimitiveRecipe* recipe,
        ct::String& error, Node* parent)
{
    if (!mOwner.mDocument.scene()) return false;
    if (!NodeClassDB::classExists(ct::String(className(item))))
    {
        error = ct::String("Can't create a node of the class '") + className(item) + "'.";
        return false;
    }
    mBuildError = ct::String();

    ct::Vector<Node*> parents;
    if (parent) parents.push_back(parent);
    else
        parents = mOwner.createParents();
    if (parents.empty()) return false;
    const Math::Vec3 focus = mOwner.focusPoint();

    const ct::Vector<Node*> selectionBefore = mOwner.mSelection.selectedNodes();
    EditorUndoRedoManager::Applying applying(&mOwner.mDocument.undoRedo());
    mOwner.mMadeEditable.clear();
    ct::Vector<Node*> created;
    for (std::size_t i = 0; i < parents.size(); ++i)
    {
        Node* node = build(item, parents[i], name, recipe);
        if (!node) continue;
        if (Node3D* spatial = node->castTo<Node3D>())
        {
            spatial->setGlobalPosition(focus);
            if (item == kDirectionalLight)
                spatial->lookAtFromPosition(focus, focus + sunTarget() - sunFrom());
        }
        created.push_back(node);
    }
    if (created.empty())
    {
        error = mBuildError;
        return false;
    }

    mOwner.mSelection.clear();
    for (std::size_t i = 0; i < created.size(); ++i) mOwner.mSelection.add(created[i]);
    mOwner.recordCreated(HierarchyActions::kAddAction, created, selectionBefore);
    return true;
}

Node* CreateMenu::build(Item item, Node* parent, const ct::String& name,
        const PrimitiveRecipe* recipe)
{
    if (recipe) return buildMesh(parent, name, *recipe);
    if (item == kStaticBody || item == kRigidBody || item == kCharacterBody)
        return buildBody(item, parent);
    if (item == kTerrain) return buildTerrain(parent, name);
    Node* node = mOwner.createUnder(parent, ct::String(className(item)));
    if (node && (item == kGrass || item == kForest))
    {
        if (Node* terrain = findTerrain(mOwner.mDocument.scene()))
            NodeClassDB::setProperty(node, "terrain",
                    PropertyValue::fromNodePath(node->getPathTo(terrain)));
    }
    return node;
}

Node* CreateMenu::findTerrain(Node* root)
{
    if (!root) return nullptr;
    ct::Vector<Node*> stack;
    stack.push_back(root);
    while (!stack.empty())
    {
        Node* node = stack.back();
        stack.pop_back();
        if (node->castTo<Terrain3D>()) return node;
        for (std::size_t i = 0; i < node->getChildCount(); ++i) stack.push_back(node->getChild(i));
    }
    return nullptr;
}

Node* CreateMenu::buildMesh(Node* parent, const ct::String& name, const PrimitiveRecipe& recipe)
{
    Node* node = mOwner.createUnder(parent, ct::String(kMeshNode));
    if (!node) return nullptr;
    node->setName(name);
    NodeClassDB::setProperty(node, "mesh",
            PropertyValue::fromResourcePath(primitiveRecipePath(recipe)));
    return node;
}

Node* CreateMenu::buildTerrain(Node* parent, const ct::String& name)
{
    TerrainImportData data;
    data.terrainSize = mTerrain.size;
    data.worldSize = mTerrain.worldSize;
    data.minBatchSize = 33;
    data.maxBatchSize = 65;
    data.lightMapSize = 512;
    data.compositeMapAmbient = Math::Vec3(0.25f, 0.25f, 0.25f);
    data.compositeMapDiffuse = Math::Vec3(1.0f, 1.0f, 1.0f);
    Pixmap heightmap;
    ct::Vector<std::uint16_t> heights16;
    if (!mTerrain.heightmap.empty())
    {
        const ct::Vector<std::uint8_t> bytes =
                FileSystem::getSingleton().readBinary(mTerrain.heightmap);
        const ct::String extension = AssetTypes::extensionOf(mTerrain.heightmap);
        const bool raw = extension == "raw" || extension == "r16";
        bool loaded = false;
        if (bytes.empty()) loaded = false;
        else if (raw)
        {
            const std::size_t samples = bytes.size() / 2;
            std::size_t side = 1;
            while ((side + 1) * (side + 1) <= samples) ++side;
            if (side * side == samples && side >= 2)
            {
                heights16.resize(samples);
                for (std::size_t i = 0; i < samples; ++i)
                    heights16[i] =
                            static_cast<std::uint16_t>(bytes[i * 2] | (bytes[i * 2 + 1] << 8));
                data.inputImage16 = heights16.data();
                data.inputImage16Width = static_cast<int>(side);
                data.inputImage16Height = static_cast<int>(side);
                loaded = true;
            }
        }
        else if (Pixmap::is_16_bit_in_memory(bytes.data(),
                         static_cast<std::uint32_t>(bytes.size())))
        {
            int width = 0;
            int height = 0;
            if (Pixmap::load_16_from_memory(bytes.data(), static_cast<std::uint32_t>(bytes.size()),
                        heights16, width, height))
            {
                data.inputImage16 = heights16.data();
                data.inputImage16Width = width;
                data.inputImage16Height = height;
                loaded = true;
            }
        }
        else if (heightmap.load_from_memory(bytes.data(), static_cast<std::uint32_t>(bytes.size())))
        {
            data.inputImage = &heightmap;
            loaded = true;
        }
        if (!loaded)
        {
            mBuildError = ct::String("Can't read the heightmap '") + mTerrain.heightmap +
                          "' (8 or 16 bit image, or a square 16 bit .raw).";
            return nullptr;
        }
        data.inputScale = mTerrain.heightScale;
    }
    else
        data.constantHeight = mTerrain.flatHeight;

    Node* node = mOwner.createUnder(parent, ct::String(className(kTerrain)));
    Terrain3D* terrain = node ? node->castTo<Terrain3D>() : nullptr;
    if (!terrain)
    {
        if (node)
        {
            parent->removeChild(node);
            node->queueFree();
        }
        mBuildError = "Can't create a Terrain3D node.";
        return nullptr;
    }
    if (!name.empty()) node->setName(name);
    if (!terrain->load(data))
    {
        parent->removeChild(node);
        node->queueFree();
        mBuildError = "The terrain could not be created; see the console.";
        return nullptr;
    }
    return node;
}

Node* CreateMenu::buildBody(Item item, Node* parent)
{
    Node* body = mOwner.createUnder(parent, ct::String(className(item)));
    if (!body) return nullptr;

    const bool character = item == kCharacterBody;
    PrimitiveRecipe recipe;
    primitiveDefaults(character ? PrimitiveKind::Capsule : PrimitiveKind::Box, recipe);

    Node* shapeNode = body->createChild(kShapeNode, true);
    CollisionShape3D* shape = shapeNode ? shapeNode->castTo<CollisionShape3D>() : nullptr;
    if (!shape) return body;

    if (character)
    {
        const float radius = recipe.values[0];
        const float height = recipe.values[1];
        shape->setCapsule(radius, height + 2.0f * radius);
    }
    else
        shape->setBox(vectorOf(recipe));
    return body;
}

} // namespace prisma3d
