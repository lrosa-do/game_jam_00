#pragma once

#include "prisma3d/editor/CreatePrimitiveDialog.h"
#include "prisma3d/editor/CreateTerrainDialog.h"
#include "prisma3d/scene/PrimitiveMeshes.h"

#include "mathc.h"

#include <igui/widgets/MenuWidgets.hpp>

#include <ct/ptr.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class HierarchyActions;
class Node;

class CreateMenu
{
public:
    enum Entry : int
    {
        kEntryAddNode,
        kEntrySeparator,
        kEntryEmpty,
        kEntryObject,
        kEntryLight,
        kEntryCamera,
        kEntryAudio,
        kEntryPhysics,
        kEntryEffects,
        kEntryEnvironment,
        kEntryVegetation,
        kEntryCount
    };

    enum Item : int
    {
        kEmpty,
        kCube,
        kSphere,
        kCapsule,
        kCylinder,
        kCone,
        kPlane,
        kTorus,
        kDirectionalLight,
        kPointLight,
        kSpotLight,
        kCamera,
        kAudioSource,
        kAudioListener,
        kStaticBody,
        kRigidBody,
        kCharacterBody,
        kParticles,
        kDecal,
        kReflectionProbe,
        kWorldEnvironment,
        kTerrain,
        kWater,
        kOcean,
        kTree,
        kGrass,
        kForest,
        kItemCount
    };

    static constexpr const char* kTitle = "Create";
    static constexpr const char* kAddNodeLabel = "Add Node...";
    static constexpr const char* kAddNodeShortcut = "Ctrl+A";

    explicit CreateMenu(HierarchyActions& owner);
    ~CreateMenu();
    CreateMenu(const CreateMenu&) = delete;
    CreateMenu& operator=(const CreateMenu&) = delete;

    static const char* label(Item item);
    static const char* className(Item item);
    static bool isPrimitive(Item item);
    static PrimitiveKind primitiveKind(Item item);
    static int entryItemCount(Entry entry);
    static Item entryItem(Entry entry, int index);
    static const char* entryLabel(Entry entry);
    static bool hasSubmenu(Entry entry);

    static Math::Vec3 sunTarget();
    static Math::Vec3 sunFrom();

    ig::retained::Menu* newMenu();
    void populate(ig::retained::Menu* menu);
    void refresh(ig::retained::Menu* menu) const;
    void buildDropdown();
    void showDropdown(float x, float y);
    ig::retained::Menu* dropdown() const;

    void request(Item item);
    bool create(Item item, ct::String& error);
    bool createUnder(Item item, Node* parent, ct::String& error);
    bool createPrimitive(PrimitiveKind kind, const ct::String& name, const PrimitiveRecipe& recipe,
            ct::String& error);

    const PrimitiveRecipe& lastRecipe(PrimitiveKind kind) const;
    CreatePrimitiveDialog* primitiveDialog() const;
    bool createTerrain(const ct::String& name, const TerrainCreateSettings& settings,
            ct::String& error);
    const TerrainCreateSettings& lastTerrain() const;
    CreateTerrainDialog* terrainDialog() const;

private:
    struct Link
    {
        CreateMenu* owner;
    };

    Node* build(Item item, Node* parent, const ct::String& name, const PrimitiveRecipe* recipe);
    Node* buildBody(Item item, Node* parent);
    Node* buildTerrain(Node* parent, const ct::String& name);
    static Node* findTerrain(Node* root);
    Node* buildMesh(Node* parent, const ct::String& name, const PrimitiveRecipe& recipe);
    bool place(Item item, const ct::String& name, const PrimitiveRecipe* recipe, ct::String& error,
            Node* parent = nullptr);
    void openPrimitive(Item item);
    void primitiveAccepted(Item item, const ct::String& name, const PrimitiveRecipe& recipe);
    void openTerrain();
    void terrainAccepted(const ct::String& name, const TerrainCreateSettings& settings);

    HierarchyActions& mOwner;
    ct::Rc<Link> mLink;
    ct::Vector<ig::retained::Menu*> mSubmenus;
    ct::Vector<PrimitiveRecipe> mLast;
    TerrainCreateSettings mTerrain;
    ct::String mBuildError;
    ig::retained::Menu* mDropdown;
};

} // namespace prisma3d
