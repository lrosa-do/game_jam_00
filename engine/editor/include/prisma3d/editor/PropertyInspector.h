#pragma once

#include "prisma3d/editor/EditorChange.h"
#include "prisma3d/editor/PropertyRow.h"
#include "prisma3d/import/PMeshWriter.h"
#include "prisma3d/scene/NodeClassDB.h"

#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/MenuWidgets.hpp>

#include <ct/function.hpp>
#include <ct/hashmap.hpp>
#include <ct/ptr.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class AnimationPlayer;

class ClassIcons;
class EditorSettings;
class Node;
class Project;

class PropertyInspector
{
public:
    PropertyInspector();
    ~PropertyInspector();
    PropertyInspector(const PropertyInspector&) = delete;
    PropertyInspector& operator=(const PropertyInspector&) = delete;

    bool attach(ig::retained::Widget* parent);
    bool attached() const;
    ig::retained::BoxLayout* root() const;

    void setChangeListener(EditorChangeListener* listener);
    void setProject(const Project* project);
    void setClassIcons(const ClassIcons* icons);
    void setSettings(EditorSettings* settings);
    using MeshSplitHandler =
            ct::Function<bool(Node*, const ct::String&, const ct::Vector<ct::String>&)>;
    void setMeshSplitHandler(const MeshSplitHandler& handler);
    using MeshRewriteHandler = ct::Function<bool(Node*, const ct::String&, const ct::String&,
            const PMeshSurfaceEdit&, ct::String&)>;
    void setMeshRewriteHandler(const MeshRewriteHandler& handler);

    void show(Node* node);
    void forget(const Node* node);
    void update();
    void releaseEditors();

    Node* node() const;
    std::size_t rowCount() const;
    PropertyRow* row(std::size_t index) const;
    PropertyRow* rowNamed(const ct::String& name) const;
    ct::String rowName(std::size_t index) const;
    std::size_t sectionCount() const;
    ig::retained::Collapsible* section(const ct::String& path) const;
    ig::retained::Collapsible* gate(const ct::String& property) const;
    std::size_t gateCount() const;
    unsigned int writeCount() const;
    unsigned int rebuildCount() const;
    ig::retained::Menu* pickerMenu() const;
    static constexpr const char* kClearMaterial = "Clear";
    static constexpr const char* kNoMaterials = "No materials in the project";
    static constexpr const char* kClearNode = "Clear";
    static constexpr const char* kNoNodes = "No matching nodes in the scene";
    static constexpr std::size_t kMaxNodeChoices = 200;

    static bool hiddenByTransform(const Node* node, const ct::String& name);

private:
    struct Link
    {
        PropertyInspector* owner;
    };

    struct Entry
    {
        ct::String name;
        PropertyRow* row = nullptr;
        PropertyValue defaultValue;
        bool hasDefault = false;
    };

    struct Gate
    {
        ct::String name;
        ig::retained::Collapsible* section = nullptr;
    };

    void rebuild(Node* node);
    void clearRows();
    ig::retained::Widget* container(const ct::String& className,
            const ct::Vector<ct::String>& components, std::size_t count, bool closed);
    void addRow(const ct::String& className, const PropertyInfo& info,
            const ct::Vector<ct::String>& sections, const ct::String& label, Node* node,
            bool closed);
    void addGate(const ct::String& className, const PropertyInfo& info, const ct::String& group,
            bool closed);
    void gateEdited(std::size_t index, bool checked);
    void syncGates();
    void syncRow(std::size_t index);
    void rowEdited(std::size_t index, const PropertyValue& value);
    void rowReverted(std::size_t index);
    void rowBrowse(std::size_t index);
    void rowPickMaterial(std::size_t index);
    void rowPickNode(std::size_t index);
    void openFileDialog(std::size_t index, const ct::String& filter);
    void fileChosen(Node* target, const ct::String& name, const ct::String& path);
    void syncMeshStats(std::size_t index);
    void rowSaveMesh(std::size_t index, int remove, int extract);
    void meshDestinationChosen(Node* target, const PMeshSurfaceEdit& edit,
            const ct::String& destination);
    void syncAnimations(std::size_t index);
    void rowImportAnimations(std::size_t index);
    void animationFileChosen(Node* target, const ct::String& path);
    void importAnimations(Node* target, const ct::String& path);
    void removeAnimation(Node* target, const ct::String& name);
    void playAnimation(Node* target, const ct::String& name);
    void stopAnimation(Node* target);
    void seekAnimation(Node* target, const ct::String& name, double time);
    void pauseAnimation(Node* target);
    void applySeek(AnimationPlayer* player, const ct::String& name, double time);
    void poseIfIdle(AnimationPlayer* player);
    void materialChosen(Node* target, const ct::String& name, const ct::String& path);
    void nodeChosen(Node* target, const ct::String& name, const ct::String& path);
    void syncRows();
    void refreshList();
    ct::String signatureOf(const Node* node) const;
    bool write(Entry& entry, const PropertyValue& value);
    bool writeProperty(const ct::String& name, const PropertyValue& value);
    std::size_t indexOf(const PropertyRow* row) const;

    ct::Rc<Link> mLink;
    ig::retained::BoxLayout* mRoot;
    EditorChangeListener* mListener;
    const Project* mProject;
    const ClassIcons* mIcons;
    EditorSettings* mSettings;
    MeshSplitHandler mMeshSplit;
    MeshRewriteHandler mMeshRewrite;
    Node* mNode;
    bool mRestHeld;
    ct::String mClass;
    ct::String mSignature;
    std::uint32_t mRevision;
    ig::retained::Menu* mPickerMenu;
    ct::Vector<Entry> mEntries;
    ct::Vector<Gate> mGates;
    ct::HashMap<ct::String, ig::retained::Widget*> mSections;
    ct::HashMap<ct::String, ig::retained::Widget*> mCategorySections;
    ct::Vector<ct::String> mSectionPaths;
    ct::HashMap<ct::String, bool> mExpanded;
    unsigned int mWrites;
    unsigned int mRebuilds;
};

} // namespace prisma3d
