#include "prisma3d/editor/PropertyInspector.h"

#include "prisma3d/editor/ClassIcons.h"
#include "prisma3d/editor/EditorTreeEdits.h"
#include "prisma3d/editor/EditorUndoRedoManager.h"
#include "prisma3d/editor/FileDialogMemory.h"
#include "prisma3d/editor/PropertyNames.h"
#include "prisma3d/import/PMeshFormat.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/render/MaterialLibrary.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/AnimationPlayer.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <igui/widgets/DialogWidgets.hpp>
#include <igui/widgets/FileDialog.hpp>
#include <igui/widgets/Theme.hpp>
#include <igui/widgets/WidgetApp.hpp>

#include "platform.h"

#include <ct/sort.hpp>

#include <math.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const float kSectionPadding = 4.0f;
const float kSectionSpacing = 4.0f;
const float kApproximate = 1.0e-5f;

bool approximately(float a, float b)
{
    return fabsf(a - b) <= kApproximate * (1.0f + fabsf(a) + fabsf(b));
}

bool different(const PropertyValue& a, const PropertyValue& b)
{
    if (a.type() != b.type()) return true;
    switch (a.type())
    {
        case PropertyType::Float:
            return !approximately(static_cast<float>(a.asFloat()), static_cast<float>(b.asFloat()));
        case PropertyType::Vec2:
        {
            const Math::Vec2 x = a.asVec2();
            const Math::Vec2 y = b.asVec2();
            return !approximately(x.x, y.x) || !approximately(x.y, y.y);
        }
        case PropertyType::Vec3:
        {
            const Math::Vec3 x = a.asVec3();
            const Math::Vec3 y = b.asVec3();
            return !approximately(x.x, y.x) || !approximately(x.y, y.y) || !approximately(x.z, y.z);
        }
        case PropertyType::Vec4:
        case PropertyType::Color:
        {
            const Math::Vec4 x = a.type() == PropertyType::Color ? a.asColor() : a.asVec4();
            const Math::Vec4 y = b.type() == PropertyType::Color ? b.asColor() : b.asVec4();
            return !approximately(x.x, y.x) || !approximately(x.y, y.y) ||
                   !approximately(x.z, y.z) || !approximately(x.w, y.w);
        }
        case PropertyType::Quat:
        {
            const Math::Quaternion x = a.asQuat();
            const Math::Quaternion y = b.asQuat();
            return fabsf(x.Dot(y)) < 1.0f - kApproximate;
        }
        default:
            return a != b;
    }
}

ct::String firstSlice(const ct::String& text)
{
    const std::size_t comma = text.find(',');
    return comma == ct::String::npos ? text : text.substr(0, comma);
}

struct MaterialChoice
{
    ct::String library;
    ct::String path;
};

bool collectLibrary(const char* path, PathType type, void* user)
{
    if (type == PATH_TYPE_FILE && path_has_extension(path, ".mat"))
        static_cast<ct::Vector<ct::String>*>(user)->push_back(ct::String(path));
    return true;
}

void collectMaterials(const Project& project, ct::Vector<MaterialChoice>& out)
{
    ct::Vector<ct::String> files;
    fs_enumerate_directory(project.root().c_str(), true, &collectLibrary, &files);
    ct::sort(files.begin(), files.end(),
            [](const ct::String& a, const ct::String& b) { return a < b; });
    for (std::size_t i = 0; i < files.size(); ++i)
    {
        ct::String relative;
        if (!project.relativePath(files[i], relative)) continue;
        MaterialLibraryData data;
        ct::String error;
        if (!MaterialLibraryFormat::load(files[i], data, error)) continue;
        for (std::size_t m = 0; m < data.materials.size(); ++m)
        {
            MaterialChoice choice;
            choice.library = relative;
            choice.path = relative + "::" + data.materials[m].info.name;
            out.push_back(choice);
        }
    }
}

bool acceptsNode(const Node* candidate, const ct::Vector<ct::String>& types)
{
    if (types.empty()) return true;
    for (std::size_t i = 0; i < types.size(); ++i)
        if (candidate->isClass(types[i].c_str())) return true;
    return false;
}

void collectNodes(const Node* origin, const Node* node, const ct::Vector<ct::String>& types,
        ct::Vector<ct::String>& out)
{
    if (out.size() >= PropertyInspector::kMaxNodeChoices) return;
    if (node != origin && !node->isInternal() && acceptsNode(node, types))
    {
        const ct::String path = origin->getPathTo(node);
        if (!path.empty()) out.push_back(path);
    }
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        collectNodes(origin, node->getChild(i), types, out);
}

void appendChain(const Node* node, ct::Vector<ct::String>& chain)
{
    for (ct::String name(node->getClassName()); !name.empty();
            name = NodeClassDB::getParentClass(name))
        chain.push_back(name);
}

} // namespace

PropertyInspector::PropertyInspector()
        : mLink(ct::make_rc<Link>(Link{ this })),
          mRoot(nullptr),
          mListener(nullptr),
          mProject(nullptr),
          mIcons(nullptr),
          mSettings(nullptr),
          mNode(nullptr),
          mRestHeld(false),
          mRevision(0),
          mPickerMenu(nullptr),
          mWrites(0),
          mRebuilds(0)
{
}

PropertyInspector::~PropertyInspector()
{
    mLink->owner = nullptr;
    if (mPickerMenu)
    {
        rt::WidgetApp* app = rt::WidgetApp::existing();
        if (app && app->popup() == mPickerMenu) app->closePopup();
    }
    delete mPickerMenu;
}

rt::Menu* PropertyInspector::pickerMenu() const
{
    return mPickerMenu;
}

bool PropertyInspector::attach(rt::Widget* parent)
{
    if (mRoot || !parent) return false;
    mRoot = parent->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mRoot->setPadding(0.0f);
    mRoot->setSpacing(kSectionSpacing);
    return true;
}

bool PropertyInspector::attached() const
{
    return mRoot != nullptr;
}

rt::BoxLayout* PropertyInspector::root() const
{
    return mRoot;
}

void PropertyInspector::setChangeListener(EditorChangeListener* listener)
{
    mListener = listener;
}

void PropertyInspector::setProject(const Project* project)
{
    mProject = project;
}

void PropertyInspector::setClassIcons(const ClassIcons* icons)
{
    mIcons = icons;
}

void PropertyInspector::setSettings(EditorSettings* settings)
{
    mSettings = settings;
}

bool PropertyInspector::hiddenByTransform(const Node* node, const ct::String& name)
{
    if (!node || !node->castTo<Node3D>()) return false;
    return name == "position" || name == "rotation" || name == "scale" || name == "quaternion" ||
           name == "visible";
}

void PropertyInspector::clearRows()
{
    if (!mRoot) return;
    const ct::Vector<rt::Widget*> children = mRoot->children();
    for (std::size_t i = 0; i < children.size(); ++i) mRoot->removeChild(children[i]);
    mEntries.clear();
    mGates.clear();
    mSections.clear();
    mCategorySections.clear();
    mSectionPaths.clear();
}

void PropertyInspector::forget(const Node* node)
{
    if (mNode == node) mNode = nullptr;
}

void PropertyInspector::releaseEditors()
{
    for (std::size_t i = 0; i < mEntries.size(); ++i)
    {
        PropertyRow* row = mEntries[i].row;
        for (int axis = 0; axis < row->componentCount(); ++axis)
            if (row->field(axis)->isEditing()) row->field(axis)->cancelEdit();
        if (row->swatch()) row->swatch()->close();
        if (row->text() && row->text()->isFocused())
            if (rt::WidgetApp* app = rt::WidgetApp::existing()) app->setFocused(nullptr);
    }
}

void PropertyInspector::show(Node* node)
{
    if (!mRoot) return;
    releaseEditors();
    if (!node)
    {
        if (mNode || !mClass.empty())
        {
            mNode = nullptr;
            mClass = ct::String();
            clearRows();
            if (rt::WidgetApp* app = rt::WidgetApp::existing()) app->requestLayout();
        }
        return;
    }

    if (node != mNode) mRestHeld = false;
    mNode = node;
    const ct::String className(node->getClassName());
    const ct::String signature = signatureOf(node);
    mRevision = node->getPropertyListRevision();
    if (className != mClass || signature != mSignature)
    {
        mClass = className;
        mSignature = signature;
        rebuild(node);
    }
    syncRows();
    if (rt::WidgetApp* app = rt::WidgetApp::existing()) app->requestLayout();
}

rt::Collapsible* PropertyInspector::gate(const ct::String& property) const
{
    for (std::size_t i = 0; i < mGates.size(); ++i)
        if (mGates[i].name == property) return mGates[i].section;
    return nullptr;
}

std::size_t PropertyInspector::gateCount() const
{
    return mGates.size();
}

void PropertyInspector::syncGates()
{
    if (!mNode) return;
    for (std::size_t i = 0; i < mGates.size(); ++i)
    {
        PropertyValue current;
        if (NodeClassDB::getProperty(mNode, mGates[i].name, current))
            mGates[i].section->setChecked(current.asBool());
    }
}

void PropertyInspector::gateEdited(std::size_t index, bool checked)
{
    if (!mNode || index >= mGates.size()) return;
    if (!writeProperty(mGates[index].name, PropertyValue::fromBool(checked))) return;
    syncGates();
}

void PropertyInspector::addGate(const ct::String& className, const PropertyInfo& info,
        const ct::String& group, bool closed)
{
    ct::Vector<ct::String> path;
    path.push_back(group);
    rt::Widget* parent = container(className, path, 1, closed);
    rt::Collapsible* section = parent ? rt::widget_cast<rt::Collapsible>(parent) : nullptr;
    if (!section) return;
    section->setCheckable(true);
    Gate entry;
    entry.name = info.name;
    entry.section = section;
    const std::size_t index = mGates.size();
    mGates.push_back(entry);
    const ct::Rc<Link> link = mLink;
    section->checkedChanged.connect(
            [link, index](bool checked)
            {
                if (link->owner) link->owner->gateEdited(index, checked);
            });
}

void PropertyInspector::syncRows()
{
    syncGates();
    for (std::size_t i = 0; i < mEntries.size(); ++i)
    {
        PropertyValue current;
        if (NodeClassDB::getProperty(mNode, mEntries[i].name, current))
            mEntries[i].row->setValue(current);
        syncRow(i);
    }
}

ct::String PropertyInspector::signatureOf(const Node* node) const
{
    ct::Vector<ct::String> chain;
    appendChain(node, chain);
    ct::String signature;
    for (std::size_t c = chain.size(); c > 0; --c)
    {
        ct::Vector<PropertyInfo> list;
        NodeClassDB::getInstancePropertyList(node, chain[c - 1], list, true);
        for (std::size_t i = 0; i < list.size(); ++i)
        {
            const PropertyInfo& info = list[i];
            signature += info.name;
            signature += '|';
            signature.append_number(static_cast<int>(info.hint));
            signature += '|';
            signature += info.hintString;
            signature += '|';
            signature.append_number(static_cast<unsigned long long>(info.usage));
            signature += '\n';
        }
    }
    return signature;
}

void PropertyInspector::refreshList()
{
    const std::uint32_t revision = mNode->getPropertyListRevision();
    if (revision == mRevision) return;
    mRevision = revision;
    const ct::String signature = signatureOf(mNode);
    if (signature == mSignature) return;
    releaseEditors();
    mSignature = signature;
    rebuild(mNode);
    syncRows();
    if (rt::WidgetApp* app = rt::WidgetApp::existing()) app->requestLayout();
}

rt::Widget* PropertyInspector::container(const ct::String& className,
        const ct::Vector<ct::String>& components, std::size_t count, bool closed)
{
    rt::Widget* parent = mRoot;
    ct::String path;
    for (std::size_t i = 0; i < count; ++i)
    {
        if (i > 0) path += '/';
        path += components[i];
        rt::Widget** found = mCategorySections.find(path);
        if (found)
        {
            parent = *found;
            continue;
        }
        const ct::String key = className + "|" + path;
        bool expanded = !(closed && i == 0);
        if (const bool* remembered = mExpanded.find(key)) expanded = *remembered;

        rt::Collapsible* section = parent->createChild<rt::Collapsible>(components[i], expanded);
        section->setContentPadding(kSectionPadding);
        section->setContentSpacing(kSectionSpacing);
        const ct::Rc<Link> link = mLink;
        section->expandedChanged.connect(
                [link, key](bool value)
                {
                    if (link->owner) link->owner->mExpanded.put(key, value);
                });
        mCategorySections.put(path, section);
        if (!mSections.find(path)) mSections.put(path, section);
        mSectionPaths.push_back(path);
        parent = section;
    }
    return parent;
}

void PropertyInspector::addRow(const ct::String& className, const PropertyInfo& info,
        const ct::Vector<ct::String>& sections, const ct::String& label, Node* node, bool closed)
{
    rt::Widget* parent =
            sections.empty() ? mRoot : container(className, sections, sections.size(), closed);
    PropertyRow* row = parent->createChild<PropertyRow>(info, label);
    if (row->kind() == PropertyRow::Kind::None)
    {
        parent->removeChild(row);
        return;
    }

    Entry entry;
    entry.name = info.name;
    entry.row = row;
    entry.hasDefault = NodeClassDB::getDefaultPropertyValue(node->getClassName(), info.name,
            entry.defaultValue);
    const std::size_t index = mEntries.size();
    mEntries.push_back(entry);

    const ct::Rc<Link> link = mLink;
    row->edited.connect(
            [link, index](const PropertyValue& value)
            {
                if (link->owner) link->owner->rowEdited(index, value);
            });
    row->revertRequested.connect(
            [link, index]
            {
                if (link->owner) link->owner->rowReverted(index);
            });
    row->browseRequested.connect(
            [link, index]
            {
                if (link->owner) link->owner->rowBrowse(index);
            });
    row->nodePickRequested.connect(
            [link, index]
            {
                if (link->owner) link->owner->rowPickNode(index);
            });
    row->materialPickRequested.connect(
            [link, index]
            {
                if (link->owner) link->owner->rowPickMaterial(index);
            });
    row->animationImportRequested.connect(
            [link, index]
            {
                if (link->owner) link->owner->rowImportAnimations(index);
            });
    row->animationDropped.connect(
            [link, node](const ct::String& path)
            {
                if (link->owner) link->owner->animationFileChosen(node, path);
            });
    row->animationRemoveRequested.connect(
            [link, node](const ct::String& name)
            {
                if (link->owner) link->owner->removeAnimation(node, name);
            });
    row->animationPlayRequested.connect(
            [link, node](const ct::String& name)
            {
                if (link->owner) link->owner->playAnimation(node, name);
            });
    row->animationStopRequested.connect(
            [link, node]
            {
                if (link->owner) link->owner->stopAnimation(node);
            });
    row->animationSeekRequested.connect(
            [link, node](const ct::String& name, float time)
            {
                if (link->owner) link->owner->seekAnimation(node, name, time);
            });
    row->animationPauseRequested.connect(
            [link, node]
            {
                if (link->owner) link->owner->pauseAnimation(node);
            });
    row->meshSaveRequested.connect(
            [link, index]
            {
                if (link->owner) link->owner->rowSaveMesh(index, -1, -1);
            });
    row->meshRemoveRequested.connect(
            [link, index](int surface)
            {
                if (link->owner) link->owner->rowSaveMesh(index, surface, -1);
            });
    row->meshExtractRequested.connect(
            [link, index](int surface)
            {
                if (link->owner) link->owner->rowSaveMesh(index, -1, surface);
            });
}

void PropertyInspector::rebuild(Node* node)
{
    clearRows();
    ++mRebuilds;

    ct::Vector<ct::String> chain;
    appendChain(node, chain);

    const ct::String className(node->getClassName());
    for (std::size_t c = chain.size(); c > 0; --c)
    {
        ct::Vector<PropertyInfo> list;
        NodeClassDB::getInstancePropertyList(node, chain[c - 1], list, true);

        bool shown = false;
        for (std::size_t i = 0; i < list.size() && !shown; ++i)
        {
            const PropertyInfo& info = list[i];
            if (info.usage & (PropertyUsage::Group | PropertyUsage::Subgroup)) continue;
            shown = (info.usage & PropertyUsage::Editor) && !hiddenByTransform(node, info.name);
        }
        if (!shown) continue;
        mRoot->createChild<CategoryHeader>(chain[c - 1],
                mIcons ? mIcons->icon(chain[c - 1]) : rt::IconId::None);
        mCategorySections.clear();

        ct::String group;
        bool groupClosed = false;
        ct::String groupBase;
        ct::String subgroup;
        ct::String subgroupBase;

        for (std::size_t i = 0; i < list.size(); ++i)
        {
            const PropertyInfo& info = list[i];
            if (info.usage & PropertyUsage::Subgroup)
            {
                subgroup = info.name;
                subgroupBase = firstSlice(info.hintString);
                continue;
            }
            if (info.usage & PropertyUsage::Group)
            {
                group = info.name;
                groupClosed = (info.usage & PropertyUsage::GroupClosed) != 0;
                groupBase = firstSlice(info.hintString);
                subgroup = ct::String();
                subgroupBase = ct::String();
                continue;
            }
            if (!(info.usage & PropertyUsage::Editor)) continue;
            if (hiddenByTransform(node, info.name)) continue;
            if (info.hint == PropertyHint::GroupEnable && !group.empty())
            {
                addGate(className, info, group, groupClosed);
                continue;
            }

            ct::String path = info.name;
            if (!subgroup.empty() && !subgroupBase.empty())
            {
                if (path.starts_with(subgroupBase)) path = path.substr(subgroupBase.size());
                else if (subgroupBase.starts_with(path))
                {
                }
                else
                    subgroup = ct::String();
            }
            if (!group.empty() && !groupBase.empty() && subgroup.empty())
            {
                if (path.starts_with(groupBase)) path = path.substr(groupBase.size());
                else if (groupBase.starts_with(path))
                {
                }
                else
                {
                    group = ct::String();
                    subgroup = ct::String();
                }
            }
            if (!subgroup.empty()) path = subgroup + "/" + path;
            if (!group.empty()) path = group + "/" + path;

            ct::Vector<ct::String> components;
            std::size_t start = 0;
            for (std::size_t k = 0; k <= path.size(); ++k)
            {
                if (k != path.size() && path[k] != '/') continue;
                components.push_back(path.substr(start, k - start));
                start = k + 1;
            }
            const ct::String last = components.back();
            components.pop_back();

            ct::Vector<ct::String> sections;
            for (std::size_t k = 0; k < components.size(); ++k)
            {
                const bool isGroup = k == 0 && !group.empty() && components[k] == group;
                const bool isSubgroup = k == 1 && !subgroup.empty() && components[k] == subgroup;
                sections.push_back(isGroup || isSubgroup ? components[k]
                                                         : capitalizePropertyName(components[k]));
            }
            ct::String label = capitalizePropertyName(last);
            if (const char* named = propertyLabelOverride(info.name)) label = named;
            addRow(className, info, sections, label, node, groupClosed && !group.empty());
        }
    }
}

std::size_t PropertyInspector::indexOf(const PropertyRow* row) const
{
    for (std::size_t i = 0; i < mEntries.size(); ++i)
        if (mEntries[i].row == row) return i;
    return mEntries.size();
}

void PropertyInspector::syncMeshStats(std::size_t index)
{
    MeshPropertyEditor* editor = mEntries[index].row->meshEditor();
    if (!editor) return;
    const MeshInstance3D* instance = mNode ? mNode->castTo<MeshInstance3D>() : nullptr;
    const MeshStore* store = MeshStore::singleton();
    const MeshInfo* info = instance && store ? store->get(instance->getMesh()) : nullptr;
    if (!info)
    {
        editor->clearStats();
        editor->clearSurfaces();
        return;
    }
    const MaterialStore* materials = MaterialStore::singleton();
    ct::Vector<MeshPropertyEditor::Surface> surfaces;
    std::uint32_t triangles = 0;
    bool skinned = false;
    for (std::size_t i = 0; i < info->surfaces.size(); ++i)
    {
        const SurfaceInfo& surface = info->surfaces[i];
        const std::uint32_t count = surface.indexCount ? surface.indexCount : surface.vertexCount;
        triangles += count / 3;
        skinned = skinned || surface.skinned();
        MeshPropertyEditor::Surface row;
        const MaterialInfo* material = materials ? materials->get(surface.material) : nullptr;
        if (material) row.material = material->name;
        row.triangles = count / 3;
        surfaces.push_back(row);
    }
    editor->setStats(static_cast<std::uint32_t>(info->surfaces.size()), triangles);
    const bool editable =
            !skinned && !info->path.empty() && editor->mode() == MeshPropertyEditor::Mode::Model;
    editor->setSurfaces(surfaces, editable);
}

void PropertyInspector::setMeshSplitHandler(const MeshSplitHandler& handler)
{
    mMeshSplit = handler;
}

void PropertyInspector::setMeshRewriteHandler(const MeshRewriteHandler& handler)
{
    mMeshRewrite = handler;
}

void PropertyInspector::rowSaveMesh(std::size_t index, int remove, int extract)
{
    if (index >= mEntries.size() || !mNode) return;
    const MeshInstance3D* instance = mNode->castTo<MeshInstance3D>();
    if (!instance) return;
    const ct::String path = sceneres::pathOf(instance->getMesh());
    if (path.empty()) return;
    const ct::String file = MeshPropertyEditor::fileOf(path);
    const ct::String absolute =
            mProject && mProject->isOpen() ? mProject->absolutePath(file) : file;
    const std::size_t slash = absolute.rfind('/');
    const ct::String folder = slash == ct::String::npos ? ct::String() : absolute.substr(0, slash);
    ct::String stem = slash == ct::String::npos ? absolute : absolute.substr(slash + 1);
    const std::size_t dot = stem.rfind('.');
    if (dot != ct::String::npos) stem = stem.substr(0, dot);

    PMeshSurfaceEdit edit;
    edit.mesh = MeshPropertyEditor::partOf(path);
    if (remove >= 0) edit.remove.push_back(static_cast<std::uint32_t>(remove));
    if (extract >= 0) edit.extract.push_back(static_cast<std::uint32_t>(extract));
    const ct::String extension = ct::String(".") + pmesh::kMeshExtension;
    if ((remove >= 0 || extract >= 0) && file.ends_with(extension.c_str()) && mMeshRewrite)
    {
        ct::String error;
        if (!mMeshRewrite(mNode, file, absolute, edit, error))
        {
            log_error("editor: could not edit the mesh: %s", error.c_str());
            rt::Toast::show(ct::String("Could not edit the mesh: ") + error,
                    rt::Toast::Type::Error);
        }
        update();
        return;
    }

    rt::WidgetApp& app = rt::WidgetApp::instance();
    rt::FileDialog* dialog = app.addFloat<rt::FileDialog>("Save mesh as");
    dialog->setMode(rt::FileDialog::Mode::Save);
    dialog->setFilter(ct::String("*.") + pmesh::kMeshExtension);
    FileDialogMemory::prepare(*dialog, mSettings, EditorSettings::kFolderFile, mProject);
    if (!folder.empty()) dialog->setPath(folder);
    dialog->setFileName(stem + "_edit." + pmesh::kMeshExtension);
    const ct::Rc<Link> link = mLink;
    Node* target = mNode;
    dialog->accepted.connect(
            [link, target, edit](const ct::String& chosen)
            {
                if (link->owner) link->owner->meshDestinationChosen(target, edit, chosen);
            });
}

void PropertyInspector::meshDestinationChosen(Node* target, const PMeshSurfaceEdit& edit,
        const ct::String& destination)
{
    MeshInstance3D* instance =
            target && target == mNode ? target->castTo<MeshInstance3D>() : nullptr;
    AssetLoader* loader = AssetLoader::singleton();
    MaterialStore* materials = MaterialStore::singleton();
    if (!instance || !loader || !materials) return;
    const ct::String path = sceneres::pathOf(instance->getMesh());
    const ct::String file = MeshPropertyEditor::fileOf(path);
    const ct::String source = mProject && mProject->isOpen() ? mProject->absolutePath(file) : file;
    ct::String targetPath = destination;
    const ct::String extension = ct::String(".") + pmesh::kMeshExtension;
    if (!targetPath.ends_with(extension.c_str())) targetPath += extension;
    ct::String stored = targetPath;
    if (mProject && mProject->isOpen() && !mProject->relativePath(targetPath, stored))
    {
        rt::Toast::show("Choose a file inside the project folder", rt::Toast::Type::Warning);
        return;
    }
    ct::Vector<ct::String> extracted;
    ct::String error;
    if (!editModelToPMesh(*loader, *materials, source, edit, targetPath, extracted, error))
    {
        log_error("editor: could not save the mesh: %s", error.c_str());
        rt::Toast::show(ct::String("Could not save the mesh: ") + error, rt::Toast::Type::Error);
        return;
    }
    const ct::String part = MeshPropertyEditor::partOf(path);
    ct::String newPath = stored;
    if (!part.empty()) newPath += ct::String(MeshPropertyEditor::kPartSeparator) + part;
    if (extracted.empty() || !mMeshSplit)
        writeProperty("mesh", PropertyValue::fromResourcePath(newPath));
    else
    {
        ct::Vector<ct::String> pieces;
        for (std::size_t i = 0; i < extracted.size(); ++i)
            pieces.push_back(stored + MeshPropertyEditor::kPartSeparator + extracted[i]);
        mMeshSplit(target, newPath, pieces);
    }
    update();
}

void PropertyInspector::syncAnimations(std::size_t index)
{
    AnimationLibraryEditor* editor = mEntries[index].row->animationEditor();
    if (!editor) return;
    AnimationPlayer* player = mNode ? mNode->castTo<AnimationPlayer>() : nullptr;
    if (player) poseIfIdle(player);
    ct::Vector<AnimationLibraryEditor::Clip> clips;
    if (player)
    {
        const bool playing = player->isPlaying();
        const ct::String& current = player->getCurrentAnimation();
        for (std::size_t i = 0; i < player->getAnimationCount(); ++i)
        {
            AnimationLibraryEditor::Clip clip;
            clip.name = player->getAnimationName(i);
            const AnimationClip* data = player->getAnimation(clip.name);
            clip.duration = data ? data->duration() : 0.0f;
            clip.playing = playing && current == clip.name;
            clip.current = current == clip.name;
            clip.position =
                    clip.current ? static_cast<float>(player->getCurrentAnimationPosition()) : 0.0f;
            clips.push_back(clip);
        }
    }
    editor->setClips(clips);
}

void PropertyInspector::rowImportAnimations(std::size_t index)
{
    if (index >= mEntries.size() || !mNode) return;
    const Entry& entry = mEntries[index];
    rt::WidgetApp& app = rt::WidgetApp::instance();
    rt::FileDialog* dialog = app.addFloat<rt::FileDialog>("Import animations");
    dialog->setMode(rt::FileDialog::Mode::Open);
    const ct::String filter = PropertyRow::fileFilter(entry.row->info());
    if (!filter.empty()) dialog->setFilter(filter);
    FileDialogMemory::prepare(*dialog, mSettings, EditorSettings::kFolderFile, mProject);
    const ct::Rc<Link> link = mLink;
    Node* target = mNode;
    EditorSettings* settings = mSettings;
    dialog->accepted.connect(
            [link, target, settings](const ct::String& path)
            {
                FileDialogMemory::remember(settings, EditorSettings::kFolderFile, path);
                if (link->owner) link->owner->animationFileChosen(target, path);
            });
}

void PropertyInspector::animationFileChosen(Node* target, const ct::String& path)
{
    if (!target || target != mNode) return;
    ct::String stored = path;
    if (mProject && mProject->isOpen() && !mProject->relativePath(path, stored))
    {
        rt::Toast::show("Choose a file inside the project folder", rt::Toast::Type::Warning);
        return;
    }
    importAnimations(target, stored);
}

namespace
{

void animationNames(const AnimationPlayer& player, ct::Vector<ct::String>& out)
{
    out.clear();
    for (std::size_t i = 0; i < player.getAnimationCount(); ++i)
        out.push_back(player.getAnimationName(i));
}

bool contains(const ct::Vector<ct::String>& names, const ct::String& name)
{
    for (std::size_t i = 0; i < names.size(); ++i)
        if (names[i] == name) return true;
    return false;
}

} // namespace

void PropertyInspector::importAnimations(Node* target, const ct::String& path)
{
    AnimationPlayer* player = target ? target->castTo<AnimationPlayer>() : nullptr;
    if (!player) return;
    ct::Vector<ct::String> before;
    animationNames(*player, before);
    const int added = player->importAnimations(path);
    if (added <= 0)
    {
        rt::Toast::show(added < 0 ? "The file could not be read; see the console"
                                  : "No animation in that file matches the skeleton",
                rt::Toast::Type::Warning);
        return;
    }
    ct::Vector<ct::String> after;
    animationNames(*player, after);
    ct::Vector<ct::String> names;
    for (std::size_t i = 0; i < after.size(); ++i)
        if (!contains(before, after[i])) names.push_back(after[i]);

    EditorUndoRedoManager* undo = mListener ? mListener->undoRedoManager() : nullptr;
    if (undo)
    {
        undo->createAction(ct::String("Import Animations"));
        undo->addDoMethod([player, path] { player->importAnimations(path); }, player);
        undo->addUndoMethod(
                [player, names]
                {
                    for (std::size_t i = 0; i < names.size(); ++i)
                        player->removeAnimation(names[i]);
                },
                player);
        undo->commitAction(false);
    }
    else if (mListener)
        mListener->sceneEdited();
    ++mWrites;
    update();
}

void PropertyInspector::removeAnimation(Node* target, const ct::String& name)
{
    AnimationPlayer* player = target ? target->castTo<AnimationPlayer>() : nullptr;
    if (!player || !target || target != mNode) return;
    const AnimationClip* clip = player->getAnimation(name);
    if (!clip) return;
    const ct::Rc<AnimationClip> copy = ct::make_rc<AnimationClip>(*clip);
    player->removeAnimation(name);
    EditorUndoRedoManager* undo = mListener ? mListener->undoRedoManager() : nullptr;
    if (undo)
    {
        undo->createAction(ct::String("Remove Animation"));
        undo->addDoMethod([player, name] { player->removeAnimation(name); }, player);
        undo->addUndoMethod([player, name, copy] { player->addAnimationCopy(name, *copy); },
                player);
        undo->commitAction(false);
    }
    else if (mListener)
        mListener->sceneEdited();
    ++mWrites;
    update();
}

void PropertyInspector::playAnimation(Node* target, const ct::String& name)
{
    AnimationPlayer* player = target ? target->castTo<AnimationPlayer>() : nullptr;
    if (!player || target != mNode || !player->hasAnimation(name)) return;
    mRestHeld = false;
    player->setActive(true);
    player->play(name);
    update();
}

void PropertyInspector::stopAnimation(Node* target)
{
    AnimationPlayer* player = target ? target->castTo<AnimationPlayer>() : nullptr;
    if (!player || target != mNode) return;
    mRestHeld = true;
    player->stop(false);
    update();
}

void PropertyInspector::seekAnimation(Node* target, const ct::String& name, double time)
{
    AnimationPlayer* player = target ? target->castTo<AnimationPlayer>() : nullptr;
    if (!player || target != mNode || !player->hasAnimation(name)) return;
    mRestHeld = false;
    applySeek(player, name, time);
    update();
}

void PropertyInspector::applySeek(AnimationPlayer* player, const ct::String& name, double time)
{
    player->setActive(true);
    if (player->getCurrentAnimation() != name) player->play(name);
    player->pause();
    player->seek(time, true);
}

void PropertyInspector::pauseAnimation(Node* target)
{
    AnimationPlayer* player = target ? target->castTo<AnimationPlayer>() : nullptr;
    if (!player || target != mNode) return;
    player->pause();
    update();
}

void PropertyInspector::poseIfIdle(AnimationPlayer* player)
{
    if (mRestHeld || player->getAnimationCount() == 0 || !player->getCurrentAnimation().empty())
        return;
    const Engine* engine = Engine::singleton();
    if (!engine || !engine->isEditorHint()) return;
    ct::String name = player->getAutoplay();
    if (name.empty() || !player->hasAnimation(name)) name = player->getAnimationName(0);
    applySeek(player, name, 0.0);
}

void PropertyInspector::syncRow(std::size_t index)
{
    syncMeshStats(index);
    syncAnimations(index);
    Entry& entry = mEntries[index];
    PropertyValue current;
    if (!mNode || !NodeClassDB::getProperty(mNode, entry.name, current)) return;
    entry.row->setRevertible(entry.hasDefault && different(current, entry.defaultValue));
}

void PropertyInspector::update()
{
    if (!mNode) return;
    refreshList();
    syncGates();
    for (std::size_t i = 0; i < mEntries.size(); ++i)
    {
        Entry& entry = mEntries[i];
        PropertyValue current;
        if (!NodeClassDB::getProperty(mNode, entry.name, current)) continue;
        if (!entry.row->isActive() && different(current, entry.row->value()))
            entry.row->setValue(current);
        entry.row->setRevertible(entry.hasDefault && different(current, entry.defaultValue));
    }
}

bool PropertyInspector::writeProperty(const ct::String& name, const PropertyValue& value)
{
    if (!mNode) return false;
    EditorUndoRedoManager* undo = mListener ? mListener->undoRedoManager() : nullptr;
    PropertyValue before;
    if (undo && NodeClassDB::getProperty(mNode, name, before))
    {
        EditorTreeEdits::setProperty(undo, ct::String("Set ") + name, mNode, name, before, value);
        ++mWrites;
        return true;
    }
    const bool written = NodeClassDB::setProperty(mNode, name, value);
    if (!written) return false;
    ++mWrites;
    if (mListener) mListener->sceneEdited();
    return true;
}

bool PropertyInspector::write(Entry& entry, const PropertyValue& value)
{
    return writeProperty(entry.name, value);
}

void PropertyInspector::rowEdited(std::size_t index, const PropertyValue& value)
{
    if (index >= mEntries.size()) return;
    Entry& entry = mEntries[index];
    write(entry, value);
    PropertyValue current;
    if (mNode && NodeClassDB::getProperty(mNode, entry.name, current) && !entry.row->isActive() &&
            different(current, entry.row->value()))
        entry.row->setValue(current);
    syncRow(index);
}

void PropertyInspector::rowReverted(std::size_t index)
{
    if (index >= mEntries.size()) return;
    Entry& entry = mEntries[index];
    if (!entry.hasDefault) return;
    write(entry, entry.defaultValue);
    PropertyValue current;
    if (mNode && NodeClassDB::getProperty(mNode, entry.name, current)) entry.row->setValue(current);
    syncRow(index);
}

void PropertyInspector::rowBrowse(std::size_t index)
{
    if (index >= mEntries.size() || !mNode) return;
    openFileDialog(index, PropertyRow::fileFilter(mEntries[index].row->info()));
}

void PropertyInspector::openFileDialog(std::size_t index, const ct::String& filter)
{
    const Entry& entry = mEntries[index];

    rt::WidgetApp& app = rt::WidgetApp::instance();
    rt::FileDialog* dialog = app.addFloat<rt::FileDialog>("Choose a file");
    dialog->setMode(rt::FileDialog::Mode::Open);
    if (!filter.empty()) dialog->setFilter(filter);
    FileDialogMemory::prepare(*dialog, mSettings, EditorSettings::kFolderFile, mProject);

    const ct::Rc<Link> link = mLink;
    Node* target = mNode;
    const ct::String name = entry.name;
    EditorSettings* settings = mSettings;
    dialog->accepted.connect(
            [link, target, name, settings](const ct::String& path)
            {
                FileDialogMemory::remember(settings, EditorSettings::kFolderFile, path);
                if (link->owner) link->owner->fileChosen(target, name, path);
            });
}

void PropertyInspector::fileChosen(Node* target, const ct::String& name, const ct::String& path)
{
    if (!target || target != mNode) return;
    PropertyRow* row = rowNamed(name);
    if (!row) return;
    const std::size_t index = indexOf(row);

    ct::String stored = path;
    if (mProject && mProject->isOpen())
    {
        if (!mProject->relativePath(path, stored))
        {
            rt::Toast::show("Choose a file inside the project folder", rt::Toast::Type::Warning);
            return;
        }
    }
    write(mEntries[index], PropertyValue::fromResourcePath(stored));
    PropertyValue current;
    if (NodeClassDB::getProperty(mNode, name, current)) row->setValue(current);
    syncRow(index);
}

void PropertyInspector::rowPickMaterial(std::size_t index)
{
    if (index >= mEntries.size() || !mNode) return;
    PropertyRow* row = mEntries[index].row;
    if (!row->materialButton()) return;

    ct::Vector<MaterialChoice> choices;
    if (mProject && mProject->isOpen()) collectMaterials(*mProject, choices);

    if (!mPickerMenu) mPickerMenu = new rt::Menu();
    mPickerMenu->clearActions();

    const ct::Rc<Link> link = mLink;
    Node* target = mNode;
    const ct::String name = mEntries[index].name;
    const auto choose = [link, target, name](const ct::String& path)
    {
        if (link->owner) link->owner->materialChosen(target, name, path);
    };

    ct::String library;
    for (std::size_t i = 0; i < choices.size(); ++i)
    {
        if (choices[i].library != library)
        {
            if (!library.empty()) mPickerMenu->addSeparator();
            library = choices[i].library;
        }
        const ct::String path = choices[i].path;
        mPickerMenu->addAction(path, [choose, path] { choose(path); });
    }
    if (choices.empty())
    {
        rt::MenuAction* none = mPickerMenu->addAction(kNoMaterials);
        none->setEnabled(false);
    }
    mPickerMenu->addSeparator();
    mPickerMenu->addAction(kClearMaterial, [choose] { choose(ct::String()); });

    const rt::Rect rect = row->materialButton()->absoluteRect();
    mPickerMenu->exec(rect.x, rect.y + rect.h, row->materialButton());
}

void PropertyInspector::rowPickNode(std::size_t index)
{
    if (index >= mEntries.size() || !mNode) return;
    PropertyRow* row = mEntries[index].row;
    if (!row->nodeButton()) return;

    ct::Vector<ct::String> types;
    const PropertyInfo& info = row->info();
    if (info.hint == PropertyHint::NodePathValidTypes)
    {
        std::size_t start = 0;
        for (std::size_t i = 0; i <= info.hintString.size(); ++i)
        {
            if (i != info.hintString.size() && info.hintString[i] != ',') continue;
            const ct::String type = info.hintString.substr(start, i - start).trimmed();
            if (!type.empty()) types.push_back(type);
            start = i + 1;
        }
    }

    const Node* scene = mNode;
    while (scene->getParent() && scene->getParent()->getParent()) scene = scene->getParent();
    ct::Vector<ct::String> paths;
    collectNodes(mNode, scene, types, paths);

    if (!mPickerMenu) mPickerMenu = new rt::Menu();
    mPickerMenu->clearActions();

    const ct::Rc<Link> link = mLink;
    Node* target = mNode;
    const ct::String name = mEntries[index].name;
    const auto choose = [link, target, name](const ct::String& path)
    {
        if (link->owner) link->owner->nodeChosen(target, name, path);
    };

    for (std::size_t i = 0; i < paths.size(); ++i)
    {
        const ct::String path = paths[i];
        mPickerMenu->addAction(path, [choose, path] { choose(path); });
    }
    if (paths.empty())
    {
        rt::MenuAction* none = mPickerMenu->addAction(kNoNodes);
        none->setEnabled(false);
    }
    mPickerMenu->addSeparator();
    mPickerMenu->addAction(kClearNode, [choose] { choose(ct::String()); });

    const rt::Rect rect = row->nodeButton()->absoluteRect();
    mPickerMenu->exec(rect.x, rect.y + rect.h, row->nodeButton());
}

void PropertyInspector::nodeChosen(Node* target, const ct::String& name, const ct::String& path)
{
    if (!target || target != mNode) return;
    PropertyRow* row = rowNamed(name);
    if (!row) return;
    const std::size_t index = indexOf(row);
    write(mEntries[index], PropertyValue::fromNodePath(path));
    PropertyValue current;
    if (NodeClassDB::getProperty(mNode, name, current)) row->setValue(current);
    syncRow(index);
}

void PropertyInspector::materialChosen(Node* target, const ct::String& name, const ct::String& path)
{
    if (!target || target != mNode) return;
    PropertyRow* row = rowNamed(name);
    if (!row) return;
    const std::size_t index = indexOf(row);
    write(mEntries[index], PropertyValue::fromResourcePath(path));
    PropertyValue current;
    if (NodeClassDB::getProperty(mNode, name, current)) row->setValue(current);
    syncRow(index);
}

Node* PropertyInspector::node() const
{
    return mNode;
}

std::size_t PropertyInspector::rowCount() const
{
    return mEntries.size();
}

PropertyRow* PropertyInspector::row(std::size_t index) const
{
    return index < mEntries.size() ? mEntries[index].row : nullptr;
}

PropertyRow* PropertyInspector::rowNamed(const ct::String& name) const
{
    for (std::size_t i = 0; i < mEntries.size(); ++i)
        if (mEntries[i].name == name) return mEntries[i].row;
    return nullptr;
}

ct::String PropertyInspector::rowName(std::size_t index) const
{
    return index < mEntries.size() ? mEntries[index].name : ct::String();
}

std::size_t PropertyInspector::sectionCount() const
{
    return mSectionPaths.size();
}

rt::Collapsible* PropertyInspector::section(const ct::String& path) const
{
    rt::Widget* const* found = mSections.find(path);
    return found ? static_cast<rt::Collapsible*>(*found) : nullptr;
}

unsigned int PropertyInspector::writeCount() const
{
    return mWrites;
}

unsigned int PropertyInspector::rebuildCount() const
{
    return mRebuilds;
}

} // namespace prisma3d
