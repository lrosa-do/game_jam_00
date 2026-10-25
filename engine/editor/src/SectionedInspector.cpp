#include "prisma3d/editor/SectionedInspector.h"

#include "prisma3d/editor/PropertyNames.h"

#include <igui/widgets/WidgetApp.hpp>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

char lowered(char c)
{
    return c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c;
}

bool containsIgnoringCase(const ct::String& text, const ct::String& needle)
{
    if (needle.empty()) return true;
    if (needle.size() > text.size()) return false;
    for (std::size_t i = 0; i + needle.size() <= text.size(); ++i)
    {
        bool same = true;
        for (std::size_t j = 0; j < needle.size() && same; ++j)
            same = lowered(text[i + j]) == lowered(needle[j]);
        if (same) return true;
    }
    return false;
}

bool isSubsequenceIgnoringCase(const ct::String& needle, const ct::String& haystack)
{
    if (needle.empty()) return true;
    if (needle.size() > haystack.size()) return false;
    std::size_t next = 0;
    for (std::size_t i = 0; i < haystack.size() && next < needle.size(); ++i)
        if (lowered(haystack[i]) == lowered(needle[next])) ++next;
    return next == needle.size();
}

ct::Vector<ct::String> splitPath(const ct::String& path)
{
    ct::Vector<ct::String> parts;
    std::size_t start = 0;
    for (std::size_t i = 0; i <= path.size(); ++i)
    {
        if (i != path.size() && path[i] != '/') continue;
        parts.push_back(path.substr(start, i - start));
        start = i + 1;
    }
    return parts;
}

ct::String withGlobal(const ct::String& name)
{
    if (name.contains('/')) return name;
    return ct::String(SectionedInspector::kGlobalSection) + "/" + name;
}

bool skipped(const PropertyInfo& info)
{
    if (info.isGroup() || !(info.usage & PropertyUsage::Editor)) return true;
    return info.name.contains(':') || info.name == "script";
}

} // namespace

SectionedInspector::SectionedInspector()
        : rt::Splitter(rt::LayoutDir::Horizontal),
          mSections(nullptr),
          mScroll(nullptr),
          mRows(nullptr),
          mSelecting(false)
{
    setRatio(kSectionsRatio);
    mSections = createChild<ActivatableTree>();
    mScroll = createChild<rt::ScrollView>();
    mRows = mScroll->setContent<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mRows->setPadding(kPadding);
    mRows->setSpacing(kRowSpacing);
    mSections->selectionChanged.connect([this](rt::TreeNode* node) { sectionSelected(node); });
}

void SectionedInspector::setSource(const Source& source)
{
    mSource = source;
}

void SectionedInspector::setProperties(const ct::Vector<PropertyInfo>& properties)
{
    mProperties = properties;
    mSelected = ct::String();
    updateCategoryList();
}

void SectionedInspector::setFilter(const ct::String& filter)
{
    if (filter == mFilter) return;
    mFilter = filter;
    updateCategoryList();
}

const ct::String& SectionedInspector::filter() const
{
    return mFilter;
}

bool SectionedInspector::pathMatches(const ct::String& path, const ct::String& filter)
{
    if (containsIgnoringCase(path, filter)) return true;
    const ct::Vector<ct::String> segments = splitPath(path);
    for (std::size_t i = 0; i < segments.size(); ++i)
        if (isSubsequenceIgnoringCase(filter, capitalizePropertyName(segments[i]))) return true;
    return false;
}

ct::String SectionedInspector::sectionTitle(const ct::String& segment)
{
    return capitalizePropertyName(segment);
}

void SectionedInspector::updateCategoryList()
{
    const ct::String previous = mSelected;
    mSelecting = true;
    releaseEditors();
    mSections->clearRows();
    mItems.clear();
    mPaths.clear();
    mSelectable.clear();
    mSectionPaths.clear();

    for (std::size_t p = 0; p < mProperties.size(); ++p)
    {
        const PropertyInfo& info = mProperties[p];
        if (skipped(info)) continue;
        const ct::String name = withGlobal(info.name);
        if (!mFilter.empty() && !pathMatches(name, mFilter)) continue;

        const ct::Vector<ct::String> parts = splitPath(name);
        const std::size_t levels = parts.size() - 1 < static_cast<std::size_t>(kMaxSectionLevels)
                                           ? parts.size() - 1
                                           : static_cast<std::size_t>(kMaxSectionLevels);
        ct::String parentPath;
        for (std::size_t i = 0; i < levels; ++i)
        {
            const ct::String path = i == 0 ? parts[0] : parentPath + "/" + parts[i];
            rt::TreeNode* parent = nullptr;
            if (i > 0)
            {
                rt::TreeNode** found = mItems.find(parentPath);
                parent = found ? *found : nullptr;
            }

            if (!mItems.contains(path))
            {
                rt::TreeNode* item = parent ? parent->addChild(sectionTitle(parts[i]))
                                            : mSections->addRoot(sectionTitle(parts[i]));
                mItems.put(path, item);
                mPaths.put(item, path);
                mSelectable.put(path, false);
                mSectionPaths.push_back(path);
            }
            if (i + 1 == levels) mSelectable.put(path, true);
            parentPath = path;
        }
    }

    mSelected = ct::String();
    rt::TreeNode* target = nullptr;
    if (!previous.empty() && sectionSelectable(previous)) target = sectionItem(previous);
    if (!target)
        for (rt::TreeNode* root: mSections->roots())
            if (!target) target = firstSelectable(root);
    if (target)
    {
        if (const ct::String* path = mPaths.find(target)) mSelected = *path;
        mSections->setSelectedNode(target);
    }
    mSelecting = false;

    rebuildRows();
    if (mSelected != previous) categoryChanged.emit(mSelected);
}

rt::TreeNode* SectionedInspector::firstSelectable(rt::TreeNode* node) const
{
    const ct::String* path = mPaths.find(node);
    if (path && sectionSelectable(*path)) return node;
    for (rt::TreeNode* child: node->children())
        if (rt::TreeNode* found = firstSelectable(child)) return found;
    return nullptr;
}

bool SectionedInspector::hasChildren(const ct::String& path) const
{
    rt::TreeNode* const* item = mItems.find(path);
    return item && (*item)->hasChildren();
}

void SectionedInspector::sectionSelected(rt::TreeNode* node)
{
    if (mSelecting || !node) return;
    const ct::String* path = mPaths.find(node);
    if (!path) return;
    if (!sectionSelectable(*path))
    {
        mSelecting = true;
        mSections->setSelectedNode(sectionItem(mSelected));
        mSelecting = false;
        return;
    }
    if (*path == mSelected) return;
    mSelected = *path;
    rebuildRows();
    categoryChanged.emit(mSelected);
}

void SectionedInspector::setCurrentSection(const ct::String& section)
{
    if (!sectionSelectable(section)) return;
    mSections->setSelectedNode(sectionItem(section));
}

const ct::String& SectionedInspector::currentSection() const
{
    return mSelected;
}

ct::String SectionedInspector::fullItemPath(const ct::String& item) const
{
    return mSelected.empty() ? item : mSelected + "/" + item;
}

void SectionedInspector::releaseEditors()
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

void SectionedInspector::clearRows()
{
    const ct::Vector<rt::Widget*> children = mRows->children();
    for (std::size_t i = 0; i < children.size(); ++i) mRows->removeChild(children[i]);
    mEntries.clear();
    mGroups.clear();
}

void SectionedInspector::rebuildRows()
{
    releaseEditors();
    clearRows();
    mScroll->setScrollY(0.0f);
    if (!mSelected.empty())
    {
        const bool leaf = !hasChildren(mSelected);
        const ct::String prefix = mSelected + "/";
        for (std::size_t p = 0; p < mProperties.size(); ++p)
        {
            const PropertyInfo& info = mProperties[p];
            if (skipped(info)) continue;
            const ct::String name = withGlobal(info.name);
            if (!name.starts_with(prefix)) continue;
            const ct::String relative = name.substr(prefix.size());
            if (!leaf && relative.contains('/')) continue;
            if (!mFilter.empty() && !pathMatches(name, mFilter)) continue;
            addRow(info, name, relative);
        }
    }
    if (rt::WidgetApp* app = rt::WidgetApp::existing()) app->requestLayout();
}

rt::Widget* SectionedInspector::container(const ct::Vector<ct::String>& components,
        std::size_t count)
{
    rt::Widget* parent = mRows;
    ct::String path;
    for (std::size_t i = 0; i < count; ++i)
    {
        if (i > 0) path += '/';
        path += components[i];
        if (rt::Widget** found = mGroups.find(path))
        {
            parent = *found;
            continue;
        }
        const ct::String key = mSelected + "|" + path;
        bool expanded = true;
        if (const bool* remembered = mExpanded.find(key)) expanded = *remembered;
        rt::Collapsible* section =
                parent->createChild<rt::Collapsible>(sectionTitle(components[i]), expanded);
        section->setContentPadding(kRowSpacing);
        section->setContentSpacing(kRowSpacing);
        section->expandedChanged.connect([this, key](bool value) { mExpanded.put(key, value); });
        mGroups.put(path, section);
        parent = section;
    }
    return parent;
}

void SectionedInspector::addRow(const PropertyInfo& info, const ct::String& name,
        const ct::String& relative)
{
    ct::Vector<ct::String> components = splitPath(relative);
    const ct::String last = components.back();
    components.pop_back();
    rt::Widget* parent = components.empty() ? mRows : container(components, components.size());

    PropertyInfo shown = info;
    shown.name = name;
    PropertyRow* row = parent->createChild<PropertyRow>(shown, sectionTitle(last));
    if (row->kind() == PropertyRow::Kind::None)
    {
        parent->removeChild(row);
        return;
    }

    const std::size_t index = mEntries.size();
    Entry entry;
    entry.name = name;
    entry.row = row;
    mEntries.push_back(entry);

    row->edited.connect([this, index](const PropertyValue& value) { rowEdited(index, value); });
    row->revertRequested.connect([this, index] { rowReverted(index); });
    syncRow(index, true);
}

void SectionedInspector::syncRow(std::size_t index, bool force)
{
    if (!mSource.get) return;
    Entry& entry = mEntries[index];
    if (force || !entry.row->isActive())
    {
        const PropertyValue current = mSource.get(entry.name);
        if (force || current != entry.row->value()) entry.row->setValue(current);
    }
    entry.row->setRevertible(mSource.isDefault && !mSource.isDefault(entry.name));
}

void SectionedInspector::update()
{
    for (std::size_t i = 0; i < mEntries.size(); ++i) syncRow(i, false);
}

bool SectionedInspector::anyActive() const
{
    for (std::size_t i = 0; i < mEntries.size(); ++i)
        if (mEntries[i].row->isActive()) return true;
    return false;
}

void SectionedInspector::rowEdited(std::size_t index, const PropertyValue& value)
{
    if (index >= mEntries.size() || !mSource.set) return;
    const ct::String name = mEntries[index].name;
    const bool written = mSource.set(name, value);
    syncRow(index, !written);
    if (written) propertyEdited.emit(name);
}

void SectionedInspector::rowReverted(std::size_t index)
{
    if (index >= mEntries.size() || !mSource.revert) return;
    const ct::String name = mEntries[index].name;
    mSource.revert(name);
    syncRow(index, true);
    propertyEdited.emit(name);
}

ActivatableTree* SectionedInspector::sections() const
{
    return mSections;
}

rt::ScrollView* SectionedInspector::scrollView() const
{
    return mScroll;
}

rt::TreeNode* SectionedInspector::sectionItem(const ct::String& path) const
{
    rt::TreeNode* const* item = mItems.find(path);
    return item ? *item : nullptr;
}

bool SectionedInspector::sectionSelectable(const ct::String& path) const
{
    const bool* selectable = mSelectable.find(path);
    return selectable && *selectable;
}

const ct::Vector<ct::String>& SectionedInspector::sectionPaths() const
{
    return mSectionPaths;
}

std::size_t SectionedInspector::rowCount() const
{
    return mEntries.size();
}

PropertyRow* SectionedInspector::row(std::size_t index) const
{
    return index < mEntries.size() ? mEntries[index].row : nullptr;
}

PropertyRow* SectionedInspector::rowNamed(const ct::String& fullName) const
{
    for (std::size_t i = 0; i < mEntries.size(); ++i)
        if (mEntries[i].name == fullName) return mEntries[i].row;
    return nullptr;
}

ct::String SectionedInspector::rowName(std::size_t index) const
{
    return index < mEntries.size() ? mEntries[index].name : ct::String();
}

rt::Collapsible* SectionedInspector::group(const ct::String& path) const
{
    rt::Widget* const* found = mGroups.find(path);
    return found ? rt::widget_cast<rt::Collapsible>(*found) : nullptr;
}

std::size_t SectionedInspector::groupCount() const
{
    return mGroups.size();
}

} // namespace prisma3d
