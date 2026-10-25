#pragma once

#include "prisma3d/editor/ActivatableTree.h"
#include "prisma3d/editor/PropertyRow.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PropertyValue.h"

#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/ScrollWidgets.hpp>
#include <igui/widgets/Signal.hpp>
#include <igui/widgets/TreePropertyColorWidgets.hpp>

#include <ct/function.hpp>
#include <ct/hashmap.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>

namespace prisma3d
{

class SectionedInspector : public ig::retained::Splitter
{
public:
    IG_WIDGET_CLASS(SectionedInspector, ig::retained::Splitter);

    struct Source
    {
        ct::Function<PropertyValue(const ct::String&)> get;
        ct::Function<bool(const ct::String&, const PropertyValue&)> set;
        ct::Function<bool(const ct::String&)> isDefault;
        ct::Function<void(const ct::String&)> revert;
    };

    static constexpr float kSectionsRatio = 0.16f;
    static constexpr float kPadding = 8.0f;
    static constexpr float kRowSpacing = 4.0f;
    static constexpr int kMaxSectionLevels = 2;
    static constexpr const char* kGlobalSection = "global";

    SectionedInspector();

    void setSource(const Source& source);
    void setProperties(const ct::Vector<PropertyInfo>& properties);
    void setFilter(const ct::String& filter);
    const ct::String& filter() const;
    void updateCategoryList();
    void update();
    bool anyActive() const;

    void setCurrentSection(const ct::String& section);
    const ct::String& currentSection() const;
    ct::String fullItemPath(const ct::String& item) const;

    static bool pathMatches(const ct::String& path, const ct::String& filter);
    static ct::String sectionTitle(const ct::String& segment);

    ActivatableTree* sections() const;
    ig::retained::ScrollView* scrollView() const;
    ig::retained::TreeNode* sectionItem(const ct::String& path) const;
    bool sectionSelectable(const ct::String& path) const;
    const ct::Vector<ct::String>& sectionPaths() const;

    std::size_t rowCount() const;
    PropertyRow* row(std::size_t index) const;
    PropertyRow* rowNamed(const ct::String& fullName) const;
    ct::String rowName(std::size_t index) const;
    ig::retained::Collapsible* group(const ct::String& path) const;
    std::size_t groupCount() const;

    ig::retained::Signal<const ct::String&> categoryChanged;
    ig::retained::Signal<const ct::String&> propertyEdited;

private:
    struct Entry
    {
        ct::String name;
        PropertyRow* row = nullptr;
    };

    void rebuildRows();
    void clearRows();
    void releaseEditors();
    ig::retained::Widget* container(const ct::Vector<ct::String>& components, std::size_t count);
    void addRow(const PropertyInfo& info, const ct::String& name, const ct::String& relative);
    void sectionSelected(ig::retained::TreeNode* node);
    void rowEdited(std::size_t index, const PropertyValue& value);
    void rowReverted(std::size_t index);
    void syncRow(std::size_t index, bool force);
    bool hasChildren(const ct::String& path) const;
    ig::retained::TreeNode* firstSelectable(ig::retained::TreeNode* node) const;

    Source mSource;
    ct::Vector<PropertyInfo> mProperties;
    ct::String mFilter;
    ct::String mSelected;
    ActivatableTree* mSections;
    ig::retained::ScrollView* mScroll;
    ig::retained::BoxLayout* mRows;
    ct::HashMap<ct::String, ig::retained::TreeNode*> mItems;
    ct::HashMap<ig::retained::TreeNode*, ct::String> mPaths;
    ct::HashMap<ct::String, bool> mSelectable;
    ct::Vector<ct::String> mSectionPaths;
    ct::HashMap<ct::String, ig::retained::Widget*> mGroups;
    ct::HashMap<ct::String, bool> mExpanded;
    ct::Vector<Entry> mEntries;
    bool mSelecting;
};

} // namespace prisma3d
