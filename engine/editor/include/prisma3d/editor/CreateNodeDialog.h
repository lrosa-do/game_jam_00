#pragma once

#include "prisma3d/editor/ActivatableTree.h"
#include "prisma3d/editor/ClassIcons.h"
#include "prisma3d/editor/EditorSettings.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/Signal.hpp>
#include <igui/widgets/TextInputWidgets.hpp>
#include <igui/widgets/ViewWidgets.hpp>

#include <ct/function.hpp>
#include <ct/hashmap.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class CreateSearchInput : public ig::retained::TextInput
{
public:
    IG_WIDGET_CLASS(CreateSearchInput, ig::retained::TextInput);

    ig::retained::Signal<> spacePressed;

    void onKeyPress(ig::retained::KeyEvent& event) override;
    void onTextInput(ig::retained::KeyEvent& event) override;
};

class FavoriteButton : public ig::retained::Widget
{
public:
    IG_WIDGET_CLASS(FavoriteButton, ig::retained::Widget);

    static constexpr int kStarPoints = 5;
    static constexpr int kStarVertices = kStarPoints * 2;
    static constexpr float kStarInnerRatio = 0.4f;
    static constexpr float kStarMargin = 6.0f;
    static constexpr float kStarDrop = 0.0954915f;

    FavoriteButton();

    bool isChecked() const;
    void setChecked(bool checked);

    Vec2f starCenter() const;
    float starRadius() const;
    void starVertices(Vec2f out[kStarVertices]) const;

    Vec2f sizeHint() const override;
    void paint(ig::retained::PaintContext& ctx) override;

private:
    bool mChecked;
};

class CreateNodeDialog : public ig::retained::FloatWindow
{
public:
    IG_WIDGET_CLASS(CreateNodeDialog, ig::retained::FloatWindow);

    static constexpr const char* kTitle = "Create New Node";
    static constexpr const char* kBaseType = "Node";
    static constexpr const char* kSearchPlaceholder = "Search";
    static constexpr const char* kFavoriteTooltip = "(Un)favorite selected item.";
    static constexpr const char* kNotInstantiableTooltip =
            "The selected class can't be instantiated.";
    static constexpr int kRecentCompletionSize = 5;

    using PersistHandler = ct::Function<void()>;

    CreateNodeDialog(EditorSettings& settings, const ClassIcons& icons,
            const ct::String& preferredType, const ct::String& search);

    static bool shouldHideType(const ct::String& type);
    static const char* hiddenReason(const ct::String& type);
    static void typeList(ct::Vector<ct::String>& types);
    static ct::Vector<ct::String> hiddenTypes();

    void setPersistHandler(const PersistHandler& handler);

    float scoreType(const ct::String& type, const ct::String& search) const;
    bool isTypePreferred(const ct::String& type) const;
    const ct::String& preferredType() const;

    ct::String searchText() const;
    void setSearchText(const ct::String& text);

    bool selectType(const ct::String& type, bool center = true);
    ct::String selectedType() const;
    bool canCreate() const;
    bool isInstantiable(const ct::String& type) const;
    bool hasType(const ct::String& type) const;
    ig::retained::TreeNode* itemOf(const ct::String& type) const;
    const ct::String& noResultsText() const;

    void toggleFavorite();
    void confirm();
    void cancel();

    ActivatableTree* optionsTree() const;
    ActivatableTree* favoritesTree() const;
    ActivatableTree* recentTree() const;
    CreateSearchInput* searchInput() const;
    FavoriteButton* favoriteButton() const;
    ig::retained::Button* createButton() const;
    ig::retained::Button* cancelButton() const;
    ig::retained::Label* statusLabel() const;

    ig::retained::Signal<const ct::String&> created;
    ig::retained::Signal<const ct::String&> closedWith;

    void layout() override;
    void onKeyPress(ig::retained::KeyEvent& event) override;
    ig::retained::Widget* focusOnOpen() override;

private:
    void fillTypeList();
    void updateSearch();
    void addType(const ct::String& type, const ct::String& search);
    void configureItem(ig::retained::TreeNode* item, const ct::String& type,
            const ct::String& search);
    void refreshFavorites();
    void refreshRecent();
    void loadFavoritesAndHistory();
    void scrollToSelected(ig::retained::TreeNode* item, bool center);
    void persist();
    void textChanged();
    void itemSelected(ig::retained::TreeNode* item);
    void listPressed(ig::retained::TreeNode* item);
    void applySearch(const ct::String& text);
    void navigate(int steps);
    void toggleCollapse();
    void close();

    EditorSettings& mSettings;
    const ClassIcons& mIcons;
    ct::String mPreferred;
    ct::Vector<ct::String> mTypes;
    ct::HashMap<ct::String, ig::retained::TreeNode*> mItemOf;
    ct::HashMap<ig::retained::TreeNode*, ct::String> mTypeOf;
    PersistHandler mPersist;
    ct::String mNoResults;
    CreateSearchInput* mSearch;
    ActivatableTree* mOptions;
    ActivatableTree* mFavorites;
    ActivatableTree* mRecent;
    FavoriteButton* mFavorite;
    ig::retained::Label* mStatus;
    ig::retained::Button* mCreate;
    ig::retained::Button* mCancel;
    ig::retained::TreeNode* mPendingNode;
    bool mPendingCenter;
    bool mSilent;
    bool mSelecting;
    bool mClosed;
};

} // namespace prisma3d
