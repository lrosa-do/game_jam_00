#include "prisma3d/editor/CreateNodeDialog.h"

#include "prisma3d/scene/NodeClassDB.h"

#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Retained.hpp>
#include <igui/widgets/Theme.hpp>
#include <igui/widgets/WidgetApp.hpp>

#include <ct/sort.hpp>

#include <math.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

struct Blocked
{
    const char* type;
    const char* reason;
};

const Blocked kBlocked[] = {
    { "MissingNode", "stands in for a class that could not be loaded" },
    { "Window", "a second screen-attached viewport would render over the editor window" },
    { "Viewport", "its children leave the edited scenario and nothing shows its output" },
};

char lowered(char c)
{
    return c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c;
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

int findIgnoringCase(const ct::String& text, const ct::String& needle)
{
    if (needle.empty() || needle.size() > text.size()) return -1;
    for (std::size_t i = 0; i + needle.size() <= text.size(); ++i)
    {
        bool same = true;
        for (std::size_t j = 0; j < needle.size() && same; ++j)
            same = lowered(text[i + j]) == lowered(needle[j]);
        if (same) return static_cast<int>(i);
    }
    return -1;
}

bool recognized(const ct::String& type)
{
    return NodeClassDB::classExists(type) &&
           NodeClassDB::isParentClass(type, CreateNodeDialog::kBaseType);
}

void prune(ct::Vector<ct::String>& list)
{
    std::size_t kept = 0;
    for (std::size_t i = 0; i < list.size(); ++i)
        if (recognized(list[i])) list[kept++] = list[i];
    list.resize(kept);
}

void fillList(ActivatableTree* tree, const ct::Vector<ct::String>& names, const ClassIcons& icons)
{
    tree->clearRows();
    for (std::size_t i = 0; i < names.size(); ++i)
    {
        rt::TreeNode* item = tree->addRoot(names[i]);
        item->setIconId(icons.icon(names[i]));
    }
}

} // namespace

void CreateSearchInput::onKeyPress(rt::KeyEvent& event)
{
    if (event.key == rt::Key::Space && !event.ctrl && !event.alt)
    {
        event.consumed = true;
        spacePressed.emit();
        return;
    }
    rt::TextInput::onKeyPress(event);
}

void CreateSearchInput::onTextInput(rt::KeyEvent& event)
{
    std::size_t kept = 0;
    for (std::size_t i = 0; event.text[i]; ++i)
        if (event.text[i] != ' ') event.text[kept++] = event.text[i];
    event.text[kept] = 0;
    if (kept == 0)
    {
        event.consumed = true;
        return;
    }
    rt::TextInput::onTextInput(event);
}

FavoriteButton::FavoriteButton()
        : mChecked(false)
{
    cursor_ = rt::CursorType::Hand;
}

bool FavoriteButton::isChecked() const
{
    return mChecked;
}

void FavoriteButton::setChecked(bool checked)
{
    if (mChecked == checked) return;
    mChecked = checked;
    markDirty();
}

rt::Widget::Vec2f FavoriteButton::sizeHint() const
{
    const float height = rt::Theme::instance().inputHeight;
    return { height, height };
}

rt::Widget::Vec2f FavoriteButton::starCenter() const
{
    const rt::Rect abs = absoluteRect();
    return { abs.x + abs.w * 0.5f, abs.y + abs.h * 0.5f + starRadius() * kStarDrop };
}

float FavoriteButton::starRadius() const
{
    const rt::Rect abs = absoluteRect();
    const float side = abs.w < abs.h ? abs.w : abs.h;
    const float radius = side * 0.5f - kStarMargin;
    return radius > 1.0f ? radius : 1.0f;
}

void FavoriteButton::starVertices(Vec2f out[kStarVertices]) const
{
    const Vec2f center = starCenter();
    const float outer = starRadius();
    const float inner = outer * kStarInnerRatio;
    const float step = 3.14159265358979f / static_cast<float>(kStarPoints);
    for (int i = 0; i < kStarVertices; ++i)
    {
        const float angle = -1.57079632679490f + step * static_cast<float>(i);
        const float radius = (i % 2 == 0) ? outer : inner;
        out[i] = { center.x + cosf(angle) * radius, center.y + sinf(angle) * radius };
    }
}

void FavoriteButton::paint(rt::PaintContext& ctx)
{
    if (!visible_) return;
    const rt::Rect abs = absoluteRect();
    if (ctx.isClipped(abs)) return;

    const rt::Theme& theme = rt::Theme::instance();
    ctx.fill.SetColor(isHovered() && enabled_ ? theme.inputBgHover : theme.inputBg);
    ctx.fillRoundedRect(abs.x, abs.y, abs.w, abs.h, theme.borderRadius, 6);
    ctx.line.SetColor(isHovered() && enabled_ ? theme.inputBorderHover : theme.inputBorder);
    ctx.lineRoundedRect(abs.x, abs.y, abs.w, abs.h, theme.borderRadius, 6);

    Vec2f points[kStarVertices];
    starVertices(points);
    const rt::Color color = !enabled_ ? theme.textDisabled
                                      : (mChecked ? rt::Color(255, 205, 64, 255) : theme.textColor);
    const rt::Vec2 center = { starCenter().x, starCenter().y };
    if (mChecked)
    {
        for (int i = 0; i < kStarVertices; ++i)
        {
            const Vec2f a = points[i];
            const Vec2f b = points[(i + 1) % kStarVertices];
            ctx.drawList.addTriangleFilled(center, { a.x, a.y }, { b.x, b.y }, color);
        }
    }
    ctx.drawList.pathClear();
    for (int i = 0; i < kStarVertices; ++i) ctx.drawList.pathLineTo({ points[i].x, points[i].y });
    ctx.drawList.pathStroke(color, 1.5f, true);
}

CreateNodeDialog::CreateNodeDialog(EditorSettings& settings, const ClassIcons& icons,
        const ct::String& preferredType, const ct::String& search)
        : rt::FloatWindow(kTitle),
          mSettings(settings),
          mIcons(icons),
          mPreferred(preferredType),
          mSearch(nullptr),
          mOptions(nullptr),
          mFavorites(nullptr),
          mRecent(nullptr),
          mFavorite(nullptr),
          mStatus(nullptr),
          mCreate(nullptr),
          mCancel(nullptr),
          mPendingNode(nullptr),
          mPendingCenter(true),
          mSilent(false),
          mSelecting(false),
          mClosed(false)
{
    setMinimizable(false);

    rt::BoxLayout* main = setContent<rt::BoxLayout>(rt::LayoutDir::Vertical);
    main->setSpacing(8.0f);
    main->setPadding(10.0f);

    rt::BoxLayout* body = main->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    body->setStretch(1.0f);
    body->setPadding(0.0f);
    body->setSpacing(8.0f);

    rt::BoxLayout* left = body->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    left->setPadding(0.0f);
    left->setSpacing(4.0f);
    left->createChild<rt::Label>("Favorites:");
    mFavorites = left->createChild<ActivatableTree>();
    mFavorites->setStretch(1.0f);
    left->createChild<rt::Label>("Recent:");
    mRecent = left->createChild<ActivatableTree>();
    mRecent->setStretch(1.0f);

    rt::BoxLayout* right = body->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    right->setStretch(1.0f);
    right->setPadding(0.0f);
    right->setSpacing(4.0f);
    right->createChild<rt::Label>("Search:");
    rt::BoxLayout* row = right->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    row->setPadding(0.0f);
    row->setSpacing(4.0f);
    mSearch = row->createChild<CreateSearchInput>();
    mSearch->setStretch(1.0f);
    mSearch->setPlaceholder(kSearchPlaceholder);
    mFavorite = row->createChild<FavoriteButton>();
    mFavorite->setTooltip(kFavoriteTooltip);
    right->createChild<rt::Label>("Matches:");
    mOptions = right->createChild<ActivatableTree>();
    mOptions->setStretch(1.0f);
    mStatus = right->createChild<rt::Label>("");

    rt::BoxLayout* buttons = main->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    buttons->setPadding(0.0f);
    buttons->setSpacing(8.0f);
    buttons->setMainAlign(rt::MainAlign::End);
    mCancel = buttons->createChild<rt::Button>("Cancel");
    mCancel->setSize(90.0f, 0.0f);
    mCreate = buttons->createChild<rt::Button>("Create");
    mCreate->setSize(90.0f, 0.0f);

    mSearch->textChanged.connect([this](const ct::String&) { textChanged(); });
    mSearch->submitted.connect([this](const ct::String&) { confirm(); });
    mSearch->spacePressed.connect([this] { toggleCollapse(); });
    mOptions->selectionChanged.connect([this](rt::TreeNode* item) { itemSelected(item); });
    mOptions->activated.connect([this](rt::TreeNode*) { confirm(); });
    mFavorites->rowPressed.connect([this](rt::TreeNode* item) { listPressed(item); });
    mFavorites->activated.connect([this](rt::TreeNode*) { confirm(); });
    mRecent->rowPressed.connect([this](rt::TreeNode* item) { listPressed(item); });
    mRecent->activated.connect([this](rt::TreeNode*) { confirm(); });
    mFavorite->clicked.connect([this] { toggleFavorite(); });
    mCreate->clicked.connect([this] { confirm(); });
    mCancel->clicked.connect([this] { cancel(); });
    closed.connect([this] { cancel(); });

    const float width = 760.0f;
    const float height = 520.0f;
    setFloatSize(width, height);
    const rt::IO& io = rt::GetIO();
    setFloatPos((io.displayWidth - width) * 0.5f, (io.displayHeight - height) * 0.5f);

    fillTypeList();
    loadFavoritesAndHistory();
    mSilent = true;
    mSearch->setText(search);
    mSearch->selectAll();
    mSilent = false;
    updateSearch();
}

bool CreateNodeDialog::shouldHideType(const ct::String& type)
{
    if (!NodeClassDB::classExists(type)) return true;
    if (!NodeClassDB::canInstantiate(type)) return true;
    if (!NodeClassDB::isParentClass(type, kBaseType)) return true;
    return hiddenReason(type) != nullptr;
}

const char* CreateNodeDialog::hiddenReason(const ct::String& type)
{
    for (const Blocked& entry: kBlocked)
        if (NodeClassDB::classExists(type) && NodeClassDB::isParentClass(type, entry.type))
            return entry.reason;
    return nullptr;
}

void CreateNodeDialog::typeList(ct::Vector<ct::String>& types)
{
    ct::Vector<ct::String> all;
    NodeClassDB::getClassList(all);
    for (std::size_t i = 0; i < all.size(); ++i)
        if (!shouldHideType(all[i])) types.push_back(all[i]);
    ct::sort(types.begin(), types.end(),
            [](const ct::String& a, const ct::String& b) { return a < b; });
}

ct::Vector<ct::String> CreateNodeDialog::hiddenTypes()
{
    ct::Vector<ct::String> all;
    NodeClassDB::getClassList(all);
    ct::Vector<ct::String> hidden;
    for (std::size_t i = 0; i < all.size(); ++i)
        if (NodeClassDB::canInstantiate(all[i]) && hiddenReason(all[i]) != nullptr)
            hidden.push_back(all[i]);
    return hidden;
}

void CreateNodeDialog::fillTypeList()
{
    mTypes.clear();
    typeList(mTypes);
}

void CreateNodeDialog::loadFavoritesAndHistory()
{
    prune(mSettings.createRecent);
    prune(mSettings.createFavorites);
    refreshFavorites();
    refreshRecent();
}

void CreateNodeDialog::refreshFavorites()
{
    fillList(mFavorites, mSettings.createFavorites, mIcons);
}

void CreateNodeDialog::refreshRecent()
{
    fillList(mRecent, mSettings.createRecent, mIcons);
}

void CreateNodeDialog::setPersistHandler(const PersistHandler& handler)
{
    mPersist = handler;
}

void CreateNodeDialog::persist()
{
    if (mPersist) mPersist();
}

bool CreateNodeDialog::isTypePreferred(const ct::String& type) const
{
    if (!NodeClassDB::classExists(type)) return false;
    return NodeClassDB::isParentClass(type, mPreferred);
}

const ct::String& CreateNodeDialog::preferredType() const
{
    return mPreferred;
}

float CreateNodeDialog::scoreType(const ct::String& type, const ct::String& search) const
{
    if (search.empty()) return 0.0f;

    if (!isSubsequenceIgnoringCase(search, type)) return -1.0f;

    if (type == search) return 1.0f;

    const float inverseLength = 1.0f / static_cast<float>(type.size());

    float w = 0.5f;
    const int pos = findIgnoringCase(type, search);
    float closeness = 3.0f * static_cast<float>(pos) * inverseLength;
    if (closeness > 1.0f) closeness = 1.0f;
    float score = (pos > -1) ? 1.0f - w * closeness : (0.9f - w > 0.0f ? 0.9f - w : 0.0f);

    w = 0.9f;
    float fit = static_cast<float>(search.size()) * inverseLength;
    if (fit > 1.0f) fit = 1.0f;
    score *= (1.0f - w) + w * fit;

    score *= isTypePreferred(type) ? 1.0f : 0.9f;

    score *= mSettings.isCreateFavorite(type) ? 1.0f : 0.8f;

    bool inRecent = false;
    std::size_t limit = static_cast<std::size_t>(kRecentCompletionSize - 1);
    if (mSettings.createRecent.size() < limit) limit = mSettings.createRecent.size();
    for (std::size_t i = 0; i < limit; ++i)
    {
        if (mSettings.createRecent[i] == type)
        {
            inRecent = true;
            break;
        }
    }
    score *= inRecent ? 1.0f : 0.9f;

    return score;
}

ct::String CreateNodeDialog::searchText() const
{
    return mSearch->text();
}

void CreateNodeDialog::setSearchText(const ct::String& text)
{
    applySearch(text);
}

void CreateNodeDialog::applySearch(const ct::String& text)
{
    mSilent = true;
    mSearch->setText(text);
    mSilent = false;
    updateSearch();
}

void CreateNodeDialog::textChanged()
{
    if (mSilent) return;
    updateSearch();
}

void CreateNodeDialog::updateSearch()
{
    mPendingNode = nullptr;
    mSelecting = true;
    mOptions->clearRows();
    mSelecting = false;
    mItemOf.clear();
    mTypeOf.clear();

    const ct::String search = mSearch->text();

    rt::TreeNode* root = mOptions->addRoot(kBaseType);
    mItemOf.put(ct::String(kBaseType), root);
    mTypeOf.put(root, ct::String(kBaseType));
    configureItem(root, ct::String(kBaseType), search);

    float highest = 0.0f;
    ct::String best;

    for (std::size_t i = 0; i < mTypes.size(); ++i)
    {
        const ct::String& candidate = mTypes[i];

        const float score = scoreType(candidate, search);
        if (score < 0.0f) continue;

        addType(candidate, search);

        if (score > highest)
        {
            highest = score;
            best = candidate;
        }
    }

    mNoResults = ct::String();
    mStatus->setText("");
    if (search.empty())
    {
        selectType(ct::String(kBaseType));
    }
    else if (!best.empty())
    {
        selectType(best);
    }
    else
    {
        mFavorite->setEnabled(false);
        mFavorite->setChecked(false);
        mNoResults = ct::String("No results for \"") + search + "\".";
        mStatus->setText(mNoResults);
        mCreate->setEnabled(false);
        mCreate->setTooltip("");
        mSelecting = true;
        mOptions->setSelectedNode(nullptr);
        mSelecting = false;
    }
}

void CreateNodeDialog::addType(const ct::String& type, const ct::String& search)
{
    if (mItemOf.contains(type)) return;

    const ct::String inherits = NodeClassDB::getParentClass(type);
    if (inherits.empty()) return;

    addType(inherits, search);

    rt::TreeNode* const* parent = mItemOf.find(inherits);
    if (!parent) return;

    rt::TreeNode* item = (*parent)->addChild(type);
    mItemOf.put(type, item);
    mTypeOf.put(item, type);
    configureItem(item, type, search);
}

void CreateNodeDialog::configureItem(rt::TreeNode* item, const ct::String& type,
        const ct::String& search)
{
    const bool canInstantiate = NodeClassDB::canInstantiate(type);

    item->setIconId(mIcons.icon(type));

    if (!search.empty())
    {
        item->setExpanded(true);
        return;
    }

    const rt::TreeNode* parent = item->parent();
    const bool shouldCollapse =
            type != kBaseType && (parent->text() != kBaseType || canInstantiate);
    item->setExpanded(!shouldCollapse);
}

bool CreateNodeDialog::selectType(const ct::String& type, bool center)
{
    rt::TreeNode* const* found = mItemOf.find(type);
    if (!found) return false;
    rt::TreeNode* item = *found;

    for (rt::TreeNode* parent = item->parent(); parent; parent = parent->parent())
        if (!parent->isExpanded()) parent->setExpanded(true);

    mSelecting = true;
    mOptions->setSelectedNode(item);
    mSelecting = false;

    mFavorite->setEnabled(true);
    mFavorite->setChecked(mSettings.isCreateFavorite(type));

    if (NodeClassDB::canInstantiate(type))
    {
        mCreate->setEnabled(true);
        mCreate->setTooltip("");
    }
    else
    {
        mCreate->setEnabled(false);
        mCreate->setTooltip(kNotInstantiableTooltip);
    }
    mStatus->setText("");

    scrollToSelected(item, center);
    return true;
}

void CreateNodeDialog::scrollToSelected(rt::TreeNode* item, bool center)
{
    if (mOptions->rect().h > 0.0f)
    {
        mOptions->scrollTo(item, center);
        return;
    }
    mPendingNode = item;
    mPendingCenter = center;
}

void CreateNodeDialog::layout()
{
    rt::FloatWindow::layout();
    if (!mPendingNode) return;
    rt::TreeNode* item = mPendingNode;
    mPendingNode = nullptr;
    mOptions->scrollTo(item, mPendingCenter);
}

void CreateNodeDialog::itemSelected(rt::TreeNode* item)
{
    if (mSelecting || !item) return;
    const ct::String* type = mTypeOf.find(item);
    if (type) selectType(*type, false);
}

void CreateNodeDialog::listPressed(rt::TreeNode* item)
{
    if (!item) return;
    const ct::String name = item->text();
    const bool fromFavorites = mFavorites->selectedNode() == item;
    if (fromFavorites) mRecent->setSelectedNode(nullptr);
    else
        mFavorites->setSelectedNode(nullptr);
    applySearch(name);
}

ct::String CreateNodeDialog::selectedType() const
{
    const rt::TreeNode* item = mOptions->selectedNode();
    if (!item) return ct::String();
    const ct::String* type = mTypeOf.find(const_cast<rt::TreeNode*>(item));
    return type ? *type : ct::String();
}

bool CreateNodeDialog::isInstantiable(const ct::String& type) const
{
    return mItemOf.contains(type) && NodeClassDB::canInstantiate(type);
}

bool CreateNodeDialog::canCreate() const
{
    const ct::String type = selectedType();
    return !type.empty() && NodeClassDB::canInstantiate(type);
}

bool CreateNodeDialog::hasType(const ct::String& type) const
{
    return mItemOf.contains(type);
}

rt::TreeNode* CreateNodeDialog::itemOf(const ct::String& type) const
{
    rt::TreeNode* const* item = mItemOf.find(type);
    return item ? *item : nullptr;
}

const ct::String& CreateNodeDialog::noResultsText() const
{
    return mNoResults;
}

void CreateNodeDialog::toggleFavorite()
{
    const ct::String type = selectedType();
    if (type.empty()) return;

    const bool favorite = !mSettings.isCreateFavorite(type);
    mSettings.setCreateFavorite(type, favorite);
    mFavorite->setChecked(favorite);
    refreshFavorites();
    persist();
}

void CreateNodeDialog::toggleCollapse()
{
    rt::TreeNode* item = mOptions->selectedNode();
    if (item) item->setExpanded(!item->isExpanded());
}

void CreateNodeDialog::navigate(int steps)
{
    rt::TreeNode* target = mOptions->neighbour(mOptions->selectedNode(), steps);
    if (!target) return;
    const ct::String* type = mTypeOf.find(target);
    if (type) selectType(*type, false);
}

void CreateNodeDialog::confirm()
{
    if (mClosed) return;
    const ct::String type = selectedType();
    if (type.empty() || !NodeClassDB::canInstantiate(type)) return;

    mSettings.rememberCreated(type);
    persist();

    close();
    created.emit(type);
}

void CreateNodeDialog::cancel()
{
    if (mClosed) return;
    close();
}

void CreateNodeDialog::close()
{
    mClosed = true;
    closedWith.emit(mSearch->text());
    rt::WidgetApp::instance().removeFloat(this);
}

void CreateNodeDialog::onKeyPress(rt::KeyEvent& event)
{
    if (event.key == rt::Key::Escape)
    {
        event.consumed = true;
        cancel();
        return;
    }
    if (event.key == rt::Key::Up || event.key == rt::Key::Down)
    {
        event.consumed = true;
        navigate(event.key == rt::Key::Up ? -1 : 1);
        return;
    }
    if (event.key == rt::Key::PageUp || event.key == rt::Key::PageDown)
    {
        event.consumed = true;
        const int page = mOptions->pageRows();
        navigate(event.key == rt::Key::PageUp ? -page : page);
        return;
    }
    if (event.key == rt::Key::Return || event.key == rt::Key::KPEnter)
    {
        event.consumed = true;
        confirm();
        return;
    }
    rt::FloatWindow::onKeyPress(event);
}

rt::Widget* CreateNodeDialog::focusOnOpen()
{
    return mSearch;
}

ActivatableTree* CreateNodeDialog::optionsTree() const
{
    return mOptions;
}

ActivatableTree* CreateNodeDialog::favoritesTree() const
{
    return mFavorites;
}

ActivatableTree* CreateNodeDialog::recentTree() const
{
    return mRecent;
}

CreateSearchInput* CreateNodeDialog::searchInput() const
{
    return mSearch;
}

FavoriteButton* CreateNodeDialog::favoriteButton() const
{
    return mFavorite;
}

rt::Button* CreateNodeDialog::createButton() const
{
    return mCreate;
}

rt::Button* CreateNodeDialog::cancelButton() const
{
    return mCancel;
}

rt::Label* CreateNodeDialog::statusLabel() const
{
    return mStatus;
}

} // namespace prisma3d
