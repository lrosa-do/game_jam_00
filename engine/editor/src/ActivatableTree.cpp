#include "prisma3d/editor/ActivatableTree.h"

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

void collect(rt::TreeNode* node, ct::Vector<rt::TreeNode*>& rows)
{
    rows.push_back(node);
    if (!node->isExpanded()) return;
    for (rt::TreeNode* child: node->children()) collect(child, rows);
}

} // namespace

ActivatableTree::ActivatableTree()
        : mScroll(0.0f)
{
}

void ActivatableTree::clearRows()
{
    clear();
    mScroll = 0.0f;
}

ct::Vector<rt::TreeNode*> ActivatableTree::rows() const
{
    ct::Vector<rt::TreeNode*> result;
    for (rt::TreeNode* root: roots()) collect(root, result);
    return result;
}

rt::TreeNode* ActivatableTree::nodeAt(float x, float y) const
{
    const rt::Rect area = absoluteRect();
    if (!area.contains(x, y)) return nullptr;
    const float local = y - area.y + mScroll;
    if (local < 0.0f) return nullptr;
    const ct::Vector<rt::TreeNode*> visible = rows();
    const std::size_t index = static_cast<std::size_t>(local / kRowHeight);
    return index < visible.size() ? visible[index] : nullptr;
}

rt::TreeNode* ActivatableTree::neighbour(const rt::TreeNode* from, int steps) const
{
    const ct::Vector<rt::TreeNode*> visible = rows();
    if (visible.empty()) return nullptr;
    int index = -1;
    for (std::size_t i = 0; i < visible.size(); ++i)
        if (visible[i] == from) index = static_cast<int>(i);
    if (index < 0) return steps >= 0 ? visible.front() : visible.back();
    int target = index + steps;
    const int last = static_cast<int>(visible.size()) - 1;
    if (target < 0) target = 0;
    if (target > last) target = last;
    return visible[static_cast<std::size_t>(target)];
}

bool ActivatableTree::isVisibleRow(const rt::TreeNode* node) const
{
    const ct::Vector<rt::TreeNode*> visible = rows();
    for (std::size_t i = 0; i < visible.size(); ++i)
        if (visible[i] == node) return true;
    return false;
}

int ActivatableTree::pageRows() const
{
    const int page = static_cast<int>(rect().h / kRowHeight) - 1;
    return page < 1 ? 1 : page;
}

float ActivatableTree::scrollPosition() const
{
    return mScroll;
}

void ActivatableTree::scrollTo(const rt::TreeNode* node, bool center)
{
    const ct::Vector<rt::TreeNode*> visible = rows();
    int index = -1;
    for (std::size_t i = 0; i < visible.size(); ++i)
        if (visible[i] == node) index = static_cast<int>(i);
    if (index < 0) return;

    const float height = rect().h;
    const float top = static_cast<float>(index) * kRowHeight;
    float wanted = mScroll;
    if (center) wanted = top - (height - kRowHeight) * 0.5f;
    else if (top < mScroll)
        wanted = top;
    else if (top + kRowHeight > mScroll + height)
        wanted = top + kRowHeight - height;
    if (wanted < 0.0f) wanted = 0.0f;

    const rt::Rect area = absoluteRect();
    rt::MouseEvent event;
    event.x = area.x + 1.0f;
    event.y = area.y + 1.0f;
    event.scrollY = (mScroll - wanted) / (kRowHeight * 2.0f);
    onMouseScroll(event);
}

void ActivatableTree::onMousePress(rt::MouseEvent& event)
{
    rt::TreeNode* hit = nodeAt(event.x, event.y);
    const unsigned int revision = rt::TreeNode::revision();
    rt::TreeView::onMousePress(event);
    if (!hit || hit != selectedNode() || rt::TreeNode::revision() != revision) return;
    rowPressed.emit(hit);
    if (event.clickCount >= 2) activated.emit(hit);
}

void ActivatableTree::onMouseScroll(rt::MouseEvent& event)
{
    rt::TreeView::onMouseScroll(event);
    if (!event.consumed) return;
    const float total = static_cast<float>(rows().size()) * kRowHeight;
    const float limit = total > rect().h ? total - rect().h : 0.0f;
    mScroll -= event.scrollY * kRowHeight * 2.0f;
    if (mScroll < 0.0f) mScroll = 0.0f;
    if (mScroll > limit) mScroll = limit;
}

} // namespace prisma3d
