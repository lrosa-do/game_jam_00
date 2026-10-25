#pragma once

#include <igui/widgets/Signal.hpp>
#include <igui/widgets/TreePropertyColorWidgets.hpp>

#include <ct/vector.hpp>

namespace prisma3d
{

class ActivatableTree : public ig::retained::TreeView
{
public:
    IG_WIDGET_CLASS(ActivatableTree, ig::retained::TreeView);

    static constexpr float kRowHeight = 24.0f;

    ActivatableTree();

    void clearRows();
    ct::Vector<ig::retained::TreeNode*> rows() const;
    ig::retained::TreeNode* nodeAt(float x, float y) const;
    ig::retained::TreeNode* neighbour(const ig::retained::TreeNode* from, int steps) const;
    bool isVisibleRow(const ig::retained::TreeNode* node) const;
    int pageRows() const;
    float scrollPosition() const;
    void scrollTo(const ig::retained::TreeNode* node, bool center);

    ig::retained::Signal<ig::retained::TreeNode*> rowPressed;
    ig::retained::Signal<ig::retained::TreeNode*> activated;

    void onMousePress(ig::retained::MouseEvent& event) override;
    void onMouseScroll(ig::retained::MouseEvent& event) override;

private:
    float mScroll;
};

} // namespace prisma3d
