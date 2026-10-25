#pragma once

#include "prisma3d/Input.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/CodeEditor.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/MenuWidgets.hpp>
#include <igui/widgets/ToolBar.hpp>

#include <ct/function.hpp>
#include <ct/ptr.hpp>
#include <ct/string.hpp>

namespace prisma3d
{

class EditorDocument;

class ScriptEditorPanel
{
public:
    static constexpr const char* kScriptFilter = "*.py";

    explicit ScriptEditorPanel(EditorDocument& document);
    ~ScriptEditorPanel();
    ScriptEditorPanel(const ScriptEditorPanel&) = delete;
    ScriptEditorPanel& operator=(const ScriptEditorPanel&) = delete;

    bool attach(ig::retained::Widget* parent);
    bool attached() const;
    void setRevealHandler(ct::Function<void()> handler);

    void open(const ct::String& path);
    bool save();
    void requestOpen();
    void requestSaveAs();
    void cut();
    void copy();
    void paste();
    void undo();
    void redo();
    void zoomIn();
    void zoomOut();
    void zoomReset();
    void update();
    bool handleEvent(const InputEvent& event);
    bool focused() const;

    const ct::String& path() const;
    bool modified() const;
    ig::retained::CodeEditor* editor() const;

private:
    struct Link
    {
        ScriptEditorPanel* owner;
    };

    bool load(const ct::String& path);
    bool writeTo(const ct::String& path);
    void toolClicked(int id);
    void refreshTitle();
    bool visibleInTree() const;

    EditorDocument& mDocument;
    ct::Rc<Link> mLink;
    ct::Function<void()> mReveal;
    ig::retained::BoxLayout* mRoot;
    ig::retained::MenuBar* mMenuBar;
    ig::retained::ToolBar* mToolBar;
    ig::retained::Label* mTitle;
    ig::retained::CodeEditor* mEditor;
    ct::String mPath;
    int mOpenButton;
    int mSaveButton;
    int mCutButton;
    int mCopyButton;
    int mPasteButton;
    bool mModified;
    bool mLoading;
    bool mControl;
};

} // namespace prisma3d
