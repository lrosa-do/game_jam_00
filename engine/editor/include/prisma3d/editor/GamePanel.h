#pragma once

#include "prisma3d/Input.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/Widget.hpp>

#include <ct/ptr.hpp>
#include <ct/string.hpp>

#include <cstdint>

namespace prisma3d
{

class Camera3D;
class EditorDocument;
class GameImage;
class Node;

class GamePanel
{
public:
    static constexpr const char* kNoCameraMessage = "No camera in the scene";

    GamePanel(RenderingServer& server, EditorDocument& document);
    ~GamePanel();
    GamePanel(const GamePanel&) = delete;
    GamePanel& operator=(const GamePanel&) = delete;

    bool attach(ig::retained::Widget* parent);
    bool attached() const;

    void setScenario(ScenarioId scenario);
    ScenarioId scenario() const;
    void setClearColor(const Math::Vec4& color);

    void connectPreDraw();
    void disconnectPreDraw();
    void update(float delta);
    bool handleEvent(const InputEvent& event);
    bool wantsInput(const InputEvent& event) const;
    bool ownsInput(const InputEvent& event) const;
    bool hovered(int x, int y) const;
    bool focused() const;
    bool captured() const;
    bool wantsPolling(int x, int y) const;
    void releaseFocus();

    static Camera3D* findCamera(Node* scene);
    Camera3D* camera() const;
    bool showingMessage() const;
    const ct::String& message() const;
    const ct::String& caption() const;

    ig::retained::Widget* view() const;
    ViewportId viewportId() const;
    CameraId cameraId() const;
    TextureId texture() const;
    ig::retained::TextureHandle shownTexture() const;
    std::uint32_t width() const;
    std::uint32_t height() const;
    bool rendering() const;

private:
    struct Link
    {
        GamePanel* owner;
    };

    bool visibleInTree() const;
    void syncSize();
    void syncCamera();

    RenderingServer& mServer;
    EditorDocument& mDocument;
    ct::Rc<Link> mLink;
    ViewportId mViewport;
    ScenarioId mScenario;
    ViewportUpdateMode mUpdateMode;
    ig::retained::BoxLayout* mColumn;
    GameImage* mView;
    Camera3D* mCamera;
    TextureId mTexture;
    ct::String mCaption;
    std::uint32_t mWidth;
    std::uint32_t mHeight;
    unsigned int mSerial;
    unsigned int mDragButtons;
    unsigned int mOutsideButtons;
    int mMouseX;
    int mMouseY;
    bool mPreDraw;
    bool mFocused;
};

} // namespace prisma3d
