#pragma once

#include "prisma3d/Input.h"

#include "prisma/rhi/Driver.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

struct PlatformWindow;

namespace prisma3d
{

enum class WindowMode : int
{
    Windowed,
    Minimized,
    Maximized,
    Fullscreen,
    ExclusiveFullscreen
};

enum class VSyncMode : int
{
    Disabled,
    Enabled,
    // No tearing and no wait for the display: the newest finished frame is shown at the next
    // refresh and older pending ones are dropped (Vulkan mailbox; with OpenGL the same as
    // Enabled). The frame rate is not tied to the refresh rate.
    Mailbox
};

enum class WindowFlag : int
{
    ResizeDisabled,
    Borderless,
    AlwaysOnTop,
    Max
};

enum WindowFlagBit : unsigned
{
    kWindowFlagResizeDisabled = 1u << static_cast<int>(WindowFlag::ResizeDisabled),
    kWindowFlagBorderless = 1u << static_cast<int>(WindowFlag::Borderless),
    kWindowFlagAlwaysOnTop = 1u << static_cast<int>(WindowFlag::AlwaysOnTop)
};

enum class MouseMode : int
{
    Visible,
    Hidden,
    Captured
};

enum class CursorShape : int
{
    Arrow,
    IBeam,
    PointingHand,
    Cross,
    Wait,
    Busy,
    Drag,
    CanDrop,
    Forbidden,
    VSize,
    HSize,
    BDiagSize,
    FDiagSize,
    Move,
    VSplit,
    HSplit,
    Help,
    Max
};

enum Screen : int
{
    kScreenWithMouseFocus = -4,
    kScreenWithKeyboardFocus = -3,
    kScreenPrimary = -2,
    kScreenOfMainWindow = -1
};

class MouseMotionTracker
{
public:
    MouseMotionTracker();

    void motion(int x, int y, int rawX, int rawY, bool captured, int& relativeX, int& relativeY);

private:
    int mLastX;
    int mLastY;
    bool mValid;
};

struct DeviceDesc
{
    const char* title = "prisma3d";
    prisma::DriverType driver = prisma::DriverType::Vulkan;
    int width = 1280;
    int height = 720;
    int screen = kScreenWithMouseFocus;
    WindowMode mode = WindowMode::Windowed;
    VSyncMode vsync = VSyncMode::Enabled;
    unsigned flags = 0;
    bool headless = false;
};

class Device
{
public:
    static Device* create(const DeviceDesc& desc);
    static void destroy(Device* device);
    static Device* singleton();
    static const char* lastError();
    static bool headlessSupported();

    prisma::Driver* driver() const;
    prisma::DriverType driverType() const;
    bool isHeadless() const;
    Input& input();

    void processEvents();
    bool closeRequested() const;
    void requestClose();
    void cancelClose();
    bool windowCanDraw() const;

    int getScreenCount() const;
    int getPrimaryScreen() const;
    int getKeyboardFocusScreen() const;
    void screenGetPosition(int screen, int* x, int* y) const;
    void screenGetSize(int screen, int* width, int* height) const;
    void screenGetUsableRect(int screen, int* x, int* y, int* width, int* height) const;
    int screenGetDpi(int screen) const;
    float screenGetScale(int screen) const;
    float screenGetRefreshRate(int screen) const;

    void windowSetTitle(const char* title);
    int windowGetCurrentScreen() const;
    void windowSetCurrentScreen(int screen);
    void windowGetPosition(int* x, int* y) const;
    void windowSetPosition(int x, int y);
    void windowGetSize(int* width, int* height) const;
    void windowSetSize(int width, int height);
    void windowGetFramebufferSize(int* width, int* height) const;
    float windowGetScale() const;
    void windowSetMinSize(int width, int height);
    void windowGetMinSize(int* width, int* height) const;
    void windowSetMaxSize(int width, int height);
    void windowGetMaxSize(int* width, int* height) const;
    void windowSetMode(WindowMode mode);
    WindowMode windowGetMode() const;
    void windowSetVSyncMode(VSyncMode mode);
    VSyncMode windowGetVSyncMode() const;
    void windowSetFlag(WindowFlag flag, bool enabled);
    bool windowGetFlag(WindowFlag flag) const;
    void windowSetIcon(int width, int height, const std::uint8_t* rgba);
    void windowRequestAttention();
    void windowMoveToForeground();
    bool windowIsFocused() const;
    void windowShow();
    void windowHide();
    bool windowIsVisible() const;

    void mouseSetMode(MouseMode mode);
    MouseMode mouseGetMode() const;
    void warpMouse(int x, int y);
    void mouseGetPosition(int* x, int* y) const;
    unsigned mouseGetButtonState() const;
    void cursorSetShape(CursorShape shape);
    CursorShape cursorGetShape() const;

    void clipboardSet(const char* text);
    const char* clipboardGet() const;
    bool clipboardHas() const;

    void setTextInput(bool active);
    bool textInputActive() const;

    ct::Vector<ct::String> takeDroppedFiles();

    std::uint64_t getTicksUsec() const;
    std::uint64_t getTicksMsec() const;
    void delayUsec(std::uint32_t usec) const;

    bool takeScreenshot(const char* path = nullptr);
    bool requestScreenshot(const char* path);
    bool screenshotSaved() const;
    bool requestFrameCapture();
    bool takeFrameCapture(ct::Vector<std::uint8_t>& pixels, int& width, int& height);
    bool gifRecordingStart(const char* path = nullptr);
    bool gifRecordingStop();
    bool isGifRecording() const;
    int gifRecordingFrames() const;
    void gifRecordingFrame();

private:
    Device();
    ~Device();
    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;

    bool open(const DeviceDesc& desc);
    bool openHeadless(const DeviceDesc& desc);
    void close();
    int resolveScreen(int screen) const;

    static Device* sSingleton;

    struct HeadlessState;
    struct ScreenCapture;

    bool readScreen(ScreenCapture& capture);
    bool readScreenPixels(ct::Vector<std::uint8_t>& pixels, int& width, int& height);

    PlatformWindow* mWindow;
    HeadlessState* mHeadless;
    ScreenCapture* mCapture;
    prisma::Driver* mDriver;
    prisma::DriverType mDriverType;
    Input mInput;
    bool mCloseRequested;
    ct::Vector<ct::String> mDroppedFiles;
    MouseMode mMouseMode;
    MouseMotionTracker mMouseMotion;
    CursorShape mCursorShape;
    VSyncMode mVSyncMode;
    bool mFlags[static_cast<int>(WindowFlag::Max)];
    int mMinWidth, mMinHeight;
    int mMaxWidth, mMaxHeight;
};

} // namespace prisma3d
