#include "prisma3d/Device.h"

#include "prisma3d/core/Pixmap.h"

#include "msf_gif.h"
#include "platform.h"

#include <ct/vector.hpp>

#include <stdio.h>

#ifdef PRISMA3D_HEADLESS
#include "Headless.h"
#endif

namespace prisma3d
{

namespace
{

#ifdef NDEBUG
const bool kDebug = false;
#else
const bool kDebug = true;
#endif

#define PRISMA3D_SAME_KEY(engine, platform)                                                        \
    static_assert(static_cast<int>(Key::engine) == platform, "key table out of step")

PRISMA3D_SAME_KEY(None, KEY_NULL);
PRISMA3D_SAME_KEY(Space, KEY_SPACE);
PRISMA3D_SAME_KEY(Apostrophe, KEY_APOSTROPHE);
PRISMA3D_SAME_KEY(Comma, KEY_COMMA);
PRISMA3D_SAME_KEY(Minus, KEY_MINUS);
PRISMA3D_SAME_KEY(Period, KEY_PERIOD);
PRISMA3D_SAME_KEY(Slash, KEY_SLASH);
PRISMA3D_SAME_KEY(Zero, KEY_ZERO);
PRISMA3D_SAME_KEY(Nine, KEY_NINE);
PRISMA3D_SAME_KEY(Semicolon, KEY_SEMICOLON);
PRISMA3D_SAME_KEY(Equal, KEY_EQUAL);
PRISMA3D_SAME_KEY(A, KEY_A);
PRISMA3D_SAME_KEY(Z, KEY_Z);
PRISMA3D_SAME_KEY(LeftBracket, KEY_LEFT_BRACKET);
PRISMA3D_SAME_KEY(Backslash, KEY_BACKSLASH);
PRISMA3D_SAME_KEY(RightBracket, KEY_RIGHT_BRACKET);
PRISMA3D_SAME_KEY(Grave, KEY_GRAVE);
PRISMA3D_SAME_KEY(Escape, KEY_ESCAPE);
PRISMA3D_SAME_KEY(Enter, KEY_ENTER);
PRISMA3D_SAME_KEY(Tab, KEY_TAB);
PRISMA3D_SAME_KEY(Backspace, KEY_BACKSPACE);
PRISMA3D_SAME_KEY(Insert, KEY_INSERT);
PRISMA3D_SAME_KEY(Delete, KEY_DELETE);
PRISMA3D_SAME_KEY(Right, KEY_RIGHT);
PRISMA3D_SAME_KEY(Left, KEY_LEFT);
PRISMA3D_SAME_KEY(Down, KEY_DOWN);
PRISMA3D_SAME_KEY(Up, KEY_UP);
PRISMA3D_SAME_KEY(PageUp, KEY_PAGE_UP);
PRISMA3D_SAME_KEY(PageDown, KEY_PAGE_DOWN);
PRISMA3D_SAME_KEY(Home, KEY_HOME);
PRISMA3D_SAME_KEY(End, KEY_END);
PRISMA3D_SAME_KEY(CapsLock, KEY_CAPS_LOCK);
PRISMA3D_SAME_KEY(NumLock, KEY_NUM_LOCK);
PRISMA3D_SAME_KEY(PrintScreen, KEY_PRINT_SCREEN);
PRISMA3D_SAME_KEY(Pause, KEY_PAUSE);
PRISMA3D_SAME_KEY(F1, KEY_F1);
PRISMA3D_SAME_KEY(F12, KEY_F12);
PRISMA3D_SAME_KEY(Kp0, KEY_KP_0);
PRISMA3D_SAME_KEY(Kp9, KEY_KP_9);
PRISMA3D_SAME_KEY(KpDecimal, KEY_KP_DECIMAL);
PRISMA3D_SAME_KEY(KpDivide, KEY_KP_DIVIDE);
PRISMA3D_SAME_KEY(KpMultiply, KEY_KP_MULTIPLY);
PRISMA3D_SAME_KEY(KpSubtract, KEY_KP_SUBTRACT);
PRISMA3D_SAME_KEY(KpAdd, KEY_KP_ADD);
PRISMA3D_SAME_KEY(KpEnter, KEY_KP_ENTER);
PRISMA3D_SAME_KEY(KpEqual, KEY_KP_EQUAL);
PRISMA3D_SAME_KEY(LeftShift, KEY_LEFT_SHIFT);
PRISMA3D_SAME_KEY(LeftControl, KEY_LEFT_CONTROL);
PRISMA3D_SAME_KEY(LeftAlt, KEY_LEFT_ALT);
PRISMA3D_SAME_KEY(LeftSuper, KEY_LEFT_SUPER);
PRISMA3D_SAME_KEY(RightShift, KEY_RIGHT_SHIFT);
PRISMA3D_SAME_KEY(RightControl, KEY_RIGHT_CONTROL);
PRISMA3D_SAME_KEY(RightAlt, KEY_RIGHT_ALT);
PRISMA3D_SAME_KEY(RightSuper, KEY_RIGHT_SUPER);
PRISMA3D_SAME_KEY(Menu, KEY_MENU);
PRISMA3D_SAME_KEY(ScrollLock, KEY_SCROLL_LOCK);
PRISMA3D_SAME_KEY(Max, KEY_MAX);

#undef PRISMA3D_SAME_KEY

static_assert(static_cast<unsigned>(kModShift) == KEYMOD_SHIFT &&
                      static_cast<unsigned>(kModControl) == KEYMOD_CTRL &&
                      static_cast<unsigned>(kModAlt) == KEYMOD_ALT &&
                      static_cast<unsigned>(kModSuper) == KEYMOD_SUPER &&
                      static_cast<unsigned>(kModCapsLock) == KEYMOD_CAPS_LOCK &&
                      static_cast<unsigned>(kModNumLock) == KEYMOD_NUM_LOCK,
        "modifier table out of step");

static_assert(static_cast<int>(MouseButton::Left) == MOUSE_LEFT &&
                      static_cast<int>(MouseButton::Right) == MOUSE_RIGHT &&
                      static_cast<int>(MouseButton::Middle) == MOUSE_MIDDLE &&
                      static_cast<int>(MouseButton::X1) == MOUSE_X1 &&
                      static_cast<int>(MouseButton::X2) == MOUSE_X2 &&
                      static_cast<int>(MouseButton::Max) == MOUSE_BUTTON_MAX,
        "mouse button table out of step");

static_assert(static_cast<int>(TouchPhase::Down) == TOUCH_DOWN &&
                      static_cast<int>(TouchPhase::Move) == TOUCH_MOVE &&
                      static_cast<int>(TouchPhase::Up) == TOUCH_UP &&
                      static_cast<int>(TouchPhase::Cancel) == TOUCH_CANCEL,
        "touch phase table out of step");

int platformCursor(CursorShape shape)
{
    switch (shape)
    {
        case CursorShape::Arrow:
            return CURSOR_ARROW;
        case CursorShape::IBeam:
            return CURSOR_IBEAM;
        case CursorShape::PointingHand:
        case CursorShape::Drag:
        case CursorShape::CanDrop:
            return CURSOR_HAND;
        case CursorShape::Cross:
            return CURSOR_CROSSHAIR;
        case CursorShape::Forbidden:
            return CURSOR_NOT_ALLOWED;
        case CursorShape::VSize:
        case CursorShape::VSplit:
            return CURSOR_RESIZE_NS;
        case CursorShape::HSize:
        case CursorShape::HSplit:
            return CURSOR_RESIZE_EW;
        case CursorShape::BDiagSize:
            return CURSOR_RESIZE_NESW;
        case CursorShape::FDiagSize:
            return CURSOR_RESIZE_NWSE;
        case CursorShape::Move:
            return CURSOR_RESIZE_ALL;
        default:
            return CURSOR_DEFAULT;
    }
}

bool makeCurrent(void* user)
{
    window_make_current(static_cast<PlatformWindow*>(user));
    return true;
}

void swapBuffers(void* user)
{
    window_swap(static_cast<PlatformWindow*>(user));
}

void framebufferSize(void* user, std::uint32_t* width, std::uint32_t* height)
{
    int w = 0;
    int h = 0;
    window_get_framebuffer_size(static_cast<PlatformWindow*>(user), &w, &h);
    *width = w > 0 ? static_cast<std::uint32_t>(w) : 0;
    *height = h > 0 ? static_cast<std::uint32_t>(h) : 0;
}

const char* const* instanceExtensions(void*, std::uint32_t* count)
{
    return vulkan_instance_extensions(count);
}

bool createSurface(void* user, void* instance, std::uint64_t* surface)
{
    return vulkan_create_surface(static_cast<PlatformWindow*>(user), instance, nullptr, surface);
}

void logMessage(const char* message)
{
    log_error("prisma: %s", message);
}

PlatformWindow* openWindow(const DeviceDesc& desc, int monitor)
{
    WindowConfig config = {};
    config.title = desc.title;
    config.width = desc.width;
    config.height = desc.height;
    config.x = WINDOW_POS_CENTERED;
    config.y = WINDOW_POS_CENTERED;
    config.monitor = monitor;
    config.resizable = (desc.flags & kWindowFlagResizeDisabled) == 0;
    config.undecorated = (desc.flags & kWindowFlagBorderless) != 0;
    config.vsync = desc.vsync != VSyncMode::Disabled;
    if (desc.mode == WindowMode::Fullscreen) config.mode = WINDOW_FULLSCREEN_BORDERLESS;
    if (desc.mode == WindowMode::ExclusiveFullscreen) config.mode = WINDOW_FULLSCREEN;

    if (desc.driver == prisma::DriverType::Vulkan)
    {
        if (!vulkan_supported())
        {
            log_error("device: Vulkan is not available");
            return nullptr;
        }
        config.render = RENDER_VULKAN;
        return window_create(&config);
    }

    config.render = RENDER_GL;
    config.gl.debug = kDebug;
#ifdef PRISMA_GLES
    config.gl.profile = GL_PROFILE_ES;
    config.gl.major = 3;
    for (int minor = 2; minor >= 0; --minor)
    {
        config.gl.minor = minor;
        PlatformWindow* window = window_create(&config);
        if (window) return window;
    }
    return nullptr;
#else
    config.gl.profile = GL_PROFILE_CORE;
    config.gl.major = 4;
    config.gl.minor = 6;
    return window_create(&config);
#endif
}

// A thread with a GL context that shares objects with the window's, where the driver links
// programs without stalling the frame. It stays null where the platform has no shared contexts.
struct SharedWorker
{
    struct Job
    {
        void (*run)(void* arg);
        void* arg;
    };

    GLWorker* context = nullptr;
    PlatformThread* thread = nullptr;
    PlatformMutex* mutex = nullptr;
    PlatformCond* wake = nullptr;
    ct::Vector<Job> jobs;
    std::size_t next = 0;
    bool started = false;
    bool current = false;
    bool stop = false;
};

SharedWorker* sShared = nullptr;

int sharedWorkerMain(void* user)
{
    SharedWorker* worker = static_cast<SharedWorker*>(user);
    mutex_lock(worker->mutex);
    worker->current = gl_worker_make_current(worker->context, true);
    worker->started = true;
    cond_broadcast(worker->wake);
    while (worker->current)
    {
        while (worker->next == worker->jobs.size() && !worker->stop)
            cond_wait(worker->wake, worker->mutex);
        if (worker->next == worker->jobs.size()) break;
        const SharedWorker::Job job = worker->jobs[worker->next++];
        if (worker->next == worker->jobs.size())
        {
            worker->jobs.clear();
            worker->next = 0;
        }
        mutex_unlock(worker->mutex);
        job.run(job.arg);
        mutex_lock(worker->mutex);
    }
    if (worker->current) gl_worker_make_current(worker->context, false);
    mutex_unlock(worker->mutex);
    return 0;
}

void stopSharedWorker()
{
    SharedWorker* worker = sShared;
    if (!worker) return;
    sShared = nullptr;
    mutex_lock(worker->mutex);
    worker->stop = true;
    cond_broadcast(worker->wake);
    mutex_unlock(worker->mutex);
    // The worker touches the window system connection to release its context, so wait here.
    thread_join(worker->thread);
    gl_worker_destroy(worker->context);
    cond_destroy(worker->wake);
    mutex_destroy(worker->mutex);
    delete worker;
}

void startSharedWorker(PlatformWindow* window)
{
    GLWorker* context = gl_worker_create(window);
    if (!context) return;
    SharedWorker* worker = new SharedWorker();
    worker->context = context;
    worker->mutex = mutex_create();
    worker->wake = cond_create();
    worker->thread = thread_create(sharedWorkerMain, worker, "gl link");
    if (!worker->thread)
    {
        gl_worker_destroy(context);
        cond_destroy(worker->wake);
        mutex_destroy(worker->mutex);
        delete worker;
        return;
    }
    sShared = worker;
    // Binding the context uses the window system connection too: stay out of it until done.
    mutex_lock(worker->mutex);
    while (!worker->started) cond_wait(worker->wake, worker->mutex);
    const bool current = worker->current;
    mutex_unlock(worker->mutex);
    if (!current) stopSharedWorker();
}

bool runShared(void*, void (*job)(void* arg), void* arg)
{
    SharedWorker* worker = sShared;
    if (!worker) return false;
    mutex_lock(worker->mutex);
    worker->jobs.push_back(SharedWorker::Job{ job, arg });
    cond_broadcast(worker->wake);
    mutex_unlock(worker->mutex);
    return true;
}

prisma::Driver* openDriver(PlatformWindow* window, prisma::DriverType type)
{
    startSharedWorker(window);

    prisma::GLPlatform gl;
    gl.user = window;
    gl.makeCurrent = makeCurrent;
    gl.swapBuffers = swapBuffers;
    gl.framebufferSize = framebufferSize;
    gl.getProcAddress = gl_proc_address;
    gl.runShared = sShared ? runShared : nullptr;

    prisma::VulkanPlatform vulkan;
    vulkan.user = window;
    vulkan.instanceExtensions = instanceExtensions;
    vulkan.createSurface = createSurface;
    vulkan.framebufferSize = framebufferSize;

    prisma::DriverDesc desc;
    desc.type = type;
    desc.gl = &gl;
    desc.vulkan = &vulkan;
    desc.log = logMessage;
    desc.debug = kDebug;

    prisma::DriverError error = prisma::DriverError::None;
    prisma::Driver* driver = prisma::createDriver(desc, &error);
    if (!driver)
    {
        log_error("driver: %s", prisma::driverErrorText(error));
        stopSharedWorker();
    }
    return driver;
}

void translate(const Event& in, InputEvent& out)
{
    switch (in.type)
    {
        case EVENT_KEY:
            out.type = InputEventType::Key;
            out.key.key = static_cast<Key>(in.data.key.key);
            out.key.scancode = in.data.key.scancode;
            out.key.pressed = in.data.key.down;
            out.key.echo = in.data.key.repeat;
            out.key.modifiers = static_cast<unsigned>(in.data.key.mods);
            break;
        case EVENT_CHAR:
            out.type = InputEventType::Char;
            out.character.codepoint = in.data.codepoint;
            break;
        case EVENT_MOUSE_MOVE:
            out.type = InputEventType::MouseMotion;
            out.mouseMotion.x = in.data.mouse.x;
            out.mouseMotion.y = in.data.mouse.y;
            out.mouseMotion.relativeX = in.data.mouse.dx;
            out.mouseMotion.relativeY = in.data.mouse.dy;
            out.mouseMotion.buttonMask = 0;
            break;
        case EVENT_MOUSE_BUTTON:
            out.type = InputEventType::MouseButton;
            out.mouseButton.x = in.data.mouse.x;
            out.mouseButton.y = in.data.mouse.y;
            out.mouseButton.button = static_cast<MouseButton>(in.data.mouse.button);
            out.mouseButton.pressed = in.data.mouse.down;
            out.mouseButton.modifiers = 0;
            break;
        case EVENT_MOUSE_WHEEL:
            out.type = InputEventType::MouseWheel;
            out.mouseWheel.x = 0;
            out.mouseWheel.y = 0;
            out.mouseWheel.deltaX = in.data.wheel.x;
            out.mouseWheel.deltaY = in.data.wheel.y;
            break;
        case EVENT_TOUCH:
            out.type = InputEventType::Touch;
            out.touch.id = in.data.touch.id;
            out.touch.x = in.data.touch.x;
            out.touch.y = in.data.touch.y;
            out.touch.pressure = in.data.touch.pressure;
            out.touch.phase = static_cast<TouchPhase>(in.data.touch.phase);
            break;
        default:
            out.type = InputEventType::None;
            break;
    }
}

} // namespace

struct Device::ScreenCapture
{
    MsfGifState gif;
    ct::Vector<unsigned char> pixels;
    char gifPath[512];
    int width = 0;
    int height = 0;
    int frameCounter = 0;
    int fileCounter = 0;
    bool recording = false;
    ct::String requestedShot;
    bool shotRequested = false;
    bool shotSaved = false;
    ct::Vector<std::uint8_t> framePixels;
    int frameWidth = 0;
    int frameHeight = 0;
    bool frameRequested = false;
    bool frameReady = false;
};

struct Device::HeadlessState
{
#ifdef PRISMA3D_HEADLESS
    headless::Context context;
    headless::Surface surface;
#else
    int unused = 0;
#endif
};

Device* Device::sSingleton = nullptr;

MouseMotionTracker::MouseMotionTracker()
        : mLastX(0),
          mLastY(0),
          mValid(false)
{
}

void MouseMotionTracker::motion(int x, int y, int rawX, int rawY, bool captured, int& relativeX,
        int& relativeY)
{
    if (!mValid)
    {
        mLastX = x;
        mLastY = y;
        mValid = true;
    }
    if (captured && (rawX != 0 || rawY != 0))
    {
        relativeX = rawX;
        relativeY = rawY;
    }
    else
    {
        relativeX = x - mLastX;
        relativeY = y - mLastY;
    }
    mLastX = x;
    mLastY = y;
}

Device::Device()
        : mWindow(nullptr),
          mHeadless(nullptr),
          mCapture(nullptr),
          mDriver(nullptr),
          mDriverType(prisma::DriverType::Null),
          mCloseRequested(false),
          mMouseMode(MouseMode::Visible),
          mCursorShape(CursorShape::Arrow),
          mVSyncMode(VSyncMode::Enabled),
          mMinWidth(0),
          mMinHeight(0),
          mMaxWidth(0),
          mMaxHeight(0)
{
    for (int i = 0; i < static_cast<int>(WindowFlag::Max); ++i) mFlags[i] = false;
}

Device::~Device()
{
    close();
}

Device* Device::create(const DeviceDesc& desc)
{
    if (sSingleton)
    {
        log_error("device: already created");
        return nullptr;
    }
    Device* device = new Device();
    if (!device->open(desc))
    {
        delete device;
        if (desc.driver != prisma::DriverType::Vulkan) return nullptr;
        log_warn("device: Vulkan is not available, falling back to OpenGL");
        DeviceDesc fallback = desc;
        fallback.driver = prisma::DriverType::OpenGL;
        device = new Device();
        if (!device->open(fallback))
        {
            delete device;
            return nullptr;
        }
    }
    sSingleton = device;
    return device;
}

void Device::destroy(Device* device)
{
    if (!device) return;
    if (sSingleton == device) sSingleton = nullptr;
    delete device;
}

Device* Device::singleton()
{
    return sSingleton;
}

const char* Device::lastError()
{
    return platform_get_error();
}

bool Device::headlessSupported()
{
#ifdef PRISMA3D_HEADLESS
    return true;
#else
    return false;
#endif
}

bool Device::openHeadless(const DeviceDesc& desc)
{
#ifdef PRISMA3D_HEADLESS
    mHeadless = new HeadlessState();
    mHeadless->surface.width = desc.width > 0 ? static_cast<std::uint32_t>(desc.width) : 1;
    mHeadless->surface.height = desc.height > 0 ? static_cast<std::uint32_t>(desc.height) : 1;

    if (desc.driver == prisma::DriverType::OpenGL)
    {
#ifdef PRISMA_GLES
        const bool es = true;
#else
        const bool es = false;
#endif
        if (!headless::openContext(&mHeadless->context, es, kDebug) ||
                !headless::openSurface(&mHeadless->context, mHeadless->surface.width,
                        mHeadless->surface.height, &mHeadless->surface))
        {
            log_error("device: could not create a headless OpenGL context");
            close();
            return false;
        }
    }

    prisma::GLPlatform gl = headless::glPlatform(&mHeadless->surface);
    prisma::VulkanPlatform vulkan = headless::vulkanPlatform(&mHeadless->surface);

    prisma::DriverDesc driverDesc;
    driverDesc.type = desc.driver;
    driverDesc.gl = &gl;
    driverDesc.vulkan = &vulkan;
    driverDesc.log = logMessage;
    driverDesc.debug = kDebug;

    prisma::DriverError error = prisma::DriverError::None;
    mDriver = prisma::createDriver(driverDesc, &error);
    if (!mDriver)
    {
        log_error("driver: %s", prisma::driverErrorText(error));
        close();
        return false;
    }
    mDriverType = desc.driver;
    mVSyncMode = desc.vsync;
    return true;
#else
    (void) desc;
    log_error("device: this build has no headless mode");
    return false;
#endif
}

bool Device::open(const DeviceDesc& desc)
{
    if (desc.headless) return openHeadless(desc);

    if (!platform_init())
    {
        log_error("platform: %s", platform_get_error());
        return false;
    }
    const std::uint64_t platformNanos = time_nanos();

    int monitor = MONITOR_MOUSE;
    if (desc.screen >= 0) monitor = desc.screen;
    if (desc.screen == kScreenPrimary)
    {
        const int count = monitor_count();
        for (int i = 0; i < count; ++i)
        {
            MonitorInfo info;
            if (monitor_get_info(i, &info) && info.primary) monitor = i;
        }
    }

    mWindow = openWindow(desc, monitor);
    if (!mWindow)
    {
        log_error("window: %s", platform_get_error());
        platform_shutdown();
        return false;
    }
    window_text_input_stop(mWindow);
    const std::uint64_t windowNanos = time_nanos();

    mDriverType = desc.driver;
    mVSyncMode = desc.vsync;
    for (int i = 0; i < static_cast<int>(WindowFlag::Max); ++i)
        mFlags[i] = (desc.flags & (1u << i)) != 0;
    if (mFlags[static_cast<int>(WindowFlag::AlwaysOnTop)]) window_set_always_on_top(mWindow, true);
    if (desc.mode == WindowMode::Maximized) window_maximize(mWindow);
    if (desc.mode == WindowMode::Minimized) window_minimize(mWindow);

    mDriver = openDriver(mWindow, desc.driver);
    if (mDriver) mDriver->setVSync(desc.vsync == VSyncMode::Enabled, desc.vsync == VSyncMode::Mailbox);
    if (!mDriver)
    {
        window_destroy(mWindow);
        mWindow = nullptr;
        platform_shutdown();
        return false;
    }
    const std::uint64_t driverNanos = time_nanos();
    log_info("device: startup window %.1f ms, driver %.1f ms",
            static_cast<double>(windowNanos - platformNanos) * 1e-6,
            static_cast<double>(driverNanos - windowNanos) * 1e-6);
    return true;
}

void Device::close()
{
    if (mCapture)
    {
        gifRecordingStop();
        delete mCapture;
        mCapture = nullptr;
    }
    if (mDriver)
    {
        prisma::destroyDriver(mDriver);
        mDriver = nullptr;
    }
    stopSharedWorker();
    if (mHeadless)
    {
#ifdef PRISMA3D_HEADLESS
        headless::closeSurface(&mHeadless->surface);
        headless::closeContext(&mHeadless->context);
#endif
        delete mHeadless;
        mHeadless = nullptr;
    }
    if (mWindow)
    {
        window_destroy(mWindow);
        mWindow = nullptr;
        platform_shutdown();
    }
}

bool Device::isHeadless() const
{
    return mHeadless != nullptr;
}

prisma::Driver* Device::driver() const
{
    return mDriver;
}

prisma::DriverType Device::driverType() const
{
    return mDriverType;
}

Input& Device::input()
{
    return mInput;
}

void Device::processEvents()
{
    mInput.beginFrame();
    if (!mWindow) return;
    window_begin_frame(mWindow);
    const unsigned modifiers = static_cast<unsigned>(key_mods(mWindow));

    Event event;
    while (poll_event(mWindow, &event))
    {
        switch (event.type)
        {
            case EVENT_WINDOW_CLOSE:
                mCloseRequested = true;
                break;
            case EVENT_WINDOW_FOCUS:
                mInput.setApplicationFocused(event.data.focus.gained);
                break;
            case EVENT_WINDOW_DROP:
                for (int i = 0; i < event.data.drop.count; ++i)
                    mDroppedFiles.push_back(ct::String(event.data.drop.paths[i]));
                break;
            default:
            {
                InputEvent translated;
                translate(event, translated);
                if (translated.type == InputEventType::MouseMotion)
                {
                    translated.mouseMotion.buttonMask = mInput.getMouseButtonMask();
                    mMouseMotion.motion(translated.mouseMotion.x, translated.mouseMotion.y,
                            event.data.mouse.dx, event.data.mouse.dy,
                            mMouseMode == MouseMode::Captured, translated.mouseMotion.relativeX,
                            translated.mouseMotion.relativeY);
                }
                if (translated.type == InputEventType::MouseButton)
                    translated.mouseButton.modifiers = modifiers;
                if (translated.type == InputEventType::MouseWheel)
                {
                    const Math::Vec2 position = mInput.getMousePosition();
                    translated.mouseWheel.x = static_cast<int>(position.x);
                    translated.mouseWheel.y = static_cast<int>(position.y);
                }
                if (translated.type != InputEventType::None) mInput.parseInputEvent(translated);
                break;
            }
        }
    }

    if (window_should_close(mWindow)) mCloseRequested = true;
}

bool Device::closeRequested() const
{
    return mCloseRequested;
}

void Device::requestClose()
{
    mCloseRequested = true;
}

void Device::cancelClose()
{
    mCloseRequested = false;
    if (mWindow) window_set_should_close(mWindow, false);
}

bool Device::windowCanDraw() const
{
    if (!mWindow) return mDriver != nullptr;
    return window_is_visible(mWindow) && !window_is_minimized(mWindow);
}

int Device::getScreenCount() const
{
    if (!mWindow) return 0;
    return monitor_count();
}

int Device::getPrimaryScreen() const
{
    if (!mWindow) return 0;
    const int count = monitor_count();
    for (int i = 0; i < count; ++i)
    {
        MonitorInfo info;
        if (monitor_get_info(i, &info) && info.primary) return i;
    }
    return 0;
}

int Device::getKeyboardFocusScreen() const
{
    if (!mWindow) return 0;
    return monitor_from_window(mWindow);
}

int Device::resolveScreen(int screen) const
{
    if (!mWindow) return screen < 0 ? 0 : screen;
    switch (screen)
    {
        case kScreenWithMouseFocus:
        {
            const int mouse = monitor_from_mouse();
            return mouse >= 0 ? mouse : monitor_from_window(mWindow);
        }
        case kScreenWithKeyboardFocus:
            return getKeyboardFocusScreen();
        case kScreenPrimary:
            return getPrimaryScreen();
        case kScreenOfMainWindow:
            return monitor_from_window(mWindow);
        default:
            return screen;
    }
}

void Device::screenGetPosition(int screen, int* x, int* y) const
{
    if (x) *x = 0;
    if (y) *y = 0;
    if (!mWindow) return;
    MonitorInfo info = {};
    monitor_get_info(resolveScreen(screen), &info);
    if (x) *x = info.x;
    if (y) *y = info.y;
}

void Device::screenGetSize(int screen, int* width, int* height) const
{
    if (width) *width = 0;
    if (height) *height = 0;
    if (!mWindow) return;
    MonitorInfo info = {};
    monitor_get_info(resolveScreen(screen), &info);
    if (width) *width = info.width;
    if (height) *height = info.height;
}

void Device::screenGetUsableRect(int screen, int* x, int* y, int* width, int* height) const
{
    if (x) *x = 0;
    if (y) *y = 0;
    if (width) *width = 0;
    if (height) *height = 0;
    if (!mWindow) return;
    MonitorInfo info = {};
    monitor_get_info(resolveScreen(screen), &info);
    if (x) *x = info.work_x;
    if (y) *y = info.work_y;
    if (width) *width = info.work_w;
    if (height) *height = info.work_h;
}

int Device::screenGetDpi(int screen) const
{
    if (!mWindow) return 96;
    MonitorInfo info = {};
    if (!monitor_get_info(resolveScreen(screen), &info) || info.phys_width_mm <= 0) return 96;
    const float pixels = static_cast<float>(info.width) * info.content_scale;
    return static_cast<int>(pixels * 25.4f / static_cast<float>(info.phys_width_mm) + 0.5f);
}

float Device::screenGetScale(int screen) const
{
    if (!mWindow) return 1.0f;
    MonitorInfo info = {};
    if (!monitor_get_info(resolveScreen(screen), &info)) return 1.0f;
    return info.content_scale;
}

float Device::screenGetRefreshRate(int screen) const
{
    if (!mWindow) return -1.0f;
    MonitorInfo info = {};
    if (!monitor_get_info(resolveScreen(screen), &info)) return -1.0f;
    return static_cast<float>(info.refresh_hz);
}

void Device::windowSetTitle(const char* title)
{
    if (!mWindow) return;
    window_set_title(mWindow, title);
}

int Device::windowGetCurrentScreen() const
{
    if (!mWindow) return 0;
    return monitor_from_window(mWindow);
}

void Device::windowSetCurrentScreen(int screen)
{
    if (!mWindow) return;
    window_set_monitor(mWindow, resolveScreen(screen));
}

void Device::windowGetPosition(int* x, int* y) const
{
    if (x) *x = 0;
    if (y) *y = 0;
    if (!mWindow) return;
    int px = 0;
    int py = 0;
    window_get_position(mWindow, &px, &py);
    if (x) *x = px;
    if (y) *y = py;
}

void Device::windowSetPosition(int x, int y)
{
    if (!mWindow) return;
    window_set_position(mWindow, x, y);
}

void Device::windowGetSize(int* width, int* height) const
{
    if (!mWindow)
    {
        windowGetFramebufferSize(width, height);
        return;
    }
    int w = 0;
    int h = 0;
    window_get_size(mWindow, &w, &h);
    if (width) *width = w;
    if (height) *height = h;
}

void Device::windowSetSize(int width, int height)
{
    if (!mWindow) return;
    window_set_size(mWindow, width, height);
}

void Device::windowGetFramebufferSize(int* width, int* height) const
{
    if (!mWindow)
    {
#ifdef PRISMA3D_HEADLESS
        if (width) *width = mHeadless ? static_cast<int>(mHeadless->surface.width) : 0;
        if (height) *height = mHeadless ? static_cast<int>(mHeadless->surface.height) : 0;
#else
        if (width) *width = 0;
        if (height) *height = 0;
#endif
        return;
    }
    int w = 0;
    int h = 0;
    window_get_framebuffer_size(mWindow, &w, &h);
    if (width) *width = w;
    if (height) *height = h;
}

float Device::windowGetScale() const
{
    if (!mWindow) return 1.0f;
    return window_content_scale(mWindow);
}

void Device::windowSetMinSize(int width, int height)
{
    mMinWidth = width;
    mMinHeight = height;
    if (!mWindow) return;
    if (!mFlags[static_cast<int>(WindowFlag::ResizeDisabled)])
        window_set_size_limits(mWindow, mMinWidth, mMinHeight, mMaxWidth, mMaxHeight);
}

void Device::windowGetMinSize(int* width, int* height) const
{
    if (width) *width = mMinWidth;
    if (height) *height = mMinHeight;
}

void Device::windowSetMaxSize(int width, int height)
{
    mMaxWidth = width;
    mMaxHeight = height;
    if (!mWindow) return;
    if (!mFlags[static_cast<int>(WindowFlag::ResizeDisabled)])
        window_set_size_limits(mWindow, mMinWidth, mMinHeight, mMaxWidth, mMaxHeight);
}

void Device::windowGetMaxSize(int* width, int* height) const
{
    if (width) *width = mMaxWidth;
    if (height) *height = mMaxHeight;
}

void Device::windowSetMode(WindowMode mode)
{
    if (!mWindow) return;
    switch (mode)
    {
        case WindowMode::Windowed:
            window_set_mode(mWindow, WINDOW_WINDOWED, MONITOR_CURRENT);
            window_restore(mWindow);
            break;
        case WindowMode::Minimized:
            window_minimize(mWindow);
            break;
        case WindowMode::Maximized:
            window_set_mode(mWindow, WINDOW_WINDOWED, MONITOR_CURRENT);
            window_maximize(mWindow);
            break;
        case WindowMode::Fullscreen:
            window_set_mode(mWindow, WINDOW_FULLSCREEN_BORDERLESS, MONITOR_CURRENT);
            break;
        case WindowMode::ExclusiveFullscreen:
            window_set_mode(mWindow, WINDOW_FULLSCREEN, MONITOR_CURRENT);
            break;
    }
}

WindowMode Device::windowGetMode() const
{
    if (!mWindow) return WindowMode::Windowed;
    switch (window_get_mode(mWindow))
    {
        case WINDOW_FULLSCREEN:
            return WindowMode::ExclusiveFullscreen;
        case WINDOW_FULLSCREEN_BORDERLESS:
            return WindowMode::Fullscreen;
        default:
            break;
    }
    if (window_is_minimized(mWindow)) return WindowMode::Minimized;
    if (window_is_maximized(mWindow)) return WindowMode::Maximized;
    return WindowMode::Windowed;
}

void Device::windowSetVSyncMode(VSyncMode mode)
{
    mVSyncMode = mode;
    if (mDriver) mDriver->setVSync(mode == VSyncMode::Enabled, mode == VSyncMode::Mailbox);
    if (!mWindow || mDriverType != prisma::DriverType::OpenGL) return;
    window_set_vsync(mWindow, mode != VSyncMode::Disabled);
}

VSyncMode Device::windowGetVSyncMode() const
{
    return mVSyncMode;
}

void Device::windowSetFlag(WindowFlag flag, bool enabled)
{
    if (flag == WindowFlag::Max) return;
    mFlags[static_cast<int>(flag)] = enabled;
    if (!mWindow) return;
    switch (flag)
    {
        case WindowFlag::ResizeDisabled:
            if (enabled)
            {
                int w = 0;
                int h = 0;
                window_get_size(mWindow, &w, &h);
                window_set_size_limits(mWindow, w, h, w, h);
            }
            else
            {
                window_set_size_limits(mWindow, mMinWidth, mMinHeight, mMaxWidth, mMaxHeight);
            }
            break;
        case WindowFlag::Borderless:
            window_set_decorated(mWindow, !enabled);
            break;
        case WindowFlag::AlwaysOnTop:
            window_set_always_on_top(mWindow, enabled);
            break;
        default:
            break;
    }
}

bool Device::windowGetFlag(WindowFlag flag) const
{
    if (flag == WindowFlag::Max) return false;
    return mFlags[static_cast<int>(flag)];
}

void Device::windowSetIcon(int width, int height, const std::uint8_t* rgba)
{
    if (!mWindow) return;
    window_set_icon(mWindow, width, height, rgba);
}

void Device::windowRequestAttention()
{
    if (!mWindow) return;
    window_request_attention(mWindow);
}

void Device::windowMoveToForeground()
{
    if (!mWindow) return;
    window_focus(mWindow);
}

void Device::windowShow()
{
    if (mWindow) window_show(mWindow);
}

void Device::windowHide()
{
    if (mWindow) window_hide(mWindow);
}

bool Device::windowIsVisible() const
{
    return mWindow && window_is_visible(mWindow);
}

bool Device::windowIsFocused() const
{
    if (!mWindow) return true;
    return window_is_focused(mWindow);
}

void Device::mouseSetMode(MouseMode mode)
{
    mMouseMode = mode;
    if (!mWindow) return;
    switch (mode)
    {
        case MouseMode::Visible:
            mouse_set_mode(mWindow, MOUSE_MODE_NORMAL);
            break;
        case MouseMode::Hidden:
            mouse_set_mode(mWindow, MOUSE_MODE_HIDDEN);
            break;
        case MouseMode::Captured:
            mouse_set_mode(mWindow, MOUSE_MODE_CAPTURED);
            break;
    }
}

MouseMode Device::mouseGetMode() const
{
    return mMouseMode;
}

void Device::warpMouse(int x, int y)
{
    if (!mWindow) return;
    mouse_set_position(mWindow, x, y);
}

void Device::mouseGetPosition(int* x, int* y) const
{
    if (x) *x = 0;
    if (y) *y = 0;
    if (!mWindow) return;
    int px = 0;
    int py = 0;
    if (!mouse_global_position(&px, &py)) mouse_position(mWindow, &px, &py);
    if (x) *x = px;
    if (y) *y = py;
}

unsigned Device::mouseGetButtonState() const
{
    if (!mWindow) return 0;
    unsigned mask = 0;
    for (int i = 0; i < static_cast<int>(MouseButton::Max); ++i)
        if (mouse_button_down(mWindow, i)) mask |= 1u << i;
    return mask;
}

void Device::cursorSetShape(CursorShape shape)
{
    if (shape == CursorShape::Max) return;
    mCursorShape = shape;
    if (!mWindow) return;
    mouse_set_cursor(mWindow, platformCursor(shape));
}

CursorShape Device::cursorGetShape() const
{
    return mCursorShape;
}

void Device::clipboardSet(const char* text)
{
    if (!mWindow) return;
    clipboard_set(text);
}

void Device::setTextInput(bool active)
{
    if (!mWindow || window_text_input_active(mWindow) == active) return;
    if (active) window_text_input_start(mWindow);
    else
        window_text_input_stop(mWindow);
}

bool Device::textInputActive() const
{
    return mWindow && window_text_input_active(mWindow);
}

const char* Device::clipboardGet() const
{
    if (!mWindow) return "";
    return clipboard_get();
}

bool Device::clipboardHas() const
{
    if (!mWindow) return false;
    const char* text = clipboard_get();
    return text && text[0] != '\0';
}

ct::Vector<ct::String> Device::takeDroppedFiles()
{
    ct::Vector<ct::String> files = mDroppedFiles;
    mDroppedFiles.clear();
    return files;
}

std::uint64_t Device::getTicksUsec() const
{
    return time_nanos() / 1000u;
}

std::uint64_t Device::getTicksMsec() const
{
    return time_nanos() / 1000000u;
}

void Device::delayUsec(std::uint32_t usec) const
{
    time_sleep((usec + 999u) / 1000u);
}

namespace
{

const int kGifRecordFramerate = 10;
const int kGifCentisecondsPerFrame = 10;
const int kGifMaxBitDepth = 16;

} // namespace

bool Device::readScreenPixels(ct::Vector<std::uint8_t>& pixels, int& width, int& height)
{
    if (!mDriver) return false;
    width = 0;
    height = 0;
    windowGetFramebufferSize(&width, &height);
    if (width <= 0 || height <= 0) return false;
    pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4);
    prisma::Rect rect;
    rect.width = static_cast<std::uint32_t>(width);
    rect.height = static_cast<std::uint32_t>(height);
    return mDriver->readPixels(prisma::RenderTarget(), rect, pixels.data());
}

bool Device::readScreen(ScreenCapture& capture)
{
    return readScreenPixels(capture.pixels, capture.width, capture.height);
}

bool Device::requestFrameCapture()
{
    if (!mCapture) mCapture = new ScreenCapture();
    mCapture->frameRequested = true;
    mCapture->frameReady = false;
    return true;
}

bool Device::takeFrameCapture(ct::Vector<std::uint8_t>& pixels, int& width, int& height)
{
    if (!mCapture || !mCapture->frameReady) return false;
    mCapture->frameReady = false;
    pixels = static_cast<ct::Vector<std::uint8_t>&&>(mCapture->framePixels);
    width = mCapture->frameWidth;
    height = mCapture->frameHeight;
    mCapture->framePixels = ct::Vector<std::uint8_t>();
    return true;
}

bool Device::takeScreenshot(const char* path)
{
    if (!mCapture) mCapture = new ScreenCapture();
    ScreenCapture& capture = *mCapture;
    char name[64];
    if (!path)
    {
        snprintf(name, sizeof(name), "screenshot%03i.png", capture.fileCounter);
        ++capture.fileCounter;
        path = name;
    }
    if (!readScreen(capture))
    {
        log_error("device: could not read the screen for %s", path);
        return false;
    }
    Pixmap image(capture.width, capture.height, 4, capture.pixels.data());
    if (!image.save(path))
    {
        log_error("device: screenshot could not be saved: %s", path);
        return false;
    }
    log_info("device: screenshot taken: %s", path);
    return true;
}

bool Device::gifRecordingStart(const char* path)
{
    if (!mCapture) mCapture = new ScreenCapture();
    ScreenCapture& capture = *mCapture;
    if (capture.recording) return false;
    int width = 0;
    int height = 0;
    windowGetFramebufferSize(&width, &height);
    if (width <= 0 || height <= 0 || !mDriver) return false;
    if (path)
    {
        snprintf(capture.gifPath, sizeof(capture.gifPath), "%s", path);
    }
    else
    {
        snprintf(capture.gifPath, sizeof(capture.gifPath), "screenrec%03i.gif",
                capture.fileCounter);
        ++capture.fileCounter;
    }
    if (!msf_gif_begin(&capture.gif, width, height))
    {
        log_error("device: could not start the GIF recording");
        return false;
    }
    capture.width = width;
    capture.height = height;
    capture.frameCounter = 0;
    capture.recording = true;
    log_info("device: start animated GIF recording: %s", capture.gifPath);
    return true;
}

bool Device::gifRecordingStop()
{
    if (!mCapture || !mCapture->recording) return false;
    ScreenCapture& capture = *mCapture;
    capture.recording = false;
    MsfGifResult result = msf_gif_end(&capture.gif);
    bool saved = false;
    if (result.data && result.dataSize > 0)
    {
        if (FILE* file = fopen(capture.gifPath, "wb"))
        {
            saved = fwrite(result.data, 1, result.dataSize, file) == result.dataSize;
            fclose(file);
        }
    }
    msf_gif_free(result);
    if (saved) log_info("device: finish animated GIF recording: %s", capture.gifPath);
    else
        log_error("device: GIF recording could not be saved: %s", capture.gifPath);
    return saved;
}

bool Device::isGifRecording() const
{
    return mCapture && mCapture->recording;
}

int Device::gifRecordingFrames() const
{
    return mCapture && mCapture->recording ? mCapture->frameCounter : 0;
}

bool Device::requestScreenshot(const char* path)
{
    if (!path || !path[0]) return false;
    if (!mCapture) mCapture = new ScreenCapture();
    mCapture->requestedShot = path;
    mCapture->shotRequested = true;
    mCapture->shotSaved = false;
    return true;
}

bool Device::screenshotSaved() const
{
    return mCapture && mCapture->shotSaved;
}

void Device::gifRecordingFrame()
{
    if (mCapture && mCapture->shotRequested)
    {
        mCapture->shotRequested = false;
        const ct::String path = mCapture->requestedShot;
        mCapture->shotSaved = takeScreenshot(path.c_str());
    }
    if (mCapture && mCapture->frameRequested)
    {
        mCapture->frameRequested = false;
        mCapture->frameReady = readScreenPixels(mCapture->framePixels, mCapture->frameWidth,
                mCapture->frameHeight);
    }
    if (!mCapture || !mCapture->recording) return;
    ScreenCapture& capture = *mCapture;
    ++capture.frameCounter;
    if (capture.frameCounter % kGifRecordFramerate != 0) return;
    const int width = capture.width;
    const int height = capture.height;
    if (!readScreen(capture) || capture.width != width || capture.height != height)
    {
        log_error("device: the window changed size; GIF recording stopped");
        gifRecordingStop();
        return;
    }
    msf_gif_frame(&capture.gif, capture.pixels.data(), kGifCentisecondsPerFrame, kGifMaxBitDepth,
            width * 4);
}

} // namespace prisma3d
