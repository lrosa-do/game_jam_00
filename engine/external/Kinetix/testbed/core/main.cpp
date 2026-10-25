// main.cpp - the testbed shell: a scene list, a clock, and the verdicts.

#include "scene.h"

#include <SDL.h>
#include <GL/gl.h>

#include <imgui.h>
#include <backends/imgui_impl_opengl2.h>
#include <backends/imgui_impl_sdl2.h>

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

using namespace testbed;

namespace
{

constexpr float kFixedStep = 1.0f / 60.0f;

const char* gRunningScene = nullptr;

// Turns a scene that never returns from step() into a named failure and a non-zero exit.
void onWatchdog(int)
{
    const char* name = gRunningScene ? gRunningScene : "?";
    // write(), not printf(): only async-signal-safe calls belong here.
    const char* prefix = "\nHUNG in scene: ";
    ssize_t ignored = write(2, prefix, strlen(prefix));
    ignored = write(2, name, strlen(name));
    ignored = write(2, "\n", 1);
    (void)ignored;
    _exit(2);
}

int runHeadless(float seconds, usize only, bool haveOnly)
{
    printf("Kinetix testbed - headless, %.1fs per scene\n\n", double(seconds));
    signal(SIGALRM, onWatchdog);
    int failed = 0;
    const int steps = int(seconds / kFixedStep);

    for (usize i = 0; i < sceneCount(); ++i)
    {
        if (haveOnly && i != only)
            continue;
        Scene* scene = createScene(i);
        if (!scene)
            continue;

        gRunningScene = scene->name();
        alarm(60);

        for (int s = 0; s < steps; ++s)
        {
            scene->step(kFixedStep);
            scene->evaluate();
        }

        alarm(0);

        const bool ok = scene->allPassed();
        printf("%-22s %s\n", scene->name(), ok ? "PASS" : "FAIL");
        for (const Check& c : scene->checks())
        {
            // A passing check with a detail is a measurement (benchmarks): printed every time.
            if (!c.everFailed && c.detail.size() == 0)
                continue;
            printf("    %s %s", c.everFailed ? "x" : "+", c.label.c_str());
            if (c.detail.size() > 0)
                printf("   (%s)", c.detail.c_str());
            printf("\n");
        }
        if (!ok)
            ++failed;
        delete scene;
    }

    printf("\n%s (%d of %d scenes failed)\n", failed ? "FAILURES" : "all scenes passed", failed,
           int(sceneCount()));
    return failed != 0;
}

void writeFramebuffer(const char* path, int width, int height)
{
    ct::Vector<unsigned char> pixels;
    pixels.resize(usize(width) * usize(height) * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

    FILE* file = fopen(path, "wb");
    if (!file)
        return;
    fprintf(file, "P6\n%d %d\n255\n", width, height);
    // glReadPixels starts at the bottom row; PPM expects the top one.
    for (int y = height - 1; y >= 0; --y)
        fwrite(pixels.data() + usize(y) * usize(width) * 3, 1, usize(width) * 3, file);
    fclose(file);
    printf("wrote %s (%dx%d)\n", path, width, height);
}

// --keys=<timeline>: a scripted keyboard fed through the real key path. Entries are comma separated:
//   W@0-3.5    hold W from 0 s to 3.5 s of simulated time
//   H@4        tap H once at 4 s (held for one step: a key-press edge)
// Key names as keyName() gives them: letters, digits, Up, Down, Left, Right,
// PageUp, PageDown, Home, End, Shift, Ctrl, Tab, ...
struct TimedKey
{
    Key key;
    int firstStep;
    int endStep; // exclusive
};

bool parseTimeline(const char* spec, ct::Vector<TimedKey>& out)
{
    const char* p = spec;
    while (*p)
    {
        const char* at = strchr(p, '@');
        if (!at)
            return false;
        TimedKey entry;
        if (!keyFromName(p, usize(at - p), entry.key))
        {
            fprintf(stderr, "--keys: unknown key '%.*s'\n", int(at - p), p);
            return false;
        }
        char* end = nullptr;
        const double start = strtod(at + 1, &end);
        double stop = start;
        bool tap = true;
        if (*end == '-')
        {
            stop = strtod(end + 1, &end);
            tap = false;
        }
        entry.firstStep = int(start / double(kFixedStep) + 0.5);
        entry.endStep = tap ? entry.firstStep + 1 : int(stop / double(kFixedStep) + 0.5);
        if (entry.endStep <= entry.firstStep)
            entry.endStep = entry.firstStep + 1;
        out.push_back(entry);
        if (*end == ',')
            ++end;
        else if (*end != '\0')
            return false;
        p = end;
    }
    return true;
}

void applyTimeline(const ct::Vector<TimedKey>& timeline, int step, bool* held)
{
    for (const TimedKey& k : timeline)
        if (step >= k.firstStep && step < k.endStep)
            held[usize(k.key)] = true;
}

void computeEdges(ControlInput& input, const bool* previous)
{
    for (usize i = 0; i < usize(Key::Count); ++i)
        input.pressed[i] = input.pressed[i] || (input.held[i] && !previous[i]);
}

void printReadouts(Scene* scene, float t)
{
    ct::Vector<Readout> lines;
    scene->readouts(lines);
    printf("t=%5.2f", double(t));
    for (const Readout& r : lines)
        printf("  %s %s", r.label, r.value);
    printf("\n");
}

int runDriven(usize index, const ct::Vector<TimedKey>& timeline, float seconds, float every)
{
    Scene* scene = createScene(index);
    if (!scene)
        return 1;
    if (!scene->drivable())
    {
        fprintf(stderr, "%s has no manual mode\n", scene->name());
        delete scene;
        return 1;
    }
    scene->setManual(true);
    printf("%s - manual, key timeline, %.1fs\n", scene->name(), double(seconds));
    gRunningScene = scene->name();
    signal(SIGALRM, onWatchdog);
    alarm(120);
    const int steps = int(seconds / kFixedStep);
    const int printEvery = every > 0.0f ? int(every / kFixedStep + 0.5f) : 0;
    bool previous[usize(Key::Count)] = {};
    for (int s = 0; s < steps; ++s)
    {
        ControlInput input;
        applyTimeline(timeline, s, input.held);
        computeEdges(input, previous);
        memcpy(previous, input.held, sizeof(previous));
        scene->control(input, kFixedStep);
        if (scene->restartRequested)
        {
            delete scene;
            scene = createScene(index);
            scene->setManual(true);
            continue;
        }
        scene->step(kFixedStep);
        if (printEvery > 0 && (s + 1) % printEvery == 0)
            printReadouts(scene, float(s + 1) * kFixedStep);
    }
    alarm(0);
    printf("final:\n");
    ct::Vector<Readout> lines;
    scene->readouts(lines);
    for (const Readout& r : lines)
        printf("    %-18s %s\n", r.label, r.value);
    delete scene;
    return 0;
}

const SDL_Scancode kScancodes[usize(Key::Count)] = {
    SDL_SCANCODE_A, SDL_SCANCODE_B, SDL_SCANCODE_C, SDL_SCANCODE_D, SDL_SCANCODE_E,
    SDL_SCANCODE_F, SDL_SCANCODE_G, SDL_SCANCODE_H, SDL_SCANCODE_I, SDL_SCANCODE_J,
    SDL_SCANCODE_K, SDL_SCANCODE_L, SDL_SCANCODE_M, SDL_SCANCODE_N, SDL_SCANCODE_O,
    SDL_SCANCODE_P, SDL_SCANCODE_Q, SDL_SCANCODE_R, SDL_SCANCODE_S, SDL_SCANCODE_T,
    SDL_SCANCODE_U, SDL_SCANCODE_V, SDL_SCANCODE_W, SDL_SCANCODE_X, SDL_SCANCODE_Y,
    SDL_SCANCODE_Z, SDL_SCANCODE_0, SDL_SCANCODE_1, SDL_SCANCODE_2, SDL_SCANCODE_3,
    SDL_SCANCODE_4, SDL_SCANCODE_5, SDL_SCANCODE_6, SDL_SCANCODE_7, SDL_SCANCODE_8,
    SDL_SCANCODE_9, SDL_SCANCODE_UP, SDL_SCANCODE_DOWN, SDL_SCANCODE_LEFT, SDL_SCANCODE_RIGHT,
    SDL_SCANCODE_PAGEUP, SDL_SCANCODE_PAGEDOWN, SDL_SCANCODE_HOME, SDL_SCANCODE_END,
    SDL_SCANCODE_INSERT, SDL_SCANCODE_DELETE, SDL_SCANCODE_TAB, SDL_SCANCODE_LSHIFT,
    SDL_SCANCODE_LCTRL, SDL_SCANCODE_RETURN, SDL_SCANCODE_BACKSPACE, SDL_SCANCODE_MINUS,
    SDL_SCANCODE_EQUALS, SDL_SCANCODE_LEFTBRACKET, SDL_SCANCODE_RIGHTBRACKET,
    SDL_SCANCODE_COMMA, SDL_SCANCODE_PERIOD};

// Keys the shell keeps for itself: they never reach a scene's ControlInput.
bool shellOwns(Key key)
{
    return key == Key::P;
}

void readKeyboard(ControlInput& input)
{
    const Uint8* state = SDL_GetKeyboardState(nullptr);
    for (usize i = 0; i < usize(Key::Count); ++i)
        input.held[i] = !shellOwns(Key(i)) && state[kScancodes[i]] != 0;
    input.held[usize(Key::Shift)] =
        input.held[usize(Key::Shift)] || state[SDL_SCANCODE_RSHIFT] != 0;
    input.held[usize(Key::Ctrl)] = input.held[usize(Key::Ctrl)] || state[SDL_SCANCODE_RCTRL] != 0;
}

bool keyForScancode(SDL_Scancode code, Key& out)
{
    if (code == SDL_SCANCODE_RSHIFT)
        code = SDL_SCANCODE_LSHIFT;
    if (code == SDL_SCANCODE_RCTRL)
        code = SDL_SCANCODE_LCTRL;
    for (usize i = 0; i < usize(Key::Count); ++i)
    {
        if (kScancodes[i] == code)
        {
            out = Key(i);
            return !shellOwns(out);
        }
    }
    return false;
}

float deadZone(float v)
{
    const float dead = 0.12f;
    if (v > dead)
        return (v - dead) / (1.0f - dead);
    if (v < -dead)
        return (v + dead) / (1.0f - dead);
    return 0.0f;
}

void readGamepad(SDL_GameController* pad, ControlInput& input)
{
    input.padConnected = pad != nullptr;
    if (!pad)
        return;
    const SDL_GameControllerAxis axes[usize(PadAxis::Count)] = {
        SDL_CONTROLLER_AXIS_LEFTX,       SDL_CONTROLLER_AXIS_LEFTY,
        SDL_CONTROLLER_AXIS_RIGHTX,      SDL_CONTROLLER_AXIS_RIGHTY,
        SDL_CONTROLLER_AXIS_TRIGGERLEFT, SDL_CONTROLLER_AXIS_TRIGGERRIGHT};
    for (usize i = 0; i < usize(PadAxis::Count); ++i)
        input.pad[i] = deadZone(float(SDL_GameControllerGetAxis(pad, axes[i])) / 32767.0f);
}

float wrapAngle(float a)
{
    while (a > 3.14159265f)
        a -= 6.2831853f;
    while (a < -3.14159265f)
        a += 6.2831853f;
    return a;
}

void drawHelp(Scene* scene, bool chaseOn)
{
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 10.0f, io.DisplaySize.y - 10.0f),
                            ImGuiCond_Always, ImVec2(1.0f, 1.0f));
    ImGui::SetNextWindowBgAlpha(0.75f);
    ImGui::Begin("controls", nullptr,
                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoFocusOnAppearing |
                     ImGuiWindowFlags_NoNav);
    if (ImGui::BeginTable("keys", 2, ImGuiTableFlags_SizingFixedFit))
    {
        // `length` < 0: the whole of `what`.
        auto row = [](const char* keys, const char* what, int length = -1)
        {
            const int lengthOfWhat = length < 0 ? int(strlen(what)) : length;
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextColored(ImVec4(0.95f, 0.8f, 0.4f, 1.0f), "%s", keys);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.*s", lengthOfWhat, what);
        };
        if (scene && scene->drivable())
        {
            const char* help = scene->controlsHelp();
            if (scene->manual() && help)
            {
                // "keys<TAB>meaning" per line.
                const char* line = help;
                while (*line)
                {
                    const char* end = strchr(line, '\n');
                    if (!end)
                        end = line + strlen(line);
                    const char* tab = static_cast<const char*>(memchr(line, '\t', usize(end - line)));
                    char keys[48];
                    const int keyLength = tab ? int(tab - line) : 0;
                    snprintf(keys, sizeof(keys), "%.*s", keyLength, line);
                    const char* what = tab ? tab + 1 : line;
                    row(keys, what, int(end - what));
                    line = *end ? end + 1 : end;
                }
            }
            row("P", scene->manual() ? "switch to the autopilot (restarts)"
                                     : "take the controls (manual)");
        }
        row("F2", chaseOn ? "chase camera on vehicles: on" : "chase camera on vehicles: off");
        row("F1", "hide this help");
        row("space", "pause");
        row("right drag, wheel", "orbit, zoom");
        row("esc", "quit");
        ImGui::EndTable();
    }
    ImGui::End();
}

void drawHud(Scene* scene)
{
    ct::Vector<Readout> lines;
    scene->readouts(lines);
    if (lines.size() == 0 && !scene->drivable())
        return;
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 10.0f, 10.0f), ImGuiCond_Always,
                            ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.75f);
    ImGui::Begin("HUD", nullptr,
                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoFocusOnAppearing |
                     ImGuiWindowFlags_NoNav);
    if (scene->drivable())
        ImGui::TextColored(scene->manual() ? ImVec4(0.4f, 0.9f, 1.0f, 1.0f)
                                           : ImVec4(0.9f, 0.8f, 0.3f, 1.0f),
                           "%s", scene->manual() ? "MANUAL" : "AUTOPILOT (scripted run)");
    if (ImGui::BeginTable("readouts", 2, ImGuiTableFlags_SizingFixedFit))
    {
        for (const Readout& r : lines)
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("%s", r.label);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%s", r.value);
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

} // namespace

int main(int argc, char** argv)
{
    float headlessSeconds = 0.0f;
    const char* shotPath = nullptr;
    int shotFrame = 90;
    usize startScene = 0;
    bool haveScene = false;
    bool haveTimeline = false;
    ct::Vector<TimedKey> timeline;
    float printEvery = 1.0f;
    bool startScripted = false;
    for (int i = 1; i < argc; ++i)
    {
        if (strncmp(argv[i], "--shot=", 7) == 0)
            shotPath = argv[i] + 7;
        else if (strncmp(argv[i], "--frames=", 9) == 0)
            shotFrame = atoi(argv[i] + 9);
        else if (strncmp(argv[i], "--scene=", 8) == 0)
        {
            haveScene = true;
            const char* which = argv[i] + 8;
            if (*which >= '0' && *which <= '9')
                startScene = usize(atoi(which));
            else
                startScene = findScene(which);
            if (startScene >= sceneCount())
            {
                fprintf(stderr, "no scene matches '%s'\n", which);
                return 1;
            }
        }
        else if (strncmp(argv[i], "--keys=", 7) == 0)
        {
            haveTimeline = true;
            if (!parseTimeline(argv[i] + 7, timeline))
            {
                fprintf(stderr, "--keys: cannot parse '%s'\n", argv[i] + 7);
                return 1;
            }
        }
        else if (strncmp(argv[i], "--every=", 8) == 0)
            printEvery = float(atof(argv[i] + 8));
        else if (strcmp(argv[i], "--autopilot") == 0)
            startScripted = true;

        if (strcmp(argv[i], "--headless") == 0)
            headlessSeconds = 10.0f;
        else if (strncmp(argv[i], "--seconds=", 10) == 0)
            headlessSeconds = float(atof(argv[i] + 10));
    }
    if (headlessSeconds > 0.0f && haveTimeline)
        return runDriven(startScene, timeline, headlessSeconds, printEvery);
    if (headlessSeconds > 0.0f)
        return runHeadless(headlessSeconds, startScene, haveScene);

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) != 0)
    {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_Window* window =
        SDL_CreateWindow("Kinetix testbed", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280,
                         800, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!window)
    {
        fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    SDL_GLContext gl = SDL_GL_CreateContext(window);
    SDL_GL_SetSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForOpenGL(window, gl);
    ImGui_ImplOpenGL2_Init();

    bool manualDefault = !startScripted;
    auto openScene = [&](usize index)
    {
        Scene* created = createScene(index);
        if (created && created->drivable() && manualDefault)
            created->setManual(true);
        return created;
    };

    usize current = startScene < sceneCount() ? startScene : 0;
    Scene* scene = openScene(current);
    int frameCount = 0;
    // Steps since the scene (re)started: the clock the --keys timeline runs on.
    int sceneStep = 0;
    bool previousHeld[usize(Key::Count)] = {};
    bool pendingPressed[usize(Key::Count)] = {};
    bool showHelp = true;
    bool chaseOn = true;
    float chaseYaw = 0.0f;
    bool wasChasing = false;
    SDL_GameController* pad = nullptr;
    for (int j = 0; j < SDL_NumJoysticks() && !pad; ++j)
        if (SDL_IsGameController(j))
            pad = SDL_GameControllerOpen(j);
    bool paused = false;
    bool stepOnce = false;
    float speed = 1.0f;
    float elapsed = 0.0f;
    bool dragging = false;
    bool grabHeld = false;
    // The scene supplies the starting view; after that the camera belongs to the shell.
    Camera3D camera3D = scene ? scene->camera3D() : Camera3D{};
    Camera2D camera2D = scene ? scene->camera2D() : Camera2D{};
    bool cameraFromScene = true;

    bool running = true;
    while (running)
    {
        float wheelDelta = 0.0f;
        bool grabPressed = false;
        bool grabReleased = false;
        bool firePressed = false;
        bool cutPressed = false;
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_MOUSEWHEEL && !ImGui::GetIO().WantCaptureMouse)
                wheelDelta += float(event.wheel.y);
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT)
                running = false;
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)
                running = false;
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_SPACE)
                paused = !paused;
            if (event.type == SDL_CONTROLLERDEVICEADDED && !pad)
                pad = SDL_GameControllerOpen(event.cdevice.which);
            if (event.type == SDL_CONTROLLERDEVICEREMOVED && pad &&
                SDL_GameControllerFromInstanceID(event.cdevice.which) == pad)
            {
                SDL_GameControllerClose(pad);
                pad = nullptr;
            }
            if (event.type == SDL_KEYDOWN && event.key.repeat == 0 &&
                !ImGui::GetIO().WantCaptureKeyboard)
            {
                const SDL_Keycode sym = event.key.keysym.sym;
                if (sym == SDLK_F1)
                    showHelp = !showHelp;
                else if (sym == SDLK_F2)
                    chaseOn = !chaseOn;
                else if (sym == SDLK_p && scene && scene->drivable())
                {
                    const bool toManual = !scene->manual();
                    manualDefault = toManual;
                    if (toManual)
                        scene->setManual(true);
                    else
                    {
                        delete scene;
                        scene = openScene(current);
                        elapsed = 0.0f;
                        sceneStep = 0;
                        wasChasing = false;
                        cameraFromScene = true;
                    }
                }
                Key key;
                if (keyForScancode(event.key.keysym.scancode, key))
                    pendingPressed[usize(key)] = true;
            }
            if (!ImGui::GetIO().WantCaptureMouse)
            {
                if (event.type == SDL_MOUSEBUTTONDOWN)
                {
                    if (event.button.button == SDL_BUTTON_LEFT)
                    {
                        grabPressed = true;
                        grabHeld = true;
                    }
                    else if (event.button.button == SDL_BUTTON_RIGHT)
                        dragging = true;
                    else if (event.button.button == SDL_BUTTON_MIDDLE)
                        firePressed = true;
                }
                if (event.type == SDL_MOUSEBUTTONUP)
                {
                    if (event.button.button == SDL_BUTTON_LEFT)
                    {
                        grabHeld = false;
                        grabReleased = true;
                    }
                    else if (event.button.button == SDL_BUTTON_RIGHT)
                        dragging = false;
                }
            }
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_f &&
                !ImGui::GetIO().WantCaptureKeyboard)
                firePressed = true;
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_c &&
                !ImGui::GetIO().WantCaptureKeyboard)
                cutPressed = true;
        }

        if (!dragging)
        {
            int dx = 0, dy = 0;
            SDL_GetRelativeMouseState(&dx, &dy);
        }

        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(380, 560), ImGuiCond_FirstUseEver);
        ImGui::Begin("Kinetix testbed");

        if (ImGui::BeginCombo("scene", sceneName(current)))
        {
            Category shown = Category(255);
            for (usize i = 0; i < sceneCount(); ++i)
            {
                const SceneEntry& entry = sceneAt(i);
                if (entry.category != shown)
                {
                    shown = entry.category;
                    ImGui::SeparatorText(categoryName(shown));
                }
                if (ImGui::Selectable(entry.name, i == current))
                {
                    delete scene;
                    current = i;
                    scene = openScene(current);
                    elapsed = 0.0f;
                    sceneStep = 0;
                    cameraFromScene = true;
                    wasChasing = false;
                }
            }
            ImGui::EndCombo();
        }

        if (ImGui::Button(paused ? "play" : "pause"))
            paused = !paused;
        ImGui::SameLine();
        if (ImGui::Button("step"))
        {
            paused = true;
            stepOnce = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("restart"))
        {
            delete scene;
            scene = openScene(current);
            elapsed = 0.0f;
            sceneStep = 0;
            cameraFromScene = true;
            wasChasing = false;
        }
        ImGui::SliderFloat("speed", &speed, 0.1f, 3.0f, "%.2fx");
        ImGui::Text("t = %.2fs", double(elapsed));

        if (scene && scene->overlayNames())
        {
            ImGui::SeparatorText("overlays");
            const char* const* names = scene->overlayNames();
            for (int bit = 0; names[bit]; ++bit)
            {
                bool on = (scene->debugFlags & (1u << bit)) != 0;
                if (bit > 0)
                    ImGui::SameLine();
                if (ImGui::Checkbox(names[bit], &on))
                    scene->debugFlags = on ? (scene->debugFlags | (1u << bit))
                                           : (scene->debugFlags & ~(1u << bit));
            }
        }

        if (scene)
        {
            ImGui::Separator();
            ImGui::TextWrapped("%s", scene->description());
            ImGui::Separator();

            const bool ok = scene->allPassed();
            if (scene->manual())
                ImGui::TextColored(ImVec4(0.4f, 0.9f, 1.0f, 1.0f),
                                   "MANUAL - checks judge the autopilot run (P)");
            else
                ImGui::TextColored(ok ? ImVec4(0.4f, 0.9f, 0.4f, 1.0f)
                                      : ImVec4(1.0f, 0.4f, 0.4f, 1.0f),
                                   ok ? "PASSING" : "FAILED");

            for (const Check& c : scene->checks())
            {
                ImVec4 colour = c.everFailed  ? ImVec4(1.0f, 0.4f, 0.4f, 1.0f)
                                : c.passed    ? ImVec4(0.4f, 0.9f, 0.4f, 1.0f)
                                              : ImVec4(0.9f, 0.8f, 0.3f, 1.0f);
                ImGui::TextColored(colour, "%s %s", c.everFailed ? "x" : (c.passed ? "+" : "~"),
                                   c.label.c_str());
                if (c.detail.size() > 0)
                {
                    ImGui::SameLine();
                    ImGui::TextDisabled("(%s)", c.detail.c_str());
                }
            }
        }

        ImGui::Separator();
        ImGui::TextDisabled("space = pause, left = grab, right = orbit");
        ImGui::TextDisabled("wheel = zoom, F/middle = throw, C = cut, esc = quit");
        ImGui::TextDisabled("F1 = controls help, F2 = chase camera, P = manual/autopilot");
        ImGui::End();

        if (scene && showHelp)
            drawHelp(scene, chaseOn);
        if (scene)
            drawHud(scene);

        if (scene && (!paused || stepOnce))
        {
            const int substeps = stepOnce ? 1 : int(speed / 1.0f + 0.5f);
            ControlInput keys;
            if (!ImGui::GetIO().WantCaptureKeyboard)
                readKeyboard(keys);
            readGamepad(pad, keys);
            for (int i = 0; i < (substeps < 1 ? 1 : substeps); ++i)
            {
                if (scene->manual())
                {
                    // Edges come from key-down events (so a tap shorter than a frame counts) and the timeline's steps.
                    ControlInput input = keys;
                    applyTimeline(timeline, sceneStep, input.held);
                    if (i == 0)
                        memcpy(input.pressed, pendingPressed, sizeof(pendingPressed));
                    computeEdges(input, previousHeld);
                    memcpy(previousHeld, input.held, sizeof(previousHeld));
                    scene->control(input, kFixedStep);
                    if (scene->restartRequested || scene->requestedScene)
                        break;
                }
                scene->step(kFixedStep);
                if (!scene->manual())
                    scene->evaluate();
                elapsed += kFixedStep;
                ++sceneStep;
            }
            memset(pendingPressed, 0, sizeof(pendingPressed));
            stepOnce = false;

            if (scene->restartRequested)
            {
                delete scene;
                scene = openScene(current);
                elapsed = 0.0f;
                sceneStep = 0;
                wasChasing = false;
            }
            if (scene->requestedScene)
            {
                const usize next = findScene(scene->requestedScene);
                scene->requestedScene = nullptr;
                if (next < sceneCount() && next != current)
                {
                    delete scene;
                    current = next;
                    scene = openScene(current);
                    elapsed = 0.0f;
                    sceneStep = 0;
                    cameraFromScene = true;
                    wasChasing = false;
                }
            }
        }

        int w = 0, h = 0;
        SDL_GetWindowSize(window, &w, &h);
        glClearColor(0.09f, 0.09f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (scene)
        {
            if (scene->is3D())
            {
                if (cameraFromScene)
                {
                    camera3D = scene->camera3D();
                    cameraFromScene = false;
                }
                // The target follows whatever the scene points at; orbit and zoom stay the user's.
                const Camera3D wanted = scene->camera3D();
                camera3D.target = wanted.target;
                const bool chase = wanted.chase && chaseOn;
                if (chase && !wasChasing)
                {
                    chaseYaw = wanted.yaw;
                    camera3D.yaw = 0.0f;
                }
                else if (chase)
                {
                    chaseYaw = wrapAngle(chaseYaw + wrapAngle(wanted.yaw - chaseYaw) * 0.05f);
                }
                else if (wasChasing)
                {
                    camera3D.yaw = wrapAngle(camera3D.yaw + chaseYaw);
                }
                wasChasing = chase;

                if (dragging)
                {
                    int dx = 0, dy = 0;
                    SDL_GetRelativeMouseState(&dx, &dy);
                    camera3D.yaw -= float(dx) * 0.008f;
                    camera3D.pitch += float(dy) * 0.008f;
                    // At exactly straight up the look-at basis is undefined and the view flips.
                    camera3D.pitch = camera3D.pitch > 1.5f    ? 1.5f
                                     : camera3D.pitch < -1.5f ? -1.5f
                                                              : camera3D.pitch;
                }
                camera3D.distance *= (1.0f - wheelDelta * 0.12f);
                if (camera3D.distance < 1.0f)
                    camera3D.distance = 1.0f;
                if (camera3D.distance > 400.0f)
                    camera3D.distance = 400.0f;

                Camera3D view = camera3D;
                if (chase)
                    view.yaw = chaseYaw + camera3D.yaw;
                beginFrame3D(view, w, h);

                Scene::Input input;
                int mx = 0, my = 0;
                SDL_GetMouseState(&mx, &my);
                unprojectRay3D(mx, my, w, h, input.rayOrigin, input.rayDirection);
                input.grabPressed = grabPressed;
                input.grabHeld = grabHeld;
                input.grabReleased = grabReleased;
                input.firePressed = firePressed;
                input.cutPressed = cutPressed;
                // A paused scene never steps, so grab/slice/explode would not show; only release always goes
                // through so a grabbed body is not left on a mouse joint.
                if (((!paused || stepOnce) && (!ImGui::GetIO().WantCaptureMouse || grabHeld)) ||
                    grabReleased)
                    scene->handleInput(input);
            }
            else
            {
                if (cameraFromScene)
                {
                    camera2D = scene->camera2D();
                    cameraFromScene = false;
                }
                camera2D.centerX = scene->camera2D().centerX;
                camera2D.centerY = scene->camera2D().centerY;
                camera2D.zoom *= (1.0f + wheelDelta * 0.12f);
                if (camera2D.zoom < 0.05f)
                    camera2D.zoom = 0.05f;
                beginFrame2D(camera2D, w, h);

                Scene::Input input;
                int mx = 0, my = 0;
                SDL_GetMouseState(&mx, &my);
                input.worldPoint = unprojectToPlane2D(mx, my, w, h);
                input.grabPressed = grabPressed;
                input.grabHeld = grabHeld;
                input.grabReleased = grabReleased;
                input.firePressed = firePressed;
                input.cutPressed = cutPressed;
                if (((!paused || stepOnce) && (!ImGui::GetIO().WantCaptureMouse || grabHeld)) ||
                    grabReleased)
                    scene->handleInput(input);
            }
            scene->draw();
        }

        // ImGui draws flat 2D quads; lighting and depth testing must be off.
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);

        ImGui::Render();
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);

        if (shotPath && ++frameCount == shotFrame)
        {
            writeFramebuffer(shotPath, w, h);
            running = false;
        }
    }

    delete scene;
    if (pad)
        SDL_GameControllerClose(pad);
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(gl);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
