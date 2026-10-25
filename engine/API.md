# prisma3d scripting API reference

Reference for the script API of the prisma3d engine, as bound in `libprisma3d/src/prisma3d/script/ZenBindings.cpp` (global functions, constants, value types and node methods), `ZenScripting.cpp` (script attachment and hooks) and the `bindProperties()` functions in `libprisma3d/src/prisma3d/scene/*.cpp` (node properties). Source revision: babcc97.

Scripts are written in Zen, a Python-like language (`libzen`, in `external/zenpy`). The language itself (syntax, built-in functions, the `math`, `time`, `struct`, `io`, `os`, `path` and `json` modules) is documented in `external/zenpy/docs`; section 2.8 lists what the engine registers.

Conventions used below:

- Types: `bool`, `int`, `float`, `string`, `Vec2`, `Vec3`, `Vec4`, `Quat`, `Color`, `Aabb`, `Ray`, `list of T`, and node class names. Booleans are `True` / `False`, the empty value is `None`.
- A parameter typed with a node class name accepts a node of that class (or a subclass), or `None`. A result typed with a node class is `None` when there is no node.
- A parameter typed `Vec4` also accepts a `Color` or a `Vec3` (alpha 1). A parameter typed `Vec2` or `Vec3` accepts exactly that type.
- Arguments of every bound function are positional. Parameter names are the names of the C++ parameters and are only documentation: the engine functions never read keyword arguments (the VM passes them aside and no binding looks at them), so `play(name="run")` is treated as `play()`. `name: type = value` marks an optional parameter and its default when omitted; where the default is a type's zero value (not shown by the binding), it is written out.
- A wrong number of arguments, a wrong argument type or an out-of-range integer or enumeration value raises a runtime error that names the function and the argument.

Contents: [1. Overview](#1-overview), [2. Globals](#2-globals), [3. Value types](#3-value-types), [4. Node classes](#4-node-classes), [5. Enums and constants used by node methods](#5-enums-and-constants-used-by-node-methods).

Size of the API: 55 global functions, 167 global constants, 10 value types (34 methods), 71 node classes with 718 properties and 609 methods.

## 1. Overview

### 1.1 Attaching a script

A script is a Zen source file (the editor creates `.py` files). It must define a class that extends a node class:

```python
class Spinner(Node3D):
    speed = 90.0

    def _ready(self):
        print("ready " + self.name)

    def _process(self, delta):
        self.yaw(self.speed * delta)
```

- A script is attached to a node by setting the `script` property of the node (a path to the file), from the editor, from a scene file, or with `node.script = "path/to/file.py"`. `create_child(class_name, script)` creates a node and attaches the script before the node enters the tree.
- The class chosen from the file is a class defined by the file whose ancestor chain reaches a registered node class. If several qualify, the one named like the file (without extension) is used, otherwise the first found, and a warning is logged. If none qualifies, an error is logged and nothing is attached.
- The node must be an instance of the class that the script extends (for example `class Player(Node3D)` fits a `Node3D`, a `Camera3D`, a `RigidBody3D`; it does not fit a plain `Node`). Otherwise an error is logged and nothing is attached.
- A class-level variable named `tool` with a true value marks a tool script. While the engine is in editor mode (`is_editor()` is `True`, scene editing and not playing) only tool scripts run.
- Script files are recompiled automatically when they change on disk (hot reload; on and checked every 0.5 s by default, the host application can change both). Instances are rebound to the new class: variables whose value differs from the old default keep it, the others take the new default; a script disabled by an error is enabled again.

### 1.2 The `self` object

`self` is the node itself, an instance of the engine class bound to the script class. Through it a script has:

- every property of the node and of its ancestor classes as an attribute (section 1.5 and section 4);
- every method of the node class and of its ancestor classes (section 4);
- the methods and variables defined by the script.

A node value obtained from the engine (`get_node`, `get_parent`, `raycast(...).node`, hook arguments, ...) is the same kind of object: it has the properties and methods of its engine class and, when a script is attached to it, that script's methods and variables.

### 1.3 Hooks

The engine calls these methods of the script class when they are defined. Each name may be written with or without the leading underscore (`_ready` or `ready`); the underscore form is looked up first.

| Hook | Signature | Called |
|---|---|---|
| `_enter_tree` | `_enter_tree(self)` | Each time the node enters the scene tree (a parent before its children). |
| `_ready` | `_ready(self)` | Once, the first time the node is ready: after its children are ready and after `_enter_tree`. |
| `_process` | `_process(self, delta)` | Every rendered frame; `delta` is the frame time in seconds (`float`). |
| `_physics_process` | `_physics_process(self, delta)` | Every physics step; `delta` is the step time in seconds (`float`). |
| `_input` | `_input(self, event)` | For every input event; `event` is an `InputEvent` (section 3). Nodes receive an event in reverse tree order (later nodes first); after `get_tree().set_input_as_handled()` no further node receives it. |
| `_exit_tree` | `_exit_tree(self)` | When the node leaves the scene tree. |
| `_body_entered` | `_body_entered(self, other)` | A physics body started touching this body; `other` is the other `PhysicsBody3D`. |
| `_button_pressed` | `_button_pressed(self, button)` | A `Button` was clicked (released over it after a press on it) or `press()` was called. The hook runs on the button's own script when it has one, otherwise on the nearest ancestor with a script; `button` is the `Button` node. |
| `_text_submitted` | `_text_submitted(self, field, text)` | ENTER was pressed in a focused `LineEdit`, or `submit()` was called; `text` is its text. Same target rule as `_button_pressed`. |
| `_text_changed` | `_text_changed(self, field, text)` | The text of a `LineEdit` changed by typing, deleting or `clear()`. Same target rule as `_button_pressed`. |
| `_close_requested` | `_close_requested(self)` | The window was asked to close (its close button, the window manager, Alt+F4). Every node with the hook hears it. With `get_tree().set_auto_accept_quit(False)` the game keeps running and decides (an "exit game?" popup, then `get_tree().quit()`); every later close calls the hook again. |
| `_body_exited` | `_body_exited(self, other)` | A physics body stopped touching this body; `other` is the other `PhysicsBody3D`. |

- A hook is accepted when its parameter count (not counting `self`) matches the table, or when it has default values covering the difference; a mismatch is reported when the script is loaded.
- When a script defining `_process`, `_physics_process` or `_input` becomes ready, the node's process, physics-process and input callbacks are switched on. They can be changed with `set_process`, `set_physics_process` and `set_process_input`.
- `_body_entered` and `_body_exited` need contact monitoring on the body. Defining either hook turns it on when the node becomes ready; `set_contact_monitor(enabled, max_contacts = 8)` controls it (section 4, `PhysicsBody3D`). The hooks only fire on `PhysicsBody3D` nodes.
- `__init__(self)`, defined without other parameters, is called once per script attachment (when the script is attached, or when it first can run), before any hook.
- A runtime error inside a hook or `__init__` is logged ("script error on node ... the script of this node is disabled") and disables the script of that node: none of its hooks run again until the file is changed and recompiled. Errors in methods called from other code surface in that code's context.
- While a game runs inside the editor, the keyboard, mouse and action queries (sections 2.1 and 2.2) answer as if nothing were pressed unless the Game tab has focus or the mouse is over it.

### 1.4 Exported variables

Class-level variables of the script class with a literal default of type `bool`, `int`, `float` or `string` are exported: they show in the Inspector of the node under "Script Variables", are stored in the scene and can be read and written with `get(name)` / `set(name, value)` or as attributes. Names that start with `_` are private and not exported, and a variable whose name equals a property of the node class is not exported (the property wins; a warning is logged). The `tool` variable is not exported (section 1.1).

### 1.5 Node properties as attributes

Every property listed in section 4 is readable and writable as `node.property` (for the node itself: `self.property`).

- A property name that contains `/` is accessed with `_` instead: `motor/enable` is `motor_enable`. `get(name)` and `set(name, value)` use the original name with `/`. Section 4 shows both.
- Reading returns: `Bool` as `bool`, `Int` as `int`, `Float` as `float`, `Vec2`, `Vec3`, `Vec4`, `Quat`, `Color` as the value types of section 3, and `String`, `NodePath` and `ResourcePath` as `string`. Resource properties return the path of the resource, empty when the resource has no path.
- Writing converts like this: `bool` accepts any value (truthiness); `int` and `float` accept `int`, `float` or `bool`; `Vec2` and `Vec3` need exactly that type; `Vec4` needs a `Vec4`; `Quat` needs a `Quat`; `Color` accepts a `Color`, a `Vec4`, or a `Vec3` (alpha 1); the string kinds need a `string`. Anything else is a runtime error naming the property.
- The ranges in section 4 are the editor ranges; writing from a script is not clamped by the binding (a property setter may still clamp or ignore a value).
- Properties of a class are attributes of that class and its subclasses only. At most 1024 distinct property names are exposed as attributes; beyond that only `get`/`set` reach them (a warning is logged; 629 distinct names are exposed at present).
- Rotation properties named `rotation` are in radians; `rotation_degrees` and `global_rotation_degrees` are in degrees. Properties flagged "value in radians" below are stored in radians while the editor shows degrees.

## 2. Globals

Global functions are available in every script without import. Sections 2.1 to 2.5 list the engine functions (47); section 2.7 lists the constants; section 2.8 lists what the language itself provides.

### 2.1 Keyboard and mouse

| Function | Description |
|---|---|
| `keyboard_check(key: int) -> bool` | `True` while the key is held. `KEY_ANY` tests every key. |
| `keyboard_check_pressed(key: int) -> bool` | `True` only on the frame the key went down. `KEY_ANY` tests every key. |
| `keyboard_check_released(key: int) -> bool` | `True` only on the frame the key was released. `KEY_ANY` tests every key. |
| `mouse_check_button(button: int) -> bool` | `True` while the mouse button is held. `MOUSE_ANY` tests every button; ids outside 0..7 give `False`. |
| `mouse_check_button_pressed(button: int) -> bool` | `True` only on the frame the button went down. |
| `mouse_check_button_released(button: int) -> bool` | `True` only on the frame the button was released. |
| `mouse_x() -> float` | Mouse position, x component, in the coordinates of the mouse events. |
| `mouse_y() -> float` | Mouse position, y component. |
| `mouse_position() -> Vec2` | Mouse position. |
| `mouse_delta() -> Vec2` | Mouse movement accumulated during the current frame. |
| `mouse_wheel() -> float` | Vertical wheel movement accumulated during the current frame. |
| `mouse_velocity() -> Vec2` | Mouse velocity in pixels per second, estimated from the motion events of the last fraction of a second (motion accumulated divided by the time accumulated). |
| `mouse_set_mode(mode: int)` | Sets the cursor mode: `MOUSE_MODE_VISIBLE`, `MOUSE_MODE_HIDDEN` or `MOUSE_MODE_CAPTURED`. Any other value is a runtime error. |
| `mouse_get_mode() -> int` | Current cursor mode (a `MOUSE_MODE_*` value). |

Key and button codes are the `KEY_*` and `MOUSE_*` constants of section 2.7; any other integer is accepted and is converted to the engine key or button of that number.

### 2.2 Actions

| Function | Description |
|---|---|
| `action_check(action: string) -> bool` | `True` while the action is held. An unknown action logs an error and gives `False`. |
| `action_check_pressed(action: string) -> bool` | `True` only on the frame the action became pressed. |
| `action_check_released(action: string) -> bool` | `True` only on the frame the action was released. |
| `action_strength(action: string) -> float` | Strength of the action (analog inputs give intermediate values). |
| `action_axis(negative: string, positive: string) -> float` | Axis value from two actions: positive strength minus negative strength. |
| `action_vector(negative_x: string, positive_x: string, negative_y: string, positive_y: string, deadzone: float = -1.0) -> Vec2` | Vector from four actions (x from the first two, y from the last two), limited to length 1. A negative `deadzone` uses the mean of the deadzones of the four actions. |
| `action_press(action: string, strength: float = 1.0)` | Presses the action from code with the given strength (clamped to 0..1). An unknown action logs an error. |
| `action_release(action: string)` | Releases the action from code: its pressed state and strength are cleared. An unknown action logs an error. |
| `action_exists(action: string) -> bool` | `True` when the action is defined. |
| `action_add(action: string, deadzone: float = 0.2)` | Defines an action (default deadzone 0.2). |
| `action_remove(action: string)` | Removes an action and its events. |
| `action_add_key(action: string, key: int)` | Binds a key (a `KEY_*` value) to an existing action. Runtime error when the action does not exist. |
| `action_add_mouse_button(action: string, button: int)` | Binds a mouse button (a `MOUSE_*` value) to an existing action. Runtime error when the action does not exist. |
| `action_clear(action: string)` | Removes every event bound to the action, keeping the action. |

Actions are the entries of the input map. An action is held, pressed or released according to the keys, mouse buttons and other events bound to it.

### 2.3 Time and engine

| Function | Description |
|---|---|
| `delta_time() -> float` | Seconds of the current frame (the scene tree process delta); 0 when there is no scene tree. |
| `physics_delta_time() -> float` | Seconds of the current physics step; 0 when there is no scene tree. |
| `current_time() -> int` | Milliseconds since the platform timer started. |
| `current_time_usec() -> int` | Microseconds since the platform timer started. |
| `frame_count() -> int` | Number of frames processed by the scene tree. |
| `fps() -> float` | Frames per second measured by the engine. |
| `time_scale() -> float` | Engine time scale. |
| `set_time_scale(scale: float)` | Sets the engine time scale. |
| `physics_interpolation_fraction() -> float` | How far the current frame is between the last physics step and the next one (0..1). A node moved in `_physics_process` can be drawn at `lerp(previous, current, fraction)` so it moves smoothly when the frame rate is above the physics rate, or in slow motion. |
| `set_bus_pitch(bus: int (AUDIO_BUS_*), pitch: float)` | Plays every sound of an audio bus faster or slower (0.01 .. 4, 1 is normal); the sounds go up or down in pitch with it. Lowering the sfx bus with the time scale gives a slow-motion sound while sounds on the music bus play unchanged. |
| `bus_pitch(bus: int (AUDIO_BUS_*)) -> float` | The pitch of an audio bus. |
| `is_editor() -> bool` | `True` while the engine is in editor mode (editing, not playing). Scripts that are not tool scripts do not run in that mode. |
| `window_set_fullscreen(on: bool)` | Switches the window between fullscreen and windowed. The project setting `display/window/size/mode` gives the mode at start. |
| `window_is_fullscreen() -> bool` | `True` in fullscreen or exclusive fullscreen. |
| `user_dir() -> str` | The folder where the game may write files (settings, saves): one per user and per project name, created at start, with no trailing slash; empty when the engine runs without a project. Engine paths accept `user://name` for a file in it. |

### 2.4 Physics queries

| Function | Description |
|---|---|
| `raycast(from: Vec3, to: Vec3, mask: int = None, exclude: PhysicsBody3D = None) -> RayHit` | Casts a ray between two global points and returns the hit as a `RayHit`, or `None` when nothing is hit (or when `from` equals `to`). `mask` is a collision mask (0..4294967295); `None` or omitted uses all layers. `exclude` is a physics body to ignore (runtime error if it is not a `PhysicsBody3D`). |
| `overlap_sphere(center: Vec3, radius: float, mask: int = None) -> list of PhysicsBody3D` | Physics bodies overlapping a sphere. A negative radius is a runtime error. `mask` as in `raycast`. |
| `overlap_box(center: Vec3, half_extents: Vec3, mask: int = None) -> list of PhysicsBody3D` | Physics bodies overlapping an axis-aligned box given by its center and half extents (none negative). `mask` as in `raycast`. |

### 2.5 Math helpers

| Function | Description |
|---|---|
| `is_valid(value) -> bool` | `True` when `value` is an object that still has its native counterpart (for example a node that has not been freed); `False` for `None`, numbers and freed nodes. |
| `move_toward(from: float, to: float, step: float) -> float` | Moves `from` toward `to` by at most `step`, without overshooting. |
| `quat_slerp(from: Quat, to: Quat, t: float) -> Quat` | Spherical interpolation between two quaternions. |
| `quat_lerp(from: Quat, to: Quat, t: float) -> Quat` | Linear interpolation between two quaternions. |
| `quat_normalized(q: Quat) -> Quat` | Normalized copy (safe for a zero-length quaternion). |
| `quat_length(q: Quat) -> float` | Length of a quaternion. |
| `quat_dot(a: Quat, b: Quat) -> float` | Dot product of two quaternions. |

### 2.6 Scene tree access

There is no global node lookup function. A script reaches the scene through its own node:

- `self.get_tree()` returns the `SceneTree` (section 3.7): `get_root()`, `get_current_scene()`, `get_nodes_in_group(group)`, `get_first_node_in_group(group)`, `change_scene(path)`, `quit()`, `set_paused(paused)`.
- `self.get_node(path)`, `self.get_parent()`, `self.get_children()`, `self.get_child(index)`, `self.find_child(name)` and `self.instantiate(path)` move around the tree (section 4, `Node`).
- `raycast(...)`, `overlap_sphere(...)` and `overlap_box(...)` return nodes found by the physics world (section 2.4).

### 2.7 Constants

All constants are integers.

**Keys.** Used with `keyboard_check*`, `action_add_key` and `event.key`.

| Constant | Value |
|---|---|
| `KEY_A` .. `KEY_Z` | 65 .. 90 (the ASCII code of the letter) |
| `KEY_0` .. `KEY_9` | 48 .. 57 (the ASCII code of the digit) |
| `KEY_F1` .. `KEY_F12` | 290 .. 301 |
| `KEY_SPACE` | 32 |
| `KEY_APOSTROPHE` | 39 |
| `KEY_COMMA` | 44 |
| `KEY_MINUS` | 45 |
| `KEY_PERIOD` | 46 |
| `KEY_SLASH` | 47 |
| `KEY_SEMICOLON` | 59 |
| `KEY_EQUAL` | 61 |
| `KEY_LEFT_BRACKET` | 91 |
| `KEY_BACKSLASH` | 92 |
| `KEY_RIGHT_BRACKET` | 93 |
| `KEY_GRAVE` | 96 |
| `KEY_ESCAPE` | 256 |
| `KEY_ENTER` | 257 |
| `KEY_TAB` | 258 |
| `KEY_BACKSPACE` | 259 |
| `KEY_INSERT` | 260 |
| `KEY_DELETE` | 261 |
| `KEY_RIGHT` | 262 |
| `KEY_LEFT` | 263 |
| `KEY_DOWN` | 264 |
| `KEY_UP` | 265 |
| `KEY_PAGE_UP` | 266 |
| `KEY_PAGE_DOWN` | 267 |
| `KEY_HOME` | 268 |
| `KEY_END` | 269 |
| `KEY_CAPS_LOCK` | 280 |
| `KEY_PAUSE` | 283 |
| `KEY_KP_ENTER` | 335 |
| `KEY_LEFT_SHIFT` | 340 |
| `KEY_LEFT_CONTROL` | 341 |
| `KEY_LEFT_ALT` | 342 |
| `KEY_RIGHT_SHIFT` | 344 |
| `KEY_RIGHT_CONTROL` | 345 |
| `KEY_RIGHT_ALT` | 346 |
| `KEY_ANY` | -1 (matches every key in `keyboard_check*`) |

**Mouse buttons.** Used with `mouse_check_button*`, `action_add_mouse_button` and `event.button`.

| Constant | Value |
|---|---|
| `MOUSE_LEFT` | 0 |
| `MOUSE_RIGHT` | 1 |
| `MOUSE_MIDDLE` | 2 |
| `MOUSE_ANY` | -1 (matches every button in `mouse_check_button*`) |

**Mouse modes.** Used with `mouse_set_mode` and `mouse_get_mode`.

| Constant | Value |
|---|---|
| `MOUSE_MODE_VISIBLE` | 0 |
| `MOUSE_MODE_HIDDEN` | 1 |
| `MOUSE_MODE_CAPTURED` | 2 |

**Input event types.** Values of `event.type`.

| Constant | Value |
|---|---|
| `EVENT_KEY` | 1 |
| `EVENT_CHAR` | 2 |
| `EVENT_MOUSE_MOTION` | 3 |
| `EVENT_MOUSE_BUTTON` | 4 |
| `EVENT_MOUSE_WHEEL` | 5 |
| `EVENT_TOUCH` | 6 |
| `EVENT_ACTION` | 7 |

The constants for node methods (`BODY_MODE_*`, `GROUND_STATE_*`, `LIGHT_TYPE_*`, `EMITTER_STATE_*`, `TOPOLOGY_*`, `ALPHA_ANTIALIASING_*`, `RAGDOLL_PART_*`, `RAGDOLL_BONE_*`, `BONE_DIRECTION_*`, `ROTATION_AXIS_*`, `CENTER_FROM_*`, `MODIFIER_CALLBACK_MODE_*`) are in section 5.

### 2.8 Language globals and modules

Every node class of section 4 is also a global class, so scripts extend them (`class Player(CharacterBody3D):`).

The engine registers the Zen base library as globals and these modules for `import`: `math`, `time`, `struct`, `io`, `os`, `path`, `json` (`numpy`, `http` and `net` are not registered). For the base functions, the `Signal` and `File` classes and the modules other than `math`, see the zen language reference (`external/zenpy/docs/zenpy_base.md` and the other `zenpy_*.md`).

Base functions registered in the library table (`builtin_base.cpp`): `Vec2`, `Vec3`, `str`, `repr`, `abs`, `min`, `max`, `round`, `sum`, `sorted`, `reversed`, `any`, `all`, `list`, `tuple`, `set`, `dict`, `bool`, `chr`, `hex`, `oct`, `bin`, `divmod`, `pow`, `callable`, `getattr`, `hasattr`, `int`, `float`, `char`, `ord`, `typeof`, `isNil`, `isBool`, `isInt`, `isFloat`, `isNumber`, `isString`, `isArray`, `isMap`, `isFunction`, `input`, `assert`, `error`, `range`, `type`, `isinstance`, `format`, `collect`, `gc_pause`, `gc_resume`, `mem_used`, `mem_info`, `gc_tune`, `gc_stats`, `clock`, `enumerate`, `zip`, `map`, `filter`, `open`, `Int8Array`, `Int16Array`, `Int32Array`, `Uint8Array`, `Uint16Array`, `Uint32Array`, `Float32Array`, `Float64Array`. `print` is a statement of the language. Output of `print` goes to the host print handler when one is set, otherwise to the engine log.

**`import math`** (module `math`, confirmed in `builtin_math.cpp`): 31 functions and 5 constants.

| Name | Arguments | Notes |
|---|---|---|
| `sin, cos, tan` | x | Angle in radians (`builtin_math.cpp` calls the C library functions directly; use `math.radians(a)` for degrees). |
| `asin, acos, atan` | x | Result in radians. |
| `atan2` | y, x | Result in radians. |
| `sqrt, cbrt` | x |  |
| `exp, log, log2, log10` | x |  |
| `ceil, floor, round` | x | Return floats. |
| `pow` | base, exponent |  |
| `abs` | x | Absolute value; not defined for vectors. |
| `min, max` | a, b | Two arguments; return floats. |
| `clamp` | value, minimum, maximum | Returns a float. |
| `lerp` | a, b, t | a + (b - a) * t. |
| `hermite` | value1, tangent1, value2, tangent2, t |  |
| `smooth_step` | a, b, t | t is clamped to 0..1. |
| `ping_pong` | t, length | Oscillates between 0 and length. |
| `angle_delta` | from, to | Shortest signed difference in degrees (-180..180). |
| `lerp_angle` | current, target, t | Interpolates along the shortest path, angles in degrees. |
| `random` | (), (max), (min, max) | Random float in [0, 1), [0, max) or [min, max). |
| `radians, degrees` | x | Convert between degrees and radians. |
| `hypot` | a, b | sqrt(a*a + b*b). |
| `isnan, isinf` | x | Return bool. |

Constants: `math.pi`, `math.e`, `math.tau`, `math.inf`, `math.nan`. `sin`, `cos`, `tan`, `asin`, `acos`, `atan`, `atan2`, `sqrt`, `cbrt`, `exp`, `log`, `log2`, `log10`, `ceil`, `floor`, `pow` and `abs` raise a runtime error when given a vector.

### 2.9 HUD (2D overlay)

Text and shapes drawn over the rendered frame, in pixels, with the origin at the top-left corner of the game view. In the runner the view is the whole window; while a game runs inside the editor it is the Game tab, and everything is clipped to it. The calls take effect only during a frame (from `_process`, `_physics_process`, `_input` or methods they call); what is drawn lasts for that frame, so a HUD is redrawn every frame. Colours are a `Color`, a `Vec4` or a `Vec3` (alpha 1), components 0..1; when the colour is omitted the drawing is white. `size` is the font size in pixels; 0 or omitted uses the GUI theme size.

| Function | Description |
|---|---|
| `hud_text(x: float, y: float, text: string, color = white, size: float = 0)` | Draws `text` with its top-left corner at (`x`, `y`). |
| `hud_text_size(text: string, size: float = 0) -> Vec2` | Width and height in pixels that `hud_text` would use for `text`. |
| `hud_rect(x: float, y: float, width: float, height: float, color = white)` | Filled rectangle. |
| `hud_rect_outline(x: float, y: float, width: float, height: float, color = white, thickness: float = 1)` | Rectangle outline. |
| `hud_line(x0: float, y0: float, x1: float, y1: float, color = white, thickness: float = 1)` | Line segment. |
| `hud_circle(x: float, y: float, radius: float, color = white)` | Filled circle. |
| `hud_image(image: string or int, x: float, y: float, width: float, height: float, degrees: float = 0, u0 = 0, v0 = 0, u1 = 1, v1 = 1, color = white)` | The image at `path` (loaded once and kept) in the rectangle, turned by `degrees` clockwise about its centre; `u0, v0, u1, v1` pick a region of the image (all four or none), and the colour tints it. |
| `hud_load_image(path: string) -> int` | Loads the image at `path` once and returns its id (-1 when it cannot be loaded). Pass the id to `hud_image` instead of the path, so a HUD drawn every frame does not look the file up by name on each call: load it in `_ready` and keep the id in a variable. |
| `hud_width() -> float` | Width of the game view in pixels. |
| `hud_height() -> float` | Height of the game view in pixels. |

```python
class Hud(Node3D):
    score = 0

    def _process(self, delta):
        text = str(self.score)
        size = hud_text_size(text, 48.0)
        hud_text((hud_width() - size.x) * 0.5, 20.0, text, Color(1, 1, 1, 1), 48.0)
        hud_rect(10.0, 10.0, 200.0, 8.0, Color(0, 0, 0, 0.5))
```

### 2.10 Tweens

`create_tween() -> Tween` makes a tween owned by the scene tree; it starts on the next frame and is freed when it finishes, unless it loops. Its steps run one after the other; `parallel()` makes the next step run with the previous one, `set_parallel(True)` makes every following step parallel, `chain()` goes back to sequential.

| Method | Description |
|---|---|
| `tween_property(node, name: str, final, seconds: float) -> PropertyTweener` | Moves the property `name` of `node` from its current value to `final` over `seconds`. Works on numbers, `Vec2`, `Vec3` and colours; the value must match the property. The tween is bound to the first node given and is killed if that node is freed. |
| `tween_interval(seconds: float) -> Tween` | Waits. |
| `tween_callback(fn) -> Tween` | Calls `fn()` when the step is reached (a method of `self` or a function). |
| `set_trans(TRANS_*)`, `set_ease(EASE_*)` | Default transition and easing of the following steps. |
| `set_loops(count: int = 0)` | Repeats the whole tween; 0 means forever. |
| `set_speed_scale(scale: float)`, `bind_node(node)`, `stop()`, `pause()`, `play()`, `kill()`, `is_running() -> bool`, `is_valid() -> bool` | As named. A killed or finished tween raises an error when used again; `is_valid()` tells. |

`PropertyTweener` (what `tween_property` returns): `set_trans(TRANS_*)`, `set_ease(EASE_*)`, `set_delay(seconds)`, `from_value(value)` (`from` is a keyword of the language), `from_current()`, `as_relative()`; each returns the tweener, so the calls chain.

Transitions: `TRANS_LINEAR`, `TRANS_SINE`, `TRANS_QUINT`, `TRANS_QUART`, `TRANS_QUAD`, `TRANS_EXPO`, `TRANS_ELASTIC`, `TRANS_CUBIC`, `TRANS_CIRC`, `TRANS_BOUNCE`, `TRANS_BACK`, `TRANS_SPRING`. Easings: `EASE_IN`, `EASE_OUT`, `EASE_IN_OUT`, `EASE_OUT_IN`.

```
tween = create_tween()
tween.tween_property(panel, "position", Vec2(366.0, 164.0), 0.6).from_value(Vec2(-420.0, 164.0)).set_trans(TRANS_BACK).set_ease(EASE_OUT)
tween.parallel().tween_property(panel, "modulate", Color(1.0, 1.0, 1.0, 1.0), 0.4).from_value(Color(1.0, 1.0, 1.0, 0.0))
tween.tween_callback(self.on_shown)
```

## 3. Value types

Values of these types are passed to and returned from the functions and properties of the other sections. `Vec2`, `Vec3` and lists come from the language; `Vec4`, `Quat` and `Color` are native structs registered by the engine; `Aabb`, `Ray`, `RayHit`, `InputEvent` and `SceneTree` are native classes that scripts receive but cannot construct.

### 3.1 Vec2 and Vec3

Constructors: `Vec2()` and `Vec3()` give the zero vector; `Vec2(x, y)` and `Vec3(x, y, z)` take numbers (integers are converted to floats). Any other argument count is a runtime error.

Fields: `x`, `y` (and `z` on `Vec3`), read as `float`. `v.x = value` rewrites the component in the variable that holds the vector (vectors are values, not shared objects). Component assignment through a property, such as `self.position.x += 1.0`, writes the modified vector back to the property.

Operators: `a + b` and `a - b` (same kind), `a * b` (component-wise, same kind), `a * s` and `s * a` (scalar `s`), `a / s` (scalar; division by zero is a runtime error), `-a`. Ordering comparisons (`<`, `>`) are not defined for vectors.

Methods (none mutates the receiver):

- `length() -> float`
- `length_sq() -> float`
- `normalized() -> Vec2 | Vec3` (the zero vector stays zero)
- `dot(other) -> float` (`other` of the same kind)
- `cross(other) -> Vec3 | float` (`Vec3.cross` is the vector product and returns a `Vec3`; `Vec2.cross` is the 2D cross product `x1*y2 - y1*x2` and returns a `float`)
- `distance(other) -> float`
- `distance_sq(other) -> float`
- `lerp(other, t: float) -> Vec2 | Vec3`
- `abs() -> Vec2 | Vec3` (component-wise absolute value)
- `with_x(x: float)`, `with_y(y: float)` -> copy with that component replaced
- `with_z(z: float) -> Vec3` (`Vec2` has no z: runtime error)
- `to_list() -> list of float` (2 or 3 elements)

### 3.2 Vec4, Quat, Color

Native structs of four `float` fields. Constructors take up to four numbers; arguments left out take the default shown.

| Type | Constructor | Fields | Notes |
|---|---|---|---|
| `Vec4` | `Vec4(x = 0, y = 0, z = 0, w = 0)` | `x`, `y`, `z`, `w` | |
| `Quat` | `Quat(x = 0, y = 0, z = 0, w = 1)` | `x`, `y`, `z`, `w` | `Quat()` is the identity rotation. Used by `quaternion` and bone pose rotations. |
| `Color` | `Color(r = 0, g = 0, b = 0, a = 1)` | `r`, `g`, `b`, `a` | |

They have no methods of their own. For `Quat` use `quat_slerp`, `quat_lerp`, `quat_normalized`, `quat_length` and `quat_dot` (section 2.5). Where a function or property takes a `Color`, a `Vec4` or a `Vec3` (alpha 1) is accepted as well.

### 3.3 Aabb

Axis-aligned box returned by the `get_aabb()` methods. Fields: `min: Vec3`, `max: Vec3`.

### 3.4 Ray

Returned by `Camera3D.project_ray` and `Camera3D.project_mouse_ray`. Fields: `origin: Vec3`, `direction: Vec3`.

### 3.5 RayHit

Returned by `raycast(...)`. Fields:

| Field | Type | Meaning |
|---|---|---|
| `node` | `PhysicsBody3D` | The body that was hit; `None` when the body has no node. |
| `position` | `Vec3` | Global hit position. |
| `normal` | `Vec3` | Surface normal at the hit. |
| `distance` | `float` | Distance from the ray origin to the hit. |
| `triangle` | `int` | Index of the triangle hit when the body's shape is a concave mesh (a model imported with `physics=concave`, or a `CollisionShape3D` set from a mesh); -1 for every other shape. |
| `surface` | `int` | Surface of the mesh that triangle came from; -1 when there is no triangle. |
| `material` | `string` | Name of that surface's material (for a glTF level, the material name of the primitive), so a shot can pick the hole decal, sound and particles of wood, metal or glass; empty when unknown. |

### 3.6 InputEvent

Passed to `_input(self, event)`. Which fields are meaningful depends on `event.type` (compare with the `EVENT_*` constants, section 2.7); the others hold their zero value.

| Field | Type | Set for | Meaning |
|---|---|---|---|
| `type` | `int` | all | `EVENT_KEY`, `EVENT_CHAR`, `EVENT_MOUSE_MOTION`, `EVENT_MOUSE_BUTTON`, `EVENT_MOUSE_WHEEL`, `EVENT_TOUCH` or `EVENT_ACTION`. |
| `pressed` | `bool` | key, mouse button, touch, action | Pressed state (for touch: finger down or moving). |
| `echo` | `bool` | key | Key repeat. |
| `key` | `int` | key | Key code (compare with `KEY_*`). |
| `scancode` | `int` | key | Physical key code reported by the platform layer, independent of the keyboard layout; 0 for events the engine makes itself. |
| `modifiers` | `int` | key, mouse button | Bit mask: 1 Shift, 2 Control, 4 Alt, 8 Super, 16 Caps Lock, 32 Num Lock. |
| `position` | `Vec2` | mouse motion, mouse button, mouse wheel, touch | Pointer position. |
| `relative` | `Vec2` | mouse motion | Movement since the previous motion event. |
| `button` | `int` | mouse button | Button (compare with `MOUSE_*`; 3 and 4 are the two extra buttons). |
| `button_mask` | `int` | mouse motion | Bit mask of the held buttons (bit n is button n). |
| `wheel` | `Vec2` | mouse wheel | Wheel movement (x horizontal, y vertical). |
| `codepoint` | `int` | char | Unicode code point typed. |
| `touch_id` | `int` | touch | Finger id. |
| `pressure` | `float` | touch | Touch pressure. |
| `action` | `string` | action | Action name. |
| `strength` | `float` | action | Action strength. |

Methods, each taking an action name and testing whether this event matches the action in the input map:

- `is_action(action: string) -> bool`
- `is_action_pressed(action: string) -> bool` (matches and is a press)
- `is_action_released(action: string) -> bool` (matches and is a release)
- `get_action_strength(action: string) -> float` (0 when the event does not match)

### 3.7 SceneTree

Returned by `get_tree()`. It is the only way to reach tree-wide operations from a script.

- `get_nodes_in_group(group: string) -> list of Node`
- `get_first_node_in_group(group: string) -> Node`
- `has_group(group: string) -> bool`
- `get_node_count_in_group(group: string) -> int`
- `change_scene(path: string, transition: string = "", seconds: float = 0.5)` and `change_scene_to_file(...)` (alias): changes to a scene file at the end of the frame. A relative path is resolved against the open project when the file exists there. With a transition name the last frame of the old scene is kept as an image and drawn over the new scene for `seconds`, animated: `fade`, `slide_left`, `slide_right`, `slide_up`, `slide_down`, `zoom_in`, `zoom_out`, `wipe_left`, `wipe_right`. The new scene runs underneath from its first frame.
- `reload_current_scene() -> bool`
- `quit(code: int = 0)`
- `set_paused(paused: bool)`
- `is_paused() -> bool`
- `set_auto_accept_quit(enabled: bool)`: with `False` closing the window does not end the game; the `_close_requested` hook runs and the script quits when it wants. `True` by default.
- `is_auto_accept_quit() -> bool`
- `get_root() -> Node`
- `get_current_scene() -> Node`
- `get_frame() -> int`
- `get_node_count() -> int`
- `set_input_as_handled()`: stops the current input event from reaching further nodes.
- `get_process_delta_time() -> float`
- `get_physics_process_delta_time() -> float`

## 4. Node classes

Every class below is a node class. `Class.property` is an attribute of nodes of that class and its subclasses; methods and properties of ancestor classes are not repeated. A subclass inherits all of them (for example every `RigidBody3D` also has all `PhysicsBody3D`, `Node3D` and `Node` members). Property tables list the script attribute name, its type, the editor range or the enumeration values, and the default when it is visible in the code (empty when it is not). Properties marked "not stored in scene files" are runtime-only. Group names group the properties; they are not attributes. Enumerations are integers: the value written is the number before `=`. In the range column, `a .. b` is the editor slider range; "no upper bound", "no lower bound" and "unbounded" mean the editor accepts values beyond that end (script writes are never clamped by the binding). A step is the editor increment.

Totals for this section: **65** classes, **703** properties and **531** methods bound on node classes (the methods of `Vec2`/`Vec3`, `InputEvent` and `SceneTree` in section 3 are not included).

### 4.0 Class hierarchy

- `Node`
  - `MissingNode` (abstract)
  - `Control`
    - `Label`
    - `ColorRect`
    - `TextureRect`
    - `Button`
    - `LineEdit`
  - `Node3D`
    - `VisualInstance3D` (abstract)
      - `MeshInstance3D`
        - `SoftBody3D`
        - `DebrisMesh3D`
      - `ManualMesh3D`
      - `MultiMeshInstance3D`
      - `ParticleSystem3D`
      - `RibbonTrail3D`
      - `Hair3D`
      - `Tree3D`
      - `Road3D`
      - `Ocean3D`
      - `Water3D`
    - `Camera3D`
    - `Light3D` (abstract)
      - `DirectionalLight3D`
      - `OmniLight3D`
      - `SpotLight3D`
    - `Skeleton3D`
    - `BoneAttachment3D`
    - `SkeletonModifier3D` (abstract)
      - `SpringBoneSimulator3D`
    - `SpringBoneCollision3D` (abstract)
      - `SpringBoneCollisionSphere3D`
      - `SpringBoneCollisionCapsule3D`
      - `SpringBoneCollisionPlane3D`
    - `Ragdoll3D`
    - `Dismember3D`
    - `NavigationRegion3D`
    - `CarDamage3D`
    - `SkidMarks3D`
    - `ImpactFx3D`
    - `VehicleDriver3D`
    - `BoostJet3D`
    - `Traffic3D`
    - `Decal3D`
    - `ReflectionProbe3D`
    - `LightmapGI`
    - `PlanarReflector3D`
    - `AudioListener3D`
    - `AudioStreamPlayer3D`
    - `Path3D`
    - `PathFollow3D`
    - `Terrain3D`
    - `Grass3D`
    - `Forest3D`
    - `ParticleCollision3D` (abstract)
      - `ParticleCollisionSphere3D`
      - `ParticleCollisionBox3D`
    - `CollisionShape3D`
    - `PhysicsBody3D` (abstract)
      - `StaticBody3D`
      - `AnimatableBody3D`
      - `RigidBody3D`
        - `CharacterRigidBody3D`
        - `GroundVehicle3D`
        - `VehicleBody3D`
        - `DestructibleBody3D`
        - `DebrisBody3D`
      - `CharacterBody3D`
    - `VehicleWheel3D`
    - `Joint3D` (abstract)
      - `PinJoint3D`
      - `HingeJoint3D`
      - `SliderJoint3D`
      - `DistanceJoint3D`
      - `FixedJoint3D`
      - `PistonJoint3D`
      - `UniversalJoint3D`
      - `WheelJoint3D`
      - `ConeTwistJoint3D`
      - `MouseJoint3D`
  - `Viewport`
    - `Window`
  - `WorldEnvironment`
  - `AnimationPlayer`
  - `AudioStreamPlayer`

Abstract classes cannot be created with `create_child`; their members are inherited by the subclasses.

### 4.1 Base nodes

#### Node

Root of the node hierarchy.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `name` | string | not stored in scene files |  |
| `process_mode` | int | 0=Inherit, 1=Pausable, 2=When Paused, 3=Always, 4=Disabled | 0 (Inherit) |
| `script` | ResourcePath (string) | file path, extension py |  |

Methods:

- `get_name()` -> `string`
- `set_name(name: string)`
- `get_parent()` -> `Node` - None when the node has no parent.
- `get_child_count()` -> `int`
- `get_child(index: int)` -> `Node` - None when the index is negative or out of range.
- `get_children()` -> `list of Node`
- `get_index()` -> `int` - Position among the parent children.
- `get_node(path: string)` -> `Node` - Path of child names separated by `/`; `..` is the parent, `.` the node itself and a leading `/` starts at the tree root. None when not found.
- `find_child(name: string, recursive: bool = True)` -> `Node` - Direct children first, then the descendants in depth-first order when recursive; None when not found.
- `create_child(class_name: string, script: string = None)` -> `Node` - Creates a node of a registered, instantiable class and adds it as a child. The optional script path is attached before the node enters the tree. Runtime error when the class is not a node class or is abstract.
- `instantiate(path: string)` -> `Node` - Instantiates a scene file as a child of this node and returns its root. The path is resolved against the open project when the file exists there. On failure a message is logged and None is returned. A model file (glb, gltf, fbx, b3d, ms3d, pmesh, obj, 3ds, mesh) is accepted as well, in scripts and in the `instance` entries of scene files: its nodes become the instance, with the options `?scale=0.01` (import scale applied to meshes, nodes, skeleton and animations) and `physics=concave` or `physics=convex` (each mesh becomes a `StaticBody3D` with a `CollisionShape3D`, or an `AnimatableBody3D` with `body=kinematic` for a platform or door moved by a script), for example `models/level.b3d?scale=0.01&physics=concave`. `collide=road_,wall_` keeps the physics to the meshes whose node names start with one of those prefixes (the rest are drawn only), and `hide=route_,debug_` leaves out the nodes whose names start with one of them, with everything under them. A B3D whose brushes carry a second texture flagged for the second UV set (a Blitz3D lightmap) gets a `LightmapGI` child with those images packed into one texture array and every lightmapped mesh assigned to it (when another lightmapped model is already in the same tree with layers at least as large, the new lightmap joins that model's atlas as blocks in extra layers, so a level and its movers count as one lightmap); `lightmap=2.0` scales that light and `lightmap=0` ignores it. The same import builds a grid of light probes from the lightmapped surfaces, and every dynamic object inside the model bounds (enemies, props, debris, the weapon in hand) is lit by the probe nearest to it, so it matches the baked light around it. A `.seq` file beside a `.b3d` (lines `Name first last`) splits its animation into named clips.
- `queue_free()`
- `is_queued_for_deletion()` -> `bool`
- `add_to_group(group: string)`
- `remove_from_group(group: string)`
- `is_in_group(group: string)` -> `bool`
- `get_groups()` -> `list of string`
- `get_tree()` -> `SceneTree`
- `is_inside_tree()` -> `bool`
- `is_ready()` -> `bool`
- `set_process(enabled: bool)`
- `is_processing()` -> `bool`
- `set_physics_process(enabled: bool)`
- `is_physics_processing()` -> `bool`
- `set_process_input(enabled: bool)`
- `is_processing_input()` -> `bool`
- `can_process()` -> `bool`
- `get_path()` -> `string`
- `get_class()` -> `string` - Class name, for example "Camera3D".
- `is_class(name: string)` -> `bool` - True for the class itself and its ancestors.
- `reparent(parent: Node)` -> `bool` - Runtime error when the argument is not a live node.
- `get_script()` -> `string` - Path of the attached script, empty when none.
- `get(name: string)` -> `value` - Reads a property by its engine name (the form with `/`, not `_`). Also reads exported script variables. Runtime error when the property does not exist.
- `set(name: string, value: value)` - Writes a property by its engine name, with the same type rules as attribute assignment. Also writes exported script variables. Runtime error when the property does not exist or the value has the wrong type.
- `move_child(child: Node, to_index: int)`
- `get_path_to(node: Node)` -> `string`
- `is_ancestor_of(node: Node)` -> `bool`
- `get_owner()` -> `Node`
- `is_instance()` -> `bool`
- `get_scene_file_path()` -> `string`
- `is_enabled()` -> `bool`
- `get_process_delta_time()` -> `float`
- `get_physics_process_delta_time()` -> `float`

#### MissingNode

Inherits `Node`. Abstract. Placeholder the scene loader creates for a node whose class is not registered; it keeps the original class name and scene.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `original_class` | string | not stored in scene files |  |
| `original_scene` | string | not stored in scene files |  |

No methods of its own.

### 4.2 3D nodes

#### Node3D

Inherits `Node`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Transform** (group) | | | |
| `position` | Vec3 | -99999 .. 99999 (unbounded); step 0.001 | Vec3(0, 0, 0) |
| `rotation` | Vec3 | -360 .. 360 (unbounded) degrees in the editor; step 0.1; value in radians |  |
| `rotation_degrees` | Vec3 | not stored in scene files |  |
| `quaternion` | Quat | not stored in scene files | Quat(0, 0, 0, 1) |
| `scale` | Vec3 |  | Vec3(1, 1, 1) |
| `global_position` | Vec3 | not stored in scene files |  |
| `global_rotation` | Vec3 | not stored in scene files |  |
| `global_rotation_degrees` | Vec3 | not stored in scene files |  |
| **Visibility** (group) | | | |
| `visible` | bool |  | True |
| **Collision** (group) | | | |
| `collision_enabled` | bool | toggles its group in the editor; not stored in scene files |  |
| `collision_type` | int |  |  |
| `collision_shape` | int | 0=None, 1=Ellipsoid, 2=Box, 3=Mesh |  |
| `collision_radii` | Vec3 | 0 .. 1024 (no upper bound); step 0.001 |  |
| `collision_box_min` | Vec3 |  |  |
| `collision_box_max` | Vec3 |  |  |
| `collision_mesh` | ResourcePath (string) | resource path, type Mesh |  |

Methods:

- `translate(offset: Vec3)` - Same as translate_object_local: the offset is rotated and scaled by the node transform.
- `translate_object_local(offset: Vec3)` - Moves along the node own axes (the offset is rotated and scaled by the node rotation and scale).
- `rotate(axis: Vec3, angle: float)` - Angle in radians.
- `rotate_object_local(axis: Vec3, angle: float)` - Angle in radians; the axis is in the node own space.
- `rotate_x(angle: float)` - Radians.
- `rotate_y(angle: float)` - Radians.
- `rotate_z(angle: float)` - Radians.
- `yaw(degrees: float)` - Rotates around Y by an angle in degrees.
- `pitch(degrees: float)` - Rotates around the local X axis by an angle in degrees.
- `roll(degrees: float)` - Rotates around the local Z axis by an angle in degrees.
- `move_forward(distance: float)` - Moves along local -Z.
- `move_right(distance: float)` - Moves along local +X.
- `move_up(distance: float)` - Moves along local +Y.
- `look_at(target: Vec3, up: Vec3 = Vec3(0, 1, 0))` - Orients the node so that -Z points at a global position.
- `look_at_from_position(position: Vec3, target: Vec3, up: Vec3 = Vec3(0, 1, 0))` - Places the node at a global position and orients -Z at the target.
- `to_local(point: Vec3)` -> `Vec3` - Global point to this node local space.
- `to_global(point: Vec3)` -> `Vec3` - Local point to global space.
- `forward()` -> `Vec3` - Normalized global direction of local -Z.
- `right()` -> `Vec3` - Normalized global direction of local +X.
- `up()` -> `Vec3` - Normalized global direction of local +Y.
- `show()`
- `hide()`
- `is_visible_in_tree()` -> `bool`
- `count_collisions()` -> `int` - Number of hits recorded by the built-in collision system (see the collision_* properties).
- `get_collision_node(index: int)` -> `Node` - Runtime error when the index is out of range.
- `get_collision_point(index: int)` -> `Vec3` - Runtime error when the index is out of range.
- `get_collision_normal(index: int)` -> `Vec3` - Runtime error when the index is out of range.
- `get_parent_node_3d()` -> `Node3D`
- `global_rotate(axis: Vec3, angle: float)`
- `global_translate(offset: Vec3)`
- `global_scale(scale: Vec3)`
- `scale_object_local(scale: Vec3)`
- `orthonormalize()`
- `set_identity()`
- `force_update_transform()`
- `set_collision_capsule(radius: float, height: float)`
- `collided_with(type: int)` -> `Node3D`
- `reset_collision()`

### 4.3 Visual nodes

#### VisualInstance3D

Inherits `Node3D`. Abstract.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Layers** (group) | | | |
| `layers` | int | bitmask of render layers | 1 |
| **Geometry** (group) | | | |
| `cast_shadow` | bool |  | True |
| `transparency` | float | 0 .. 1; step 0.01. Above 0 the instance is drawn in the transparent pass with its materials' alpha scaled by 1 - transparency (fading a character when the camera gets close). Each surface writes its own depth first and blends only its front layer, so double-sided and alpha-cutout materials fade smoothly without showing their inner faces | 0 |
| `reflector` | NodePath (string) | node path, valid types PlanarReflector3D |  |
| **Global Illumination** (group) | | | |
| `gi_mode` | int | 0=Disabled, 1=Static, 2=Dynamic | 1 (Static) |
| `gi_lightmap_texel_scale` | float | 0.01 .. 10 (no upper bound); step 0.0001 | 1 |

Methods:

- `set_layer_mask_value(layer_number: int, enable: bool)`
- `get_layer_mask_value(layer_number: int)` -> `bool`
- `get_reflector()` -> `PlanarReflector3D`
- `get_aabb()` -> `Aabb`

#### MeshInstance3D

Inherits `VisualInstance3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `mesh` | ResourcePath (string) | resource path, type Mesh |  |
| `color` | Color |  | Color(1, 1, 1, 1) |
| `material_override` | ResourcePath (string) | resource path, type Material |  |
| **Skeleton** (group) | | | |
| `skeleton` | NodePath (string) | node path, valid types Skeleton3D | ".." |

Methods:

- `set_surface_visible(surface: int, visible: bool)`
- `is_surface_visible(surface: int)` -> `bool`
- `get_skeleton()` -> `Skeleton3D`

#### MultiMeshInstance3D

Inherits `VisualInstance3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `mesh` | ResourcePath (string) | resource path, type Mesh |  |
| `material_override` | ResourcePath (string) | resource path, type Material |  |
| `use_colors` | bool |  |  |
| `use_custom_data` | bool |  |  |
| `instance_count` | int | 0 .. 16777216 (no upper bound); step 1 |  |
| `visible_instance_count` | int | -1 .. 16777216 (no upper bound); step 1 |  |

Methods:

- `set_instance_color(instance: int, color: Vec4)`
- `get_instance_color(instance: int)` -> `Color`
- `set_instance_custom_data(instance: int, data: Vec4)`
- `get_instance_custom_data(instance: int)` -> `Vec4`

#### ManualMesh3D

Inherits `VisualInstance3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Dynamic** (group) | | | |
| `dynamic_vertex_capacity` | int | 0 .. 16777216 (no upper bound); step 1 | 0 |
| `dynamic_index_capacity` | int | 0 .. 16777216 (no upper bound); step 1 | 0 |

Methods:

- `surface_begin(topology: int (TOPOLOGY_*))` -> `bool`
- `surface_set_color(color: Vec4)`
- `surface_set_normal(normal: Vec3)`
- `surface_set_tangent(tangent: Vec4)`
- `surface_set_uv(uv: Vec2)`
- `surface_set_uv2(uv2: Vec2)`
- `surface_add_vertex(position: Vec3)`
- `surface_add_index(index: int)`
- `surface_end()` -> `bool`
- `set_surface_material(surface: int, path: string)` -> `bool` - Material of a finished surface (index from 0), as a material resource path such as `assets/materials/props.mat::Barrel`.
- `is_surface_active()` -> `bool`
- `clear_surfaces()`
- `get_surface_count()` -> `int`
- `is_dynamic()` -> `bool`

#### Hair3D

Inherits `VisualInstance3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `skeleton` | NodePath (string) | node path, valid types Skeleton3D | ".." |
| `material` | ResourcePath (string) | resource path, type Material |  |
| `texture` | ResourcePath (string) | resource path, type Texture2D |  |
| `simulation` | int | 0=Auto, 1=Cpu, 2=Gpu | 0 (Auto) |
| **Strands** (group) | | | |
| `strand_count` | int | 0 .. 1000000 (no upper bound); step 1 |  |
| `segment_count` | int | 1 .. 64 (no upper bound); step 1 |  |
| `billboard_count` | int | 1 .. 64 (no upper bound); step 1 |  |
| `random_seed` | int |  |  |
| `length` | float | 0 .. 100 (no upper bound); step 0.001 |  |
| `width` | float | 0 .. 100 (no upper bound); step 0.001 |  |
| `randomness` | float | 0 .. 1 (no upper bound); step 0.001 |  |
| `uniformity` | float | 0 .. 1 (no upper bound); step 0.001 |  |
| `view_distance` | float | 0 .. 100000 (no upper bound); step 0.1 |  |
| `camera_bend` | bool |  |  |
| **Simulation** (group) | | | |
| `stiffness` | float | 0 .. 1 (no upper bound); step 0.001 |  |
| `drag` | float | 0 .. 1 (no upper bound); step 0.001 |  |
| `gravity_power` | float | 0 .. 100 (no upper bound); step 0.001 |  |
| **Wind** (group) | | | |
| `wind_direction` | Vec3 |  | Vec3(0, 0, 0) |
| `wind_speed` | float | 0 .. 100 (unbounded); step 0.01 | 1 |
| `wind_randomness` | float | 0 .. 100 (no upper bound); step 0.01 | 5 |
| `wind_wave_size` | float | 0 .. 100 (no upper bound); step 0.01 | 1 |

Methods:

- `reset()`
- `get_scalp_triangle_count()` -> `int`
- `get_skeleton()` -> `Skeleton3D`
- `clear_colliders()`
- `get_collider_count()` -> `int`
- `clear_force_fields()`
- `is_simulating_on_gpu()` -> `bool`

#### Decal3D

Inherits `Node3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `size` | Vec3 | 0 .. 1024 (no upper bound); step 0.001 |  |
| **Textures** (group) | | | |
| `texture_albedo` | ResourcePath (string) | resource path, type Texture2D |  |
| `texture_normal` | ResourcePath (string) | resource path, type Texture2D |  |
| `texture_orm` | ResourcePath (string) | resource path, type Texture2D |  |
| `texture_emission` | ResourcePath (string) | resource path, type Texture2D |  |
| **Parameters** (group) | | | |
| `emission_energy` | float | 0 .. 16 (no upper bound); step 0.01 |  |
| `modulate` | Color |  |  |
| `albedo_mix` | float | 0 .. 1; step 0.01 |  |
| `normal_fade` | float | 0 .. 0.999; step 0.001 |  |
| **Vertical Fade** (group) | | | |
| `upper_fade` | float | 0 .. 16 (no upper bound); step 0.001 |  |
| `lower_fade` | float | 0 .. 16 (no upper bound); step 0.001 |  |
| **Distance Fade** (group) | | | |
| `distance_fade_enabled` | bool |  |  |
| `distance_fade_begin` | float | 0 .. 4096 (no upper bound); step 0.01 |  |
| `distance_fade_length` | float | 0 .. 4096 (no upper bound); step 0.01 |  |
| **Cull Mask** (group) | | | |
| `cull_mask` | int | bitmask of render layers |  |
| `sorting_offset` | float | -1024 .. 1024 (unbounded); step 0.01 | 0 |

Methods:

- `get_aabb()` -> `Aabb`

#### ReflectionProbe3D

Inherits `Node3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `update_mode` | int | 0=Once (Fast), 1=Always (Slow) |  |
| `intensity` | float | 0 .. 1 (no upper bound); step 0.01 |  |
| `blend_distance` | float | 0 .. 8 (no upper bound); step 0.01 |  |
| `max_distance` | float | 0 .. 16384 (no upper bound); step 0.1 |  |
| `size` | Vec3 |  |  |
| `origin_offset` | Vec3 |  |  |
| `box_projection` | bool |  |  |
| `interior` | bool |  |  |
| `enable_shadows` | bool |  |  |
| `cull_mask` | int | bitmask of render layers |  |
| `reflection_mask` | int | bitmask of render layers |  |
| `mesh_lod_threshold` | float | 0 .. 1024; step 0.1 |  |
| **Ambient** (group) | | | |
| `ambient_mode` | int | 0=Disabled, 1=Environment, 2=Constant Color |  |
| `ambient_color` | Color | editor hides alpha |  |
| `ambient_color_energy` | float | 0 .. 16; step 0.01 |  |

Methods:

- `get_aabb()` -> `Aabb`

#### LightmapGI

Inherits `Node3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Tweaks** (group) | | | |
| `quality` | int | 0=Low, 1=Medium, 2=High, 3=Ultra | 1 (Medium) |
| `supersampling` | bool |  | False |
| `supersampling_factor` | float | 1 .. 8; step 1 | 2 |
| `bounces` | int | 0 .. 6 (no upper bound); step 1 | 3 |
| `bounce_indirect_energy` | float | 0 .. 2; step 0.01 | 1 |
| `directional` | bool |  | False |
| `specular_intensity` | float | 0 .. 10; step 0.01 | 0 |
| `shadowmask_mode` | int | 0=None, 1=Replace, 2=Overlay | 0 (None) |
| `use_texture_for_bounces` | bool |  | True |
| `interior` | bool |  | False |
| `use_denoiser` | bool |  | True |
| `denoiser_strength` | float | 0.001 .. 0.2 (no upper bound); step 0.001 | 0.1 |
| `denoiser_range` | int | 1 .. 20; step 1 | 10 |
| `bias` | float | 0.00001 .. 0.1 (no upper bound); step 0.00001 | 0.0005 |
| `texel_scale` | float | 0.01 .. 100; step 0.01 | 1 |
| `max_texture_size` | int | 2048 .. 16384; step 1 | 16384 |
| **Environment** (group) | | | |
| `environment_mode` | int | 0=Disabled, 1=Scene, 2=Custom Color | 1 (Scene) |
| `environment_custom_color` | Color | editor hides alpha | Color(1, 1, 1, 1) |
| `environment_custom_energy` | float | 0 .. 64; step 0.01 | 1 |
| **Gen Probes** (group) | | | |
| `generate_probes_subdiv` | int | 0=Disabled, 1=4, 2=8, 3=16, 4=32 | 2 (8) |
| **Data** (group) | | | |
| `light_data` | ResourcePath (string) | file path, extension lmdata |  |

Methods:

- `update_light_data()`
- `get_aabb()` -> `Aabb`

#### PlanarReflector3D

Inherits `Node3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `half_size` | Vec2 |  | Vec2(1, 1) |
| `resolution` | int | 16 .. 8192; step 1 | 512 |
| `max_distance` | float | 0 .. 4096 (no upper bound); step 0.01 | 5 |

No methods of its own.

### 4.4 Lights

#### Light3D

Inherits `Node3D`. Abstract.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Light** (group) | | | |
| `light_color` | Color | editor hides alpha | Color(1, 1, 1, 1) |
| `light_energy` | float | 0 .. 16 (no upper bound); step 0.001 |  |
| `light_negative` | bool |  | False |
| `light_specular` | float | 0 .. 16 (no upper bound); step 0.001 |  |
| `light_cull_mask` | int | bitmask of render layers | 4294967295 |
| `light_bake_mode` | int | 0=Disabled, 1=Static, 2=Dynamic | 2 (Dynamic) |
| **Shadow** (group) | | | |
| `shadow_enabled` | bool | toggles its group in the editor | False |
| `shadow_bias` | float | 0 .. 10; step 0.001 |  |
| `shadow_normal_bias` | float | 0 .. 10; step 0.001 |  |
| `shadow_opacity` | float | 0 .. 1; step 0.01 |  |
| `shadow_blur` | float | 0 .. 10; step 0.001 |  |

Methods:

- `get_light_type()` -> `int (LIGHT_TYPE_*)`
- `clear_lens_flare()`
- `get_lens_flare_count()` -> `int`

#### DirectionalLight3D

Inherits `Light3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Light** (group) | | | |
| `light_angular_distance` | float | 0 .. 90; step 0.01 |  |
| **Directional Shadow** (group) | | | |
| `directional_shadow_mode` | int | 0=Orthogonal (Fast), 1=PSSM 2 Splits (Average), 2=PSSM 4 Splits (Slow) | 2 (PSSM 4 Splits (Slow)) |
| `directional_shadow_split_1` | float | 0 .. 1; step 0.001 |  |
| `directional_shadow_split_2` | float | 0 .. 1; step 0.001 |  |
| `directional_shadow_split_3` | float | 0 .. 1; step 0.001 |  |
| `directional_shadow_blend_splits` | bool |  | False |
| `directional_shadow_fade_start` | float | 0 .. 1; step 0.001 |  |
| `directional_shadow_max_distance` | float | 0 .. 8192 (no upper bound); step 0.1 |  |
| `directional_shadow_pancake_size` | float | 0 .. 1024 (no upper bound); step 0.1 |  |

No methods of its own.

#### OmniLight3D

Inherits `Light3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Light** (group) | | | |
| `light_size` | float | 0 .. 1 (no upper bound); step 0.001 |  |
| **Omni** (group) | | | |
| `omni_range` | float | 0 .. 4096 (no upper bound); step 0.001 |  |
| `omni_attenuation` | float | -10 .. 10 (unbounded); step 0.001 |  |
| `omni_shadow_mode` | int | 0=Dual Paraboloid, 1=Cube |  |

No methods of its own.

#### SpotLight3D

Inherits `Light3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Light** (group) | | | |
| `light_size` | float | 0 .. 1 (no upper bound); step 0.001 |  |
| **Spot** (group) | | | |
| `spot_range` | float | 0 .. 4096 (no upper bound); step 0.001 |  |
| `spot_attenuation` | float | -10 .. 10 (unbounded); step 0.01 |  |
| `spot_angle` | float | 0 .. 180; step 0.01 |  |
| `spot_angle_attenuation` | float |  |  |

No methods of its own.

### 4.5 Camera, viewport and environment

#### Camera3D

Inherits `Node3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `projection` | int | 0=Perspective, 1=Orthogonal | 0 (Perspective) |
| `current` | bool |  |  |
| `fov` | float | 1 .. 179; step 0.1 | 75 |
| `size` | float | 0.001 .. 100 (no upper bound); step 0.001 | 1 |
| `near` | float | 0.001 .. 10 (no upper bound); step 0.001 | 0.05 |
| `far` | float | 0.01 .. 4000 (no upper bound); step 0.01 | 4000 |
| **Cull Mask** (group) | | | |
| `cull_mask` | int | bitmask of render layers | 4294967295 |
| **Attributes** (group) | | | |
| `attributes_override` | bool | toggles its group in the editor | False |
| `attributes_dof_blur_amount` | float | 0 .. 1; step 0.01 | 0.1 |
| **Attributes DOF Far** (group) | | | |
| `attributes_dof_blur_far_enabled` | bool | toggles its group in the editor | False |
| `attributes_dof_blur_far_distance` | float | 0.01 .. 8192; step 0.01 | 10 |
| `attributes_dof_blur_far_transition` | float | -1 .. 8192; step 0.01 | 5 |
| **Attributes DOF Near** (group) | | | |
| `attributes_dof_blur_near_enabled` | bool | toggles its group in the editor | False |
| `attributes_dof_blur_near_distance` | float | 0.01 .. 8192; step 0.01 | 2 |
| `attributes_dof_blur_near_transition` | float | -1 .. 8192; step 0.01 | 1 |

Methods:

- `make_current()`
- `is_current()` -> `bool`
- `project_ray_origin(point: Vec2)` -> `Vec3` - Also accepts two numbers (x, y). Runtime error when the camera is not inside a scene with a viewport.
- `project_ray_normal(point: Vec2)` -> `Vec3` - Also accepts two numbers (x, y). Runtime error when the camera is not inside a scene with a viewport.
- `set_perspective(fovy_degrees: float, z_near: float, z_far: float)`
- `set_orthogonal(size: float, z_near: float, z_far: float)`
- `clear_current(enable_next: bool = True)` - With `enable_next` True another camera of the viewport becomes current; with False the viewport has no current camera.
- `project_local_ray_normal(point: Vec2)` -> `Vec3` - Also accepts two numbers (x, y). Runtime error when the camera is not inside a scene with a viewport.
- `project_ray(point: Vec2)` -> `Ray` - Also accepts two numbers (x, y). Runtime error when the camera is not inside a scene with a viewport.
- `project_mouse_ray()` -> `Ray`
- `project_position(point: Vec2, depth: float)` -> `Vec3` - Also accepts three numbers (x, y, depth). Runtime error when the camera is not inside a scene with a viewport.
- `unproject_position(position: Vec3)` -> `Vec2`
- `pick_depth(point: Vec2)` -> `Vec3` - Also accepts two numbers (x, y). Returns the picked position, or None when nothing is hit. Runtime error when the camera is not inside a scene with a viewport.
- `is_position_behind(position: Vec3)` -> `bool`
- `is_position_in_frustum(position: Vec3)` -> `bool`
- `get_viewport()` -> `Viewport`

#### Viewport

Inherits `Node`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `size` | Vec2 | 0 .. 16384 (no upper bound); step 1 |  |
| `clear_color` | Color |  | Color(0, 0, 0, 1) |
| `msaa_3d` | int | 0=Disabled, 1=2x, 2=4x, 3=8x | 0 (Disabled) |
| `scaling_3d_scale` | float | 0.25 .. 2; step 0.01 | 1 |
| `scaling_3d_size` | Vec2 | 0 .. 16384; step 1. A fixed size for the 3D picture, in pixels; with 0 in either the picture follows `scaling_3d_scale` instead | Vec2(0, 0) |
| `scaling_3d_aspect` | int | 0=Stretch, 1=Keep, 2=Crop, 3=Expand. How a picture of `scaling_3d_size` is fitted to the window when the aspects differ: Stretch fills the window and loses the aspect; Keep keeps it, with black bars; Crop keeps it and scales until the window is covered, cutting the edges; Expand keeps the picture's lines (or its columns, on a taller window) and widens it to the window's aspect, so there are no bars and no distortion | 0 (Stretch) |
| `scaling_3d_mode` | int | 0=Bilinear, 1=FSR 1.0. How the picture is scaled to its rectangle: one linear sample per pixel, or FSR 1.0 (EASU then RCAS, two compute passes at the window's size) | 0 (Bilinear) |
| `fsr_sharpness` | float | 0 .. 2; step 0.01. The RCAS sharpness of FSR 1.0 in stops: 0 the sharpest, 2 the softest | 0.2 |
| `screen_space_aa` | int | 0=Disabled, 1=FXAA, 2=SMAA. FXAA runs inside the tonemap; SMAA runs after it on the tonemapped picture at the 3D size, in three passes (edges with a stencil, blending weights where the stencil is set, blending), with the edge contrast of the project setting `rendering/anti_aliasing/quality/smaa_edge_detection_threshold` (0.05) | 0 (Disabled) |
| `mesh_lod_threshold` | float | 0 .. 1024; step 0.1 | 1 |
| **Positional Shadow Atlas** (group) | | | |
| `positional_shadow_atlas_size` | int | 0 .. 16384; step 1 | 4096 |
| `positional_shadow_atlas_16_bits` | bool |  | False |
| `positional_shadow_atlas_quad_0` | int | 0 .. 1024; step 1 |  |
| `positional_shadow_atlas_quad_1` | int | 0 .. 1024; step 1 |  |
| `positional_shadow_atlas_quad_2` | int | 0 .. 1024; step 1 |  |
| `positional_shadow_atlas_quad_3` | int | 0 .. 1024; step 1 |  |

The 3D picture is tonemapped at its own size and then scaled to the window by `scaling_3d_mode`; the HUD is drawn at the window's size on top. `get_aspect()` is the aspect of the 3D picture, and the camera's `project_*`, `unproject_position` and `pick_depth` take and give window pixels, mapped through the rectangle the picture fills, so a point under a black bar of Keep is outside the picture. The project settings `rendering/scaling_3d/width`, `height`, `aspect`, `mode` and `fsr_sharpness` set the same for the window.

Methods:

- `get_aspect()` -> `float`
- `get_mouse_position()` -> `Vec2`
- `get_msaa_samples()` -> `int`
- `get_camera_3d()` -> `Camera3D`

#### Window

Inherits `Viewport`. A viewport attached to the screen. It adds no members of its own.

No properties of its own.

No methods of its own.

#### WorldEnvironment

Inherits `Node`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Background** (group) | | | |
| `background_mode` | int | 0=Clear Color, 1=Custom Color, 2=Sky | 0 (Clear Color) |
| `background_color` | Color | editor hides alpha | Color(0, 0, 0, 1) |
| `background_energy_multiplier` | float | 0 .. 16; step 0.01 | 1 |
| **Sky** (group) | | | |
| `sky_mode` | int | 0=Procedural, 1=Panorama, 2=Cubemap | 0 (Procedural) |
| `sky_texture` | ResourcePath (string) | resource path, type Texture2D |  |
| `sky_texture_energy_multiplier` | float | 0 .. 16; step 0.01 | 1 |
| `sky_top_color` | Color | editor hides alpha | Color(0.385, 0.454, 0.55, 1) |
| `sky_horizon_color` | Color | editor hides alpha | Color(0.6463, 0.6558, 0.6708, 1) |
| `sky_curve` | float | 0 .. 1; step 0.001 | 0.15 |
| `sky_energy_multiplier` | float | 0 .. 64; step 0.01 | 1 |
| `sky_ground_bottom_color` | Color | editor hides alpha | Color(0.2, 0.169, 0.133, 1) |
| `sky_ground_horizon_color` | Color | editor hides alpha | Color(0.6463, 0.6558, 0.6708, 1) |
| `sky_ground_curve` | float | 0 .. 1; step 0.001 | 0.02 |
| `sky_ground_energy_multiplier` | float | 0 .. 64; step 0.01 | 1 |
| `sky_sun_angle_max` | float | 0 .. 360; step 0.01 | 30 |
| `sky_sun_curve` | float | 0 .. 1; step 0.001 | 0.15 |
| `sky_overall_energy_multiplier` | float | 0 .. 64; step 0.01 | 1 |
| `sky_use_debanding` | bool |  | True |
| `sky_radiance_size` | int |  | 256 |
| **Ambient Light** (group) | | | |
| `ambient_light_source` | int | 0=Background, 1=Color, 2=Disabled, 3=Sky | 0 (Background) |
| `ambient_light_color` | Color | editor hides alpha | Color(0, 0, 0, 1) |
| `ambient_light_sky_contribution` | float | 0 .. 1; step 0.01 | 1 |
| `ambient_light_energy` | float | 0 .. 16; step 0.01 | 1 |
| **Reflected Light** (group) | | | |
| `reflected_light_source` | int | 0=Background, 1=Disabled, 2=Sky | 0 (Background) |
| **Tonemap** (group) | | | |
| `tonemap_mode` | int | 0=Linear, 1=Reinhard, 2=Filmic, 3=ACES, 4=AgX | 0 (Linear) |
| `tonemap_exposure` | float | 0 .. 4; step 0.01 | 1 |
| `tonemap_white` | float | 1 .. 16; step 0.01 | 1 |
| `tonemap_agx_contrast` | float | 1 .. 2; step 0.01 | 1.25 |
| **SSR** (group) | | | |
| `ssr_enabled` | bool | toggles its group in the editor | False |
| `ssr_max_steps` | int | 32 .. 512; step 1 | 64 |
| `ssr_fade_in` | float | 0 .. 8; step 0.01 | 0.15 |
| `ssr_fade_out` | float | 0 .. 8; step 0.01 | 2 |
| `ssr_depth_tolerance` | float | 0.01 .. 128; step 0.1 | 0.5 |
| **SSAO** (group) | | | |
| `ssao_enabled` | bool | toggles its group in the editor | False |
| `ssao_radius` | float | 0.01 .. 16; step 0.01 | 1 |
| `ssao_intensity` | float | 0 .. 16; step 0.01 | 2 |
| `ssao_power` | float | 0.01 .. 16; step 0.01 | 1.5 |
| `ssao_detail` | float | 0 .. 5; step 0.01 | 0.5 |
| `ssao_horizon` | float | 0 .. 1; step 0.01 | 0.06 |
| `ssao_sharpness` | float | 0 .. 1; step 0.01 | 0.98 |
| `ssao_light_affect` | float | 0 .. 1; step 0.01 | 0 |
| `ssao_ao_channel_affect` | float | 0 .. 1; step 0.01 | 0 |
| **Glow** (group) | | | |
| `glow_enabled` | bool | toggles its group in the editor | False |
| `glow_levels_1` (`glow_levels/1`) | float | 0 .. 16 (no upper bound); step 0.01 | 0 |
| `glow_levels_2` (`glow_levels/2`) | float | 0 .. 16 (no upper bound); step 0.01 | 0.8 |
| `glow_levels_3` (`glow_levels/3`) | float | 0 .. 16 (no upper bound); step 0.01 | 0.4 |
| `glow_levels_4` (`glow_levels/4`) | float | 0 .. 16 (no upper bound); step 0.01 | 0.1 |
| `glow_levels_5` (`glow_levels/5`) | float | 0 .. 16 (no upper bound); step 0.01 | 0 |
| `glow_levels_6` (`glow_levels/6`) | float | 0 .. 16 (no upper bound); step 0.01 | 0 |
| `glow_levels_7` (`glow_levels/7`) | float | 0 .. 16 (no upper bound); step 0.01 | 0 |
| `glow_normalized` | bool |  | False |
| `glow_intensity` | float | 0 .. 8; step 0.01 | 0.3 |
| `glow_strength` | float | 0 .. 2; step 0.01 | 1 |
| `glow_mix` | float | 0 .. 1; step 0.001 | 0.05 |
| `glow_bloom` | float | 0 .. 1; step 0.01 | 0 |
| `glow_blend_mode` | int | 0=Additive, 1=Screen, 2=Soft Light, 3=Replace, 4=Mix | 1 (Screen) |
| `glow_hdr_threshold` | float | 0 .. 4; step 0.01 | 1 |
| `glow_hdr_scale` | float | 0 .. 4; step 0.01 | 2 |
| `glow_hdr_luminance_cap` | float | 0 .. 256; step 0.01 | 12 |
| `glow_map_strength` | float | 0 .. 1; step 0.01 | 0.8 |
| `glow_map` | ResourcePath (string) | resource path, type Texture2D |  |
| **Fog** (group) | | | |
| `fog_enabled` | bool | toggles its group in the editor | False |
| `fog_mode` | int | 0=Exponential, 1=Depth | 0 (Exponential) |
| `fog_light_color` | Color | editor hides alpha | Color(0.518, 0.553, 0.608, 1) |
| `fog_light_energy` | float | 0 .. 16; step 0.01 | 1 |
| `fog_sun_scatter` | float | 0 .. 1; step 0.01 | 0 |
| `fog_density` | float | 0 .. 1; step 0.0001 | 0.01 |
| `fog_aerial_perspective` | float | 0 .. 1; step 0.001 | 0 |
| `fog_sky_affect` | float | 0 .. 1; step 0.001 | 1 |
| `fog_height` | float | -1024 .. 1024; step 0.01 | 0 |
| `fog_height_density` | float | -16 .. 16; step 0.0001 | 0 |
| `fog_depth_curve` | float |  | 1 |
| `fog_depth_begin` | float | 0 .. 4000; step 0.1 | 10 |
| `fog_depth_end` | float | 0 .. 4000; step 0.1 | 100 |
| **Adjustments** (group) | | | |
| `adjustment_enabled` | bool | toggles its group in the editor | False |
| `adjustment_brightness` | float | 0 .. 2; step 0.01 | 1 |
| `adjustment_contrast` | float | 0.75 .. 1.25; step 0.005 | 1 |
| `adjustment_saturation` | float | 0 .. 2; step 0.01 | 1 |
| `adjustment_color_correction` | ResourcePath (string) | resource path, type Texture2D |  |
| **Depth of Field** (group) | | | |
| `dof_blur_amount` | float | 0 .. 1; step 0.01 | 0.1 |
| **Depth of Field Far** (group) | | | |
| `dof_blur_far_enabled` | bool | toggles its group in the editor | False |
| `dof_blur_far_distance` | float | 0.01 .. 8192; step 0.01 | 10 |
| `dof_blur_far_transition` | float | -1 .. 8192; step 0.01 | 5 |
| **Depth of Field Near** (group) | | | |
| `dof_blur_near_enabled` | bool | toggles its group in the editor | False |
| `dof_blur_near_distance` | float | 0.01 .. 8192; step 0.01 | 2 |
| `dof_blur_near_transition` | float | -1 .. 8192; step 0.01 | 1 |

No methods of its own.

### 4.6 Physics bodies

#### PhysicsBody3D

Inherits `Node3D`. Abstract.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Collision** (group) | | | |
| `collision_layer` | int | bitmask of physics layers | 1 |
| `collision_mask` | int | bitmask of physics layers | 1 |
| **Material** (group) | | | |
| `friction` | float | 0 .. 1 (no upper bound); step 0.01 | 1 |
| `bounce` | float | 0 .. 1 (no upper bound); step 0.01 | 0 |
| **Contacts** (group) | | | |
| `contact_monitor` | bool |  | False |
| `max_contacts_reported` | int | 0 .. 64 (no upper bound); step 1 | 8 |

Methods:

- `get_mode()` -> `int (BODY_MODE_*)`
- `is_simulated()` -> `bool`
- `set_contact_monitor(enabled: bool, max_contacts: int = 8)` - Turns contact reporting (and the `_body_entered` / `_body_exited` hooks) on or off; a negative `max_contacts` is treated as 0.

#### StaticBody3D

Inherits `PhysicsBody3D`.

No properties of its own.

No methods of its own.

#### AnimatableBody3D

Inherits `PhysicsBody3D`.

No properties of its own.

No methods of its own.

#### RigidBody3D

Inherits `PhysicsBody3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `mass` | float | 0.001 .. 1000 (no upper bound); step 0.001 | 1 |
| **Mass Distribution** (group) | | | |
| `inertia` | Vec3 |  | Vec3(0, 0, 0) |
| **Deactivation** (group) | | | |
| `sleeping` | bool |  |  |
| `can_sleep` | bool |  | True |
| `lock_rotation` | bool |  | False |
| **Solver** (group) | | | |
| `interpolated` | bool |  | True |
| **Linear** (group) | | | |
| `linear_velocity` | Vec3 |  |  |
| `linear_damp_mode` | int | 0=Combine, 1=Replace | 0 (Combine) |
| `linear_damp` | float | 0 .. 100 (no upper bound); step 0.001 | 0 |
| **Angular** (group) | | | |
| `angular_velocity` | Vec3 |  |  |
| `angular_damp_mode` | int | 0=Combine, 1=Replace | 0 (Combine) |
| `angular_damp` | float | 0 .. 100 (no upper bound); step 0.001 | 0 |
| **Constant Forces** (group) | | | |
| `constant_force` | Vec3 |  | Vec3(0, 0, 0) |
| `constant_torque` | Vec3 |  | Vec3(0, 0, 0) |

Methods:

- `apply_central_impulse(impulse: Vec3)`
- `apply_impulse(impulse: Vec3, offset: Vec3)`
- `apply_torque_impulse(impulse: Vec3)`
- `apply_central_force(force: Vec3)`
- `apply_force(force: Vec3, offset: Vec3)`
- `apply_torque(torque: Vec3)`
- `set_linear_velocity(velocity: Vec3)`
- `get_linear_velocity()` -> `Vec3`
- `set_angular_velocity(velocity: Vec3)`
- `get_angular_velocity()` -> `Vec3`
- `get_velocity_at_position(global_position: Vec3)` -> `Vec3`
- `get_computed_inertia()` -> `Vec3`
- `get_contact_count()` -> `int`
- `get_colliding_bodies()` -> `list of PhysicsBody3D`

### 4.7 Collision shapes

#### CarDamage3D

Inherits `Node3D`. The crumpling of a car, as arcade crashes do it: the mesh is never simulated. A child of a `GroundVehicle3D`; it draws the car itself from `model`, baked into the frame of the vehicle with its own transform. Every vertex has a weight from 0 to 1 that goes up near where the car is hit, and is drawn at its place plus the weight times a fixed crumpled displacement, worked out once in zones (the nose and the tail toward their tips, the flanks, the roof) and zero in the cabin, so however hard the crash the car stays a car. The weights follow what the physics reports: every contact point of the vehicle's body over `min_impulse` (N s) at the place it happened. The drawn damage closes in over a tenth of a second; glass breaks where it is crumpled 0.3. When the nose or the tail comes in the body goes on as far, and its collision shape is pulled in with it.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `model` | String (file) | *.glb, *.gltf, *.b3d, *.fbx, *.obj | "" |
| `skip_prefix` | String |  | "wheel" |
| `min_impulse` | float | 0 .. 100000 (no upper bound); step 1 | 300 |
| `impact_fx` | NodePath (ImpactFx3D) |  |  |

Nodes of the model whose name starts with `skip_prefix` are left out (the wheels). Opaque and masked surfaces are the metal; blended ones are the glass. With `impact_fx`, every blow over `min_impulse` throws sparks and smoke where it hit (of two cars that hit each other only one throws them), and a nose or a tail crushed more than 0.25 m goes on smoking from there.

Methods:

- `is_built()` -> `bool`
- `get_peak()` -> `float` - The most any vertex is crumpled, 0 to 1.
- `get_nose_crush()` -> `float` - How far the nose has come in, m.
- `get_tail_crush()` -> `float`
- `get_step_impulse()` -> `float` - What hit the car in the last physics step, N s.
- `get_total_impulse()` -> `float`
- `damage_near(point: Vec3, radius: float)` -> `float` - The most the body is crumpled within `radius` of a world point.
- `hit(point: Vec3, impulse: float)` - A blow at a world point.
- `reset()` - The car as it was built.

#### SkidMarks3D

Inherits `Node3D`. The marks tyres leave on the ground, as one mesh built as the cars go: a strip of quads behind every wheel, as wide as the tyre, each corner set on the ground under it by a ray, so the strip follows rises, dips and bridges. A mark is as dark as the tyre was sliding when it laid it, fades when it is old, and ends where the wheel left the ground. The 2048 quads are a ring: when all are used the oldest are written over. `GroundVehicle3D` feeds it through its `skid_marks` property; up to 64 wheels can mark one node.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `color` | Color |  | Color(0.17, 0.1, 0.06, 1) |
| `life` | float | 0.1 .. 600 (no upper bound); step 0.1; seconds a mark lasts | 12 |
| `fade` | float | 0.01 .. 600 (no upper bound); step 0.01; the last of them, over which it fades | 5 |
| `rolling_strength` | float | 0 .. 1; step 0.01; how dark a wheel that only rolls marks (sand, mud) | 0.16 |

Methods:

- `clear()`

#### ImpactFx3D

Inherits `Node3D`. What a crash throws up, with four internal `ParticleSystem3D` in world space that only emit when asked: sparks that streak out from a blow and fall, grey smoke that puffs up from it and drifts, flames, and black smoke. Each frame lets out at most 60 sparks, 3 puffs and 14 flames, so many contacts at once do not flood it. `crash_sounds` is a comma-separated list of sound files played in turn where a crash is heard. `CarDamage3D` feeds it through its `impact_fx` property: a car that loses more than 3.5 m/s in a step (times the time scale) while going faster than 4 m/s is heard at 0.9, and a lighter contact at 0.35 plus 0.08 per m/s lost, at most once every 0.45 s.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `crash_sounds` | String |  | "" |

Methods:

- `impact(point: Vec3, impulse: float)` - A blow of `impulse` (N s) at a world point: 3 to 30 sparks and one or two puffs.
- `smoke(point: Vec3, count: int)` - Puffs of smoke at a world point.
- `flame(point: Vec3, count: int)` - Flames and black smoke from a world point, `count` of each call (a burning car calls it every frame).
- `blast(point: Vec3)` - An explosion: a fireball, sparks and a black cloud.
- `crash(point: Vec3, volume: float)` - The next of `crash_sounds` at a world point.

#### BoostJet3D

Inherits `Node3D`. The flame of a boost at an exhaust, fixed to whatever carries it so it stays at the pipe at any speed. Two jets, an outer flame 2 m long and a core 1.1 m long, run down +Z from the node: each is six sheets of a flame drawn in code (white where it leaves the pipe, blue along its body, orange at its tip), crossed along its length and added to what is behind, and they stretch and shrink many times a second, the core faster than the flame. Four glows that turn to the camera sit along them, blue-white at the pipe to orange at the tip, so the jets are seen from behind where the sheets are edge on. `phase` shifts the flicker, so two pipes do not move together.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `burning` | bool |  | False |
| `glow_texture` | String (file) | *.png, *.jpg, *.tga, *.dds | "" |
| `phase` | float |  | 0 |

#### Traffic3D

Inherits `Node3D`. Runs the `GroundVehicle3D` children it has along a `Path3D` around the `target`, as traffic on rails. Each car is a kinematic body moved along its lane of the line (its `VehicleDriver3D`'s `lane_offset`) at its driver's `top_speed`, at the rest height of its wheels and turned along the line; on `rail_layer` and seeing only `rail_mask`, so it touches neither the world nor the other cars, and the physics spends nothing on its wheels. Put the target car on a layer in `rail_mask` and give it `rail_layer` in its mask to hit them. A car that takes a blow (its `CarDamage3D` reports an impulse) becomes a rigid body with its own layer and mask and the speed it was carrying, with no driver. A car `behind_distance` behind the target, or at the end of the line, is put back on its rails `ahead_distance` ahead of it, up to 80 m further, repaired.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `path` | NodePath (Path3D) |  |  |
| `target` | NodePath (Node3D) |  |  |
| `ahead_distance` | float | 0 .. 5000 (no upper bound); m | 350 |
| `behind_distance` | float | 0 .. 5000 (no upper bound); m | 150 |
| `rail_layer` | int (physics layers) |  | 4 |
| `rail_mask` | int (physics layers) |  | 8 |

Methods:

- `get_active_count()` -> `int` - How many cars have been hit and are in the physics.

#### VehicleDriver3D

Inherits `Node`. Drives the `GroundVehicle3D` it is a child of along a `Path3D`, its baked points the line of the road: it steers at a point of the line some way ahead (7 m plus 0.45 s of speed), takes the speed the sharpest bend ahead allows (the square root of `cornering` over the bend, between 9 m/s and `top_speed`), backs out with the wheel the other way when it has had no speed for 1.6 s, and is put back on the line, pointing along it and repaired, when it is 45 m from the line or stuck for 6 s. It goes the way the car points when it starts, `lane_offset` metres to the right of the line. While `active` it sets the throttle, brake, handbrake and steering of the car every physics step.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `path` | NodePath (Path3D) |  |  |
| `cornering` | float | 0.1 .. 50 (no upper bound); m/s² | 7 |
| `top_speed` | float | 1 .. 200 (no upper bound); m/s | 33 |
| `lane_offset` | float | -20 .. 20; m | 0 |
| `active` | bool |  | True |

Methods:

- `get_progress()` -> `float` - How far along the line the car has gone, in baked points, counted the way it drives.
- `wants_boost()` -> `bool` - The road ahead is straight and the car is well under the speed it allows.

#### CollisionShape3D

Inherits `Node3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `shape_type` | int | 0=None, 1=Box, 2=Sphere, 3=Capsule, 4=Cylinder, 5=Plane, 6=Convex Hull, 7=Concave Mesh, 8=Height Map | 0 (None) |
| `box_size` | Vec3 | 0.001 .. 1024 (no upper bound); step 0.001 | Vec3(1, 1, 1) |
| `sphere_radius` | float | 0.001 .. 1024 (no upper bound); step 0.001 | 0.5 |
| `capsule_radius` | float | 0.001 .. 1024 (no upper bound); step 0.001 | 0.5 |
| `capsule_height` | float | 0.001 .. 1024 (no upper bound); step 0.001 | 2 |
| `cylinder_radius` | float | 0.001 .. 1024 (no upper bound); step 0.001 | 0.5 |
| `cylinder_height` | float | 0.001 .. 1024 (no upper bound); step 0.001 | 2 |
| `plane_normal` | Vec3 |  | Vec3(0, 1, 0) |
| `plane_distance` | float | -1024 .. 1024 (unbounded); step 0.001 | 0 |
| `source_mesh` | ResourcePath (string) | resource path, type Mesh |  |
| `source_terrain` | NodePath (string) | node path, valid types Terrain3D |  |
| `disabled` | bool |  | False |

Methods:

- `set_box(size: Vec3)`
- `set_sphere(radius: float)`
- `set_capsule(radius: float, height: float)`
- `set_cylinder(radius: float, height: float)`
- `set_plane(normal: Vec3, distance: float)`
- `set_convex_from_points(points: list of Vec3)` -> `bool`
- `set_height_map_from_terrain(terrain: Terrain3D)` -> `bool` - Runtime error when the argument is not a Terrain3D.
- `get_hull_points()` -> `list of Vec3`

### 4.8 Joints

#### Joint3D

Inherits `Node3D`. Abstract.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `node_a` | NodePath (string) | node path, valid types PhysicsBody3D |  |
| `node_b` | NodePath (string) | node path, valid types PhysicsBody3D |  |
| `enabled` | bool |  | True |
| **Collision** (group) | | | |
| `exclude_nodes_from_collision` | bool |  | True |

Methods:

- `set_bodies(body_a: PhysicsBody3D, body_b: PhysicsBody3D)`
- `get_body_a()` -> `PhysicsBody3D`
- `get_body_b()` -> `PhysicsBody3D`
- `is_configured()` -> `bool`
- `get_configuration_warning()` -> `string`

#### PinJoint3D

Inherits `Joint3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Motor** (group) | | | |
| `motor_enable` (`motor/enable`) | bool |  | False |
| `motor_axis` (`motor/axis`) | Vec3 |  | Vec3(0, 1, 0) |
| `motor_target_velocity` (`motor/target_velocity`) | float | -200 .. 200 (unbounded) degrees in the editor; step 0.01; value in radians | 0 |
| `motor_max_torque` (`motor/max_torque`) | float | 0 .. 100000 (no upper bound); step 0.01 | 0 |

Methods:

- `set_motor(local_axis: Vec3, target_angular_velocity: float, max_torque: float)`
- `disable_motor()`

#### HingeJoint3D

Inherits `Joint3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Angular Limit** (group) | | | |
| `angular_limit_enable` (`angular_limit/enable`) | bool |  | False |
| `angular_limit_upper` (`angular_limit/upper`) | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians |  |
| `angular_limit_lower` (`angular_limit/lower`) | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians |  |
| **Motor** (group) | | | |
| `motor_enable` (`motor/enable`) | bool |  | False |
| `motor_target_velocity` (`motor/target_velocity`) | float | -200 .. 200 (unbounded) degrees in the editor; step 0.01; value in radians | 1 |
| `motor_max_torque` (`motor/max_torque`) | float | 0 .. 100000 (no upper bound); step 0.01 | 1 |

Methods:

- `set_motor(target_velocity: float, max_torque: float)`
- `disable_motor()`
- `get_angle()` -> `float`

#### SliderJoint3D

Inherits `Joint3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Linear Limit** (group) | | | |
| `linear_limit_enable` (`linear_limit/enable`) | bool |  | False |
| `linear_limit_upper_distance` (`linear_limit/upper_distance`) | float | -1024 .. 1024 (unbounded); step 0.01 | 1 |
| `linear_limit_lower_distance` (`linear_limit/lower_distance`) | float | -1024 .. 1024 (unbounded); step 0.01 | -1 |
| **Motor** (group) | | | |
| `motor_enable` (`motor/enable`) | bool |  | False |
| `motor_target_velocity` (`motor/target_velocity`) | float | -1024 .. 1024 (unbounded); step 0.01 | 0 |
| `motor_force_limit` (`motor/force_limit`) | float | 0 .. 100000 (no upper bound); step 0.01 | 0 |

Methods:

- `set_motor(target_velocity: float, max_force: float)`
- `disable_motor()`
- `get_position()` -> `float`

#### DistanceJoint3D

Inherits `Joint3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `offset_b` | Vec3 |  | Vec3(0, 0, 0) |
| `distance_minimum` | float | 0 .. 4096 (no upper bound); step 0.01 | 0 |
| `distance_maximum` | float | 0 .. 4096 (no upper bound); step 0.01 | 0 |
| `custom_distance` | bool |  | False |

Methods:

- `set_distance(minimum: float, maximum: float)`

#### FixedJoint3D

Inherits `Joint3D`.

No properties of its own.

No methods of its own.

#### PistonJoint3D

Inherits `Joint3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Linear Limit** (group) | | | |
| `linear_limit_enabled` (`linear_limit/enabled`) | bool |  | False |
| `linear_limit_upper_distance` (`linear_limit/upper_distance`) | float | -1024 .. 1024 (unbounded); step 0.01 | 1 |
| `linear_limit_lower_distance` (`linear_limit/lower_distance`) | float | -1024 .. 1024 (unbounded); step 0.01 | -1 |
| **Angular Limit** (group) | | | |
| `angular_limit_enabled` (`angular_limit/enabled`) | bool |  | False |
| `angular_limit_upper_angle` (`angular_limit/upper_angle`) | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians |  |
| `angular_limit_lower_angle` (`angular_limit/lower_angle`) | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians |  |
| **Linear Motor** (group) | | | |
| `linear_motor_enabled` (`linear_motor/enabled`) | bool |  | False |
| `linear_motor_target_velocity` (`linear_motor/target_velocity`) | float | -1024 .. 1024 (unbounded); step 0.01 | 0 |
| `linear_motor_force_limit` (`linear_motor/force_limit`) | float | 0 .. 100000 (no upper bound); step 0.01 | 0 |
| **Angular Motor** (group) | | | |
| `angular_motor_enabled` (`angular_motor/enabled`) | bool |  | False |
| `angular_motor_target_velocity` (`angular_motor/target_velocity`) | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians | 0 |
| `angular_motor_force_limit` (`angular_motor/force_limit`) | float | 0 .. 100000 (no upper bound); step 0.01 | 0 |

Methods:

- `set_linear_limit(lower: float, upper: float)`
- `set_angular_limit(lower: float, upper: float)`
- `set_linear_motor(target_velocity: float, max_force: float)`
- `set_angular_motor(target_velocity: float, max_torque: float)`
- `disable_linear_motor()`
- `disable_angular_motor()`
- `get_position()` -> `float`
- `get_angle()` -> `float`

#### UniversalJoint3D

Inherits `Joint3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Angular Limit X** (group) | | | |
| `angular_limit_x_enabled` (`angular_limit_x/enabled`) | bool |  | False |
| `angular_limit_x_upper_angle` (`angular_limit_x/upper_angle`) | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians |  |
| `angular_limit_x_lower_angle` (`angular_limit_x/lower_angle`) | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians |  |
| **Angular Limit Y** (group) | | | |
| `angular_limit_y_enabled` (`angular_limit_y/enabled`) | bool |  | False |
| `angular_limit_y_upper_angle` (`angular_limit_y/upper_angle`) | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians |  |
| `angular_limit_y_lower_angle` (`angular_limit_y/lower_angle`) | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians |  |
| **Angular Motor X** (group) | | | |
| `angular_motor_x_enabled` (`angular_motor_x/enabled`) | bool |  | False |
| `angular_motor_x_target_velocity` (`angular_motor_x/target_velocity`) | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians | 0 |
| `angular_motor_x_force_limit` (`angular_motor_x/force_limit`) | float | 0 .. 100000 (no upper bound); step 0.01 | 0 |
| **Angular Motor Y** (group) | | | |
| `angular_motor_y_enabled` (`angular_motor_y/enabled`) | bool |  | False |
| `angular_motor_y_target_velocity` (`angular_motor_y/target_velocity`) | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians | 0 |
| `angular_motor_y_force_limit` (`angular_motor_y/force_limit`) | float | 0 .. 100000 (no upper bound); step 0.01 | 0 |

Methods:

- `set_limit_x(lower: float, upper: float)`
- `set_limit_y(lower: float, upper: float)`
- `set_motor_x(target_velocity: float, max_torque: float)`
- `set_motor_y(target_velocity: float, max_torque: float)`
- `disable_motor_x()`
- `disable_motor_y()`
- `get_angle_x()` -> `float`
- `get_angle_y()` -> `float`

#### WheelJoint3D

Inherits `Joint3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Suspension** (group) | | | |
| `suspension_rest_length` (`suspension/rest_length`) | float | 0 .. 100 (no upper bound); step 0.001 | 0 |
| `suspension_stiffness` (`suspension/stiffness`) | float | 0 .. 100000 (no upper bound); step 0.01 | 2000 |
| `suspension_damping` (`suspension/damping`) | float | 0 .. 100000 (no upper bound); step 0.01 | 200 |
| **Steering Limit** (group) | | | |
| `steering_limit_enabled` (`steering_limit/enabled`) | bool |  | False |
| `steering_limit_upper_angle` (`steering_limit/upper_angle`) | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians |  |
| `steering_limit_lower_angle` (`steering_limit/lower_angle`) | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians |  |
| **Steering Servo** (group) | | | |
| `steering_servo_enabled` (`steering_servo/enabled`) | bool |  | False |
| `steering_servo_target_angle` (`steering_servo/target_angle`) | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians | 0 |
| `steering_servo_force_limit` (`steering_servo/force_limit`) | float | 0 .. 100000 (no upper bound); step 0.01 | 0 |
| `steering_servo_max_angular_velocity` (`steering_servo/max_angular_velocity`) | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians | 0 |
| **Spin Motor** (group) | | | |
| `spin_motor_enabled` (`spin_motor/enabled`) | bool |  | False |
| `spin_motor_target_velocity` (`spin_motor/target_velocity`) | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians | 0 |
| `spin_motor_force_limit` (`spin_motor/force_limit`) | float | 0 .. 100000 (no upper bound); step 0.01 | 0 |

Methods:

- `set_steering_limit(lower: float, upper: float)`
- `set_steering_servo(target_angle: float, max_torque: float, max_angular_velocity: float)`
- `disable_steering_servo()`
- `set_spin_motor(target_velocity: float, max_torque: float)`
- `disable_spin_motor()`
- `get_steering_angle()` -> `float`
- `get_suspension_travel()` -> `float`

#### ConeTwistJoint3D

Inherits `Joint3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `swing_limit_y` | float | 0 .. 180 degrees in the editor; step 0.1; value in radians | 3.141592653589793 |
| `swing_limit_z` | float | 0 .. 180 degrees in the editor; step 0.1; value in radians | 3.141592653589793 |
| `twist_lower` | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians |  |
| `twist_upper` | float | -180 .. 180 (unbounded) degrees in the editor; step 0.1; value in radians | 3.141592653589793 |

No methods of its own.

#### MouseJoint3D

Inherits `Joint3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `target` | Vec3 |  | Vec3(0, 0, 0) |
| `use_target` | bool |  | False |
| **Spring** (group) | | | |
| `spring_max_force` (`spring/max_force`) | float | 0 .. 1000000 (no upper bound); step 0.01 | 500 |
| `spring_frequency` (`spring/frequency`) | float | 0 .. 1000 (no upper bound); step 0.01 | 5 |
| `spring_damping_ratio` (`spring/damping_ratio`) | float | 0 .. 100 (no upper bound); step 0.01 | 0.7 |

Methods:

- `set_target(global_target: Vec3)`

### 4.9 Vehicles

#### GroundVehicle3D

Inherits `RigidBody3D`. A passenger car on the physics' ground vehicle (1400 kg, 2.6 m wheelbase, 100 kW, five gears, tyre model, drivetrain with differentials). The body gets its own box from the car's layout; collision shapes added as children are ignored. Body axes: +Z forward, +Y up, +X left. Wheels: 0 front left, 1 front right, 2 rear left, 3 rear right.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Vehicle** (group) | | | |
| `drive` | int | 0=Front Wheel, 1=Rear Wheel, 2=All Wheel | 1 (Rear Wheel) |
| `automatic` | bool |  | True |
| `anti_lock` | bool |  | False |
| `wheel_ray_mask` | int | 3D physics layers | all |
| `wheel_radius` | float | 0 .. 2; step 0.001; 0 keeps the car's 0.31 m | 0 |
| `power` | float | 0.01 .. 10 (no upper bound); step 0.01; scales the engine's torque curve and the clutch capacity with it | 1 |
| `grip` | float | 0.01 .. 5 (no upper bound); step 0.01; scales the friction of the ground under the tyres | 1 |
| `roll_influence` | float | 0 .. 1; step 0.01; how much of the height of the tyres' side force reaches the body: 1 applies it at the contact patch, 0 at the height of the centre of mass, so a car with a lot of grip does not roll over in a turn | 1 |
| `top_speed_kmh` | float | 0 .. 1000; step 1; 0 keeps the car's gearing. Otherwise the final drive is set so that the top gear reaches this speed at the engine's limiter (limiter rpm x wheel circumference / top gear ratio); whether the engine pulls it against drag is up to `power` | 0 |
| `handbrake_torque` | float | 0 .. 20000 (no upper bound); step 10; N m on each wheel with a handbrake, 0 keeps the car's 1500 | 0 |
| **Controls** (group) | | | |
| `throttle` | float | 0 .. 1; step 0.01 | 0 |
| `brake` | float | 0 .. 1; step 0.01 | 0 |
| `handbrake` | float | 0 .. 1; step 0.01 | 0 |
| `steering` | float | -1 .. 1; step 0.01 | 0 |
| **Wheels** (group) | | | |
| `wheel_front_left` | NodePath (Node3D) |  |  |
| `wheel_front_right` | NodePath (Node3D) |  |  |
| `wheel_rear_left` | NodePath (Node3D) |  |  |
| `wheel_rear_right` | NodePath (Node3D) |  |  |
| `skid_marks` | NodePath (SkidMarks3D) |  |  |
| `collision_model` | String (file) | *.glb, *.gltf, *.pmesh, *.obj | "" |
| `collision_scale` | Vec3 |  | Vec3(1, 1, 1) |
| `collision_offset` | Vec3 |  | Vec3(0, 0, 0) |

Where the four wheel nodes stand when the car is built sets its axles: each axle at the middle of its two wheels along Z, its half track the mean of their distances along X. With `wheel_radius` the car rides higher by the difference to 0.31 m; the wheel centres stay where they were relative to the body. The car starts in neutral with the handbrake held, and engages first gear on the first throttle or brake. Holding the brake below 0.3 m/s selects reverse, where the throttle and the brake swap; the throttle selects drive again. `steering` +1 turns right. With `skid_marks` set, every wheel on the ground lays a mark in that node as dark as it slides beyond the peak grip of its tyre (`get_wheel_slide`). With `collision_model` the body collides as the convex hull of that model's vertices, each scaled by `collision_scale` and moved by `collision_offset` into the frame of the car, instead of the box; the inertia stays the box's, so the handling does not change. A `CarDamage3D` that crushes the nose or the tail clips the hull there. The four wheel nodes are placed on the wheels every physics step (axle along +X, spin included) and hidden when a wheel comes off.

Methods:

- `get_collision_half_extents()` -> `Vec3`
- `get_rest_height()` -> `float` - Height of the centre of mass above flat ground at rest.
- `get_speed_kmh()` -> `float` - Forward speed, negative in reverse.
- `get_forward_speed()` -> `float` - m/s.
- `get_rpm()` -> `float`
- `get_gear()` -> `int` - 0 neutral or shifting, -1 reverse.
- `get_wheel_count()` -> `int`
- `is_wheel_in_contact(wheel: int)` -> `bool`
- `get_wheel_slip_ratio(wheel: int)` -> `float`
- `get_wheel_slip_angle(wheel: int)` -> `float` - rad.
- `get_wheel_slide(wheel: int)` -> `float` - How far past the tyre's grip the wheel is, 0 inside the grip to 1 fully sliding, 0 when the wheel is off the ground. The skid marks use this value; a skid sound is the script's to play from it.
- `get_wheel_radius(wheel: int)` -> `float`
- `is_wheel_detached(wheel: int)` -> `bool`
- `set_wheel_detached(wheel: int, detached: bool)` - The wheel no longer touches the ground nor carries load.
- `set_wheel_radius_scale(wheel: int, scale: float)` - A burst tyre: the wheel rolls on its rim.
- `stop_wheels()` - Stops the wheel spin, after the car was moved by hand.
- `park()` - Neutral and the handbrake on.
- `set_axle_grip(axle: int, scale: float)` - Scales the peak friction of the tyres of one axle (0 front, 1 rear), on top of `grip`.
- `is_reversing()` -> `bool`

#### VehicleBody3D

Inherits `RigidBody3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Motion** (group) | | | |
| `engine_force` | float |  | 0 |
| `brake` | float | 0 .. 1000 (no upper bound); step 0.01 | 0 |
| `steering` | float | -1.5708 .. 1.5708 (unbounded); step 0.001 | 0 |
| **Motorcycle** (group) | | | |
| `motorcycle_enabled` | bool |  | False |
| `motorcycle_steer_input` | float | -1 .. 1 (unbounded); step 0.01 | 0 |
| `motorcycle_max_steer_angle` | float | 0 .. 90 (no upper bound) degrees in the editor; step 0.1; value in radians | 0.7 |
| `motorcycle_caster_angle` | float | -90 .. 90 (unbounded) degrees in the editor; step 0.1; value in radians | 0 |
| `motorcycle_max_lean_angle` | float | 0 .. 90 (no upper bound) degrees in the editor; step 0.1; value in radians | 0.7853982 |
| `motorcycle_lean_spring` | float | 0 .. 1000000 (no upper bound); step 0.1 | 5000 |
| `motorcycle_lean_damping` | float | 0 .. 1000000 (no upper bound); step 0.1 | 1000 |
| `motorcycle_lean_controller_enabled` | bool |  | True |

Methods:

- `get_speed_kmh()` -> `float`
- `get_wheel_count()` -> `int`
- `get_wheel(index: int)` -> `VehicleWheel3D`
- `get_lean_angle()` -> `float`

#### VehicleWheel3D

Inherits `Node3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Per-Wheel Motion** (group) | | | |
| `engine_force` | float |  | 0 |
| `brake` | float | 0 .. 1000 (no upper bound); step 0.01 | 0 |
| `steering` | float | -1.5708 .. 1.5708 (unbounded); step 0.001 | 0 |
| `use_as_traction` | bool |  | False |
| `use_as_steering` | bool |  | False |
| **Wheel** (group) | | | |
| `wheel_roll_influence` | float | 0 .. 1 (no upper bound); step 0.01 | 0.1 |
| `wheel_radius` | float | 0.001 .. 100 (no upper bound); step 0.001 | 0.5 |
| `wheel_rest_length` | float | 0 .. 100 (no upper bound); step 0.001 | 0.15 |
| `wheel_friction_slip` | float | 0 .. 1000 (no upper bound); step 0.01 | 10.5 |
| **Suspension** (group) | | | |
| `suspension_travel` | float | 0 .. 100 (no upper bound); step 0.001 | 5 |
| `suspension_stiffness` | float | 0 .. 1000 (no upper bound); step 0.01 | 5.88 |
| `suspension_max_force` | float | 0 .. 100000 (no upper bound); step 0.01 | 6000 |
| **Damping** (group) | | | |
| `damping_compression` | float | 0 .. 10 (no upper bound); step 0.01 | 0.83 |
| `damping_relaxation` | float | 0 .. 10 (no upper bound); step 0.01 | 0.88 |

Methods:

- `is_in_contact()` -> `bool`
- `get_contact_body()` -> `PhysicsBody3D`
- `get_contact_point()` -> `Vec3`
- `get_contact_normal()` -> `Vec3`
- `get_skid_info()` -> `float`
- `get_rpm()` -> `float`

### 4.10 Soft bodies and destructibles

#### SoftBody3D

Inherits `MeshInstance3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `mass` | float | 0.001 .. 1000 (no upper bound); step 0.001 |  |
| `structural_compliance` | float | 0 .. 1 (no upper bound); step 0.000001 |  |
| `bend_compliance` | float | 0 .. 1 (no upper bound); step 0.000001 |  |
| `bend_constraint` | int | 0=Distance, 1=Dihedral |  |
| `pressure` | float | 0 .. 1000 (no upper bound); step 0.01 |  |
| `substeps` | int | 1 .. 64 (no upper bound); step 1 |  |
| `damping` | float | 0 .. 1; step 0.001 |  |
| `collision_margin` | float | 0 .. 1 (no upper bound); step 0.001 |  |
| **Collision** (group) | | | |
| `collision_layer` | int | bitmask of physics layers |  |
| `collision_mask` | int | bitmask of physics layers |  |
| **Simulation** (group) | | | |
| `attachment_limit` | float | 0 .. 100 (no upper bound); step 0.01 | 1 |
| `wind` | Vec3 |  | Vec3(0, 0, 0) |
| `simulation_enabled` | bool |  | True |

Methods:

- `make_cloth(width: float, depth: float, segments_x: int, segments_z: int, uv_tiles: float = 1.0)`
- `make_ball(radius: float, rings: int = 12, slices: int = 18)`
- `set_pinned(vertex: int, pinned: bool)`
- `is_pinned(vertex: int)` -> `bool`
- `pin_in_sphere(local_center: Vec3, radius: float, pinned: bool = True)`
- `pin_in_box(minimum: Vec3, maximum: Vec3, pinned: bool = True)` - The box is given in the soft body local space.
- `clear_pins()`
- `add_velocity(delta: Vec3)`
- `reset()`
- `is_built()` -> `bool`
- `get_particle_count()` -> `int`
- `get_particle_position(vertex: int)` -> `Vec3`
- `get_worst_stretch()` -> `float`

#### DestructibleBody3D

Inherits `RigidBody3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `fragment_count` | int | 2 .. 256 (no upper bound); step 1 | 12 |
| `break_speed` | float | 0 .. 1000 (no upper bound); step 0.01 | 0 |
| `density` | float | 0.001 .. 100000 (no upper bound); step 0.001 | 500 |
| `debris_lifetime` | float | 0 .. 3600 (no upper bound); step 0.01 | 8 |
| `push_speed` | float | 0 .. 1000 (no upper bound); step 0.01 | 2 |
| `seed` | int |  | 2463534242 |
| `outer_material` | ResourcePath (string) | resource path, type Material |  |
| `inner_material` | ResourcePath (string) | resource path, type Material |  |

Methods:

- `break_at(global_point: Vec3, impulse: Vec3 = Vec3(0, 0, 0))` -> `bool`
- `is_broken()` -> `bool`

#### DebrisBody3D

Inherits `RigidBody3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `lifetime` | float | 0 .. 3600 (no upper bound); step 0.01 | 0 |

Methods:

- `get_age()` -> `float`

#### DebrisMesh3D

Inherits `MeshInstance3D`.

No properties of its own.

No methods of its own.

### 4.11 Characters

#### CharacterBody3D

Inherits `PhysicsBody3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `velocity` | Vec3 |  | Vec3(0, 0, 0) |
| `up_direction` | Vec3 |  |  |
| `max_slides` | int | 1 .. 256 (no upper bound); step 1 |  |
| `radius` | float | 0.001 .. 100 (no upper bound); step 0.001 |  |
| `height` | float | 0 .. 100 (no upper bound); step 0.001 |  |
| **Floor** (group) | | | |
| `floor_stop_on_slope` | bool |  |  |
| `slope_limit` | float | 0 .. 90; step 0.1 |  |
| `step_offset` | float | 0 .. 10 (no upper bound); step 0.001 |  |
| `ground_snap` | float | 0 .. 10 (no upper bound); step 0.001 |  |
| **Wall** (group) | | | |
| `wall_min_slide_angle` | float | 0 .. 180; step 0.1 |  |
| `slide_on_ceiling` | bool |  |  |
| **Collision** (group) | | | |
| `skin_width` | float | 0.0001 .. 1 (no upper bound); step 0.0001 |  |
| `push_force` | float | 0 .. 10000 (no upper bound); step 0.01 | 0 |

Methods:

- `set_velocity(velocity: Vec3)`
- `get_velocity()` -> `Vec3`
- `move_and_slide(delta_time: float = 0.0)` -> `bool` - A delta of 0 (or less) uses the physics step time (1/60 s when that is not available). Returns False when the body is not inside the scene tree.
- `is_on_floor()` -> `bool`
- `is_on_wall()` -> `bool`
- `is_on_ceiling()` -> `bool`
- `get_floor_normal()` -> `Vec3`
- `get_wall_normal()` -> `Vec3`
- `get_ceiling_normal()` -> `Vec3`
- `get_floor_angle()` -> `float`
- `is_near_ground(distance: float)` -> `bool`
- `get_slide_collision_count()` -> `int`
- `get_slide_collider(index: int)` -> `PhysicsBody3D`
- `get_slide_normal(index: int)` -> `Vec3`

#### CharacterRigidBody3D

Inherits `RigidBody3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `max_slope_angle` | float | 0 .. 90; step 0.1 |  |
| `supporting_height` | float | 0 .. 10 (no upper bound); step 0.001 |  |
| `max_separation` | float | 0 .. 10 (no upper bound); step 0.001 |  |
| `up_direction` | Vec3 |  |  |

Methods:

- `probe_ground()`
- `get_ground_state()` -> `int (GROUND_STATE_*)`
- `is_on_ground()` -> `bool`
- `is_supported()` -> `bool`
- `get_ground_normal()` -> `Vec3`
- `get_ground_position()` -> `Vec3`
- `get_ground_velocity()` -> `Vec3`
- `get_ground_body()` -> `PhysicsBody3D`

### 4.12 Skeleton and animation

#### Skeleton3D

Inherits `Node3D`. The `SkeletonModifier3D` children of a skeleton (in child order) run every time the skeleton is updated: the pose is saved, each active modifier changes it (blended by its `influence`), the result is sent to the renderer and to the bone attachments, and the saved pose is restored. Reading a bone pose afterwards gives the unmodified pose.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `modifier_callback_mode_process` | int | 0=Physics, 1=Idle, 2=Manual | 0 (Physics) |

Methods:

- `advance(delta: float)` - Adds `delta` seconds to the time the modifiers receive in their next update and schedules that update. Called by the skeleton itself every physics frame (Physics) or every frame (Idle); call it from a script in Manual mode. Does nothing when the skeleton has no modifier.
- `clear_bones()`
- `get_bone_count()` -> `int`
- `find_bone(name: string)` -> `int`
- `get_bone_name(bone: int)` -> `string`
- `get_bone_parent(bone: int)` -> `int`
- `set_bone_pose_position(bone: int, position: Vec3)`
- `set_bone_pose_rotation(bone: int, rotation: Quat)`
- `set_bone_pose_scale(bone: int, scale: Vec3)`
- `get_bone_pose_position(bone: int)` -> `Vec3`
- `get_bone_pose_rotation(bone: int)` -> `Quat`
- `get_bone_pose_scale(bone: int)` -> `Vec3`
- `reset_bone_pose(bone: int)`
- `reset_bone_poses()`
- `force_update_all_bone_transforms()`
- `update_skeleton()`
- `is_dirty()` -> `bool`
- `get_pose_version()` -> `int`

#### BoneAttachment3D

Inherits `Node3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `bone_name` | string |  |  |
| `bone_idx` | int |  | -1 |
| `override_pose` | bool |  | False |
| `use_external_skeleton` | bool |  | False |
| `external_skeleton` | NodePath (string) | node path, valid types Skeleton3D |  |

Methods:

- `get_skeleton()` -> `Skeleton3D`

#### SkeletonModifier3D

Inherits `Node3D`. Abstract. Must be a child of a `Skeleton3D`; it changes the bone poses each time the skeleton is updated (see `Skeleton3D`).

| Property | Type | Range / values | Default |
|---|---|---|---|
| `active` | bool |  | True |
| `influence` | float | 0 .. 1; step 0.001 | 1 |

Methods:

- `get_skeleton()` -> `Skeleton3D` - The parent skeleton, `None` while the modifier is outside the tree or its parent is not a skeleton.

#### SpringBoneSimulator3D

Inherits `SkeletonModifier3D`. Spring bone physics for bone chains (hair, cloth, tails). It never moves the skeleton node: it rotates the bones of each chain so that the chain trails behind the motion of the skeleton, with the length of each bone kept.

A simulator holds a list of settings; each setting is one chain from a root bone to an end bone. Setting indices run from 0 to `setting_count - 1`; an index out of range logs an error and the call does nothing. Child `SpringBoneCollision3D` nodes of the simulator push the chains away.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `external_force` | Vec3 |  | Vec3(0, 0, 0) |
| `mutable_bone_axes` | bool | when true the direction and length of each bone follow its current pose, when false its rest pose | True |
| `setting_count` | int | 0 .. 4096 (no upper bound); not stored in scene files | 0 |

The settings are stored in the `data` of the node in scene files:

```json
"data": {
  "version": 1,
  "settings": [
    {
      "root_bone_name": "hair1", "end_bone_name": "hair4",
      "extend_end_bone": true, "end_bone_direction": 2, "end_bone_length": 0.1,
      "center_from": 0, "center_node": "", "center_bone_name": "",
      "individual_config": false,
      "rotation_axis": 3, "rotation_axis_vector": [1, 0, 0],
      "radius": 0.02, "radius_damping_curve": [1.0, 0.5],
      "stiffness": 1.0, "drag": 0.4, "gravity": 0.0, "gravity_direction": [0, -1, 0],
      "enable_all_child_collisions": true, "exclude_collisions": ["Ball"]
    }
  ]
}
```

`root_bone` / `end_bone` / `center_bone` (bone indices) replace the matching `*_name` when the name is missing. `end_bone_direction` is a `BONE_DIRECTION_*` value, `center_from` a `CENTER_FROM_*` value, `rotation_axis` a `ROTATION_AXIS_*` value. The damping curves (`radius_damping_curve`, `stiffness_damping_curve`, `drag_damping_curve`, `gravity_damping_curve`) are lists of evenly spaced samples along the chain (root to end), interpolated linearly, and multiply the value of the setting; they are only used when `individual_config` is false. With `individual_config` true the setting also stores `joints`, a list with one object per joint (`rotation_axis`, `rotation_axis_vector`, `radius`, `stiffness`, `drag`, `gravity`, `gravity_direction`). Without `enable_all_child_collisions` the setting lists `collisions` (node paths relative to the simulator) instead of `exclude_collisions`.

Methods:

- `add_setting(root_bone_name: string, end_bone_name: string)` -> `int` - Appends a setting and returns its index. The bone names are resolved when the simulator is in a skeleton; the end bone must be the root bone or a descendant of it.
- `clear_settings()`
- `reset()` - Restarts the simulation of every chain from the current pose.
- `get_bone_vector(index: int, joint: int)` -> `Vec3`
- `set_setting_root_bone_name(index: int, bone_name: string)`
- `get_setting_root_bone_name(index: int)` -> `string`
- `set_setting_root_bone(index: int, bone: int)`
- `get_setting_root_bone(index: int)` -> `int`
- `set_setting_end_bone_name(index: int, bone_name: string)`
- `get_setting_end_bone_name(index: int)` -> `string`
- `set_setting_end_bone(index: int, bone: int)`
- `get_setting_end_bone(index: int)` -> `int`
- `set_setting_extend_end_bone(index: int, enabled: bool)` - Adds a virtual joint after the end bone, `end_bone_length` long, so the end bone moves too.
- `get_setting_extend_end_bone(index: int)` -> `bool`
- `set_setting_end_bone_direction(index: int, bone_direction: int (BONE_DIRECTION_*))`
- `get_setting_end_bone_direction(index: int)` -> `int`
- `set_setting_end_bone_length(index: int, length: float)`
- `get_setting_end_bone_length(index: int)` -> `float`
- `set_setting_center_from(index: int, center_from: int (CENTER_FROM_*))` - The space the chain is simulated in: the world, another node or a bone. A moving center is not treated as movement of the chain.
- `get_setting_center_from(index: int)` -> `int`
- `set_setting_center_node(index: int, node_path: string)`
- `get_setting_center_node(index: int)` -> `string`
- `set_setting_center_bone_name(index: int, bone_name: string)`
- `get_setting_center_bone_name(index: int)` -> `string`
- `set_setting_center_bone(index: int, bone: int)`
- `get_setting_center_bone(index: int)` -> `int`
- `set_setting_rotation_axis(index: int, axis: int (ROTATION_AXIS_*))` - Limits the rotation of the joints to one axis (ignored while `individual_config` is on).
- `get_setting_rotation_axis(index: int)` -> `int`
- `set_setting_rotation_axis_vector(index: int, vector: Vec3)` - Used when the axis is `ROTATION_AXIS_CUSTOM`.
- `get_setting_rotation_axis_vector(index: int)` -> `Vec3`
- `set_setting_radius(index: int, radius: float)` - Radius of the joints for collisions.
- `get_setting_radius(index: int)` -> `float`
- `set_setting_stiffness(index: int, stiffness: float)` - How strongly each bone returns to its pose.
- `get_setting_stiffness(index: int)` -> `float`
- `set_setting_drag(index: int, drag: float)` - How much of the speed of the previous step is lost, 0 .. 1.
- `get_setting_drag(index: int)` -> `float`
- `set_setting_gravity(index: int, gravity: float)`
- `get_setting_gravity(index: int)` -> `float`
- `set_setting_gravity_direction(index: int, gravity_direction: Vec3)` - A zero vector is rejected.
- `get_setting_gravity_direction(index: int)` -> `Vec3`
- `set_setting_individual_config(index: int, enabled: bool)` - With true every joint has its own values (`set_joint_*`) and the values of the setting are ignored.
- `get_setting_individual_config(index: int)` -> `bool`
- `get_joint_count(index: int)` -> `int` - Joints are the bones from the root bone to the end bone, found when the simulator is in a skeleton.
- `get_joint_bone_name(index: int, joint: int)` -> `string`
- `get_joint_bone(index: int, joint: int)` -> `int`
- `set_joint_rotation_axis(index: int, joint: int, axis: int (ROTATION_AXIS_*))`
- `get_joint_rotation_axis(index: int, joint: int)` -> `int`
- `set_joint_rotation_axis_vector(index: int, joint: int, vector: Vec3)`
- `get_joint_rotation_axis_vector(index: int, joint: int)` -> `Vec3`
- `set_joint_radius(index: int, joint: int, radius: float)`
- `get_joint_radius(index: int, joint: int)` -> `float`
- `set_joint_stiffness(index: int, joint: int, stiffness: float)`
- `get_joint_stiffness(index: int, joint: int)` -> `float`
- `set_joint_drag(index: int, joint: int, drag: float)`
- `get_joint_drag(index: int, joint: int)` -> `float`
- `set_joint_gravity(index: int, joint: int, gravity: float)`
- `get_joint_gravity(index: int, joint: int)` -> `float`
- `set_joint_gravity_direction(index: int, joint: int, gravity_direction: Vec3)`
- `get_joint_gravity_direction(index: int, joint: int)` -> `Vec3`
- `set_enable_all_child_collisions(index: int, enabled: bool)` - True: the chain collides with every `SpringBoneCollision3D` child except the excluded ones. False: only with the listed ones.
- `are_all_child_collisions_enabled(index: int)` -> `bool`
- `set_exclude_collision_path(index: int, collision: int, node_path: string)` - The path must lead to a collision child of the simulator. Only used while all child collisions are enabled.
- `get_exclude_collision_path(index: int, collision: int)` -> `string`
- `set_exclude_collision_count(index: int, count: int)`
- `get_exclude_collision_count(index: int)` -> `int`
- `clear_exclude_collisions(index: int)`
- `set_collision_path(index: int, collision: int, node_path: string)` - Only used while all child collisions are disabled.
- `get_collision_path(index: int, collision: int)` -> `string`
- `set_collision_count(index: int, count: int)`
- `get_collision_count(index: int)` -> `int`
- `clear_collisions(index: int)`
- `get_valid_collisions(index: int)` -> `list of SpringBoneCollision3D` - The collisions the chain collides with.

#### SpringBoneCollision3D

Inherits `Node3D`. Abstract. Collision shape for the chains of a `SpringBoneSimulator3D`; it must be a child of the simulator. With a bone set, the node follows that bone (plus the offsets) every update; without a bone its own transform is used.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `bone_name` | string | bone names of the skeleton | "" |
| `bone` | int | not shown in the editor | -1 |
| `position_offset` | Vec3 | hidden by the editor while no bone is set |  |
| `rotation_offset` | Quat | hidden by the editor while no bone is set | Quat(0, 0, 0, 1) |

Methods:

- `get_skeleton()` -> `Skeleton3D`
- `sync_pose()` - Moves the node to the bone.

#### SpringBoneCollisionSphere3D

Inherits `SpringBoneCollision3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `radius` | float | 0 .. 1 (no upper bound); step 0.001 | 0.1 |
| `inside` | bool | keeps the chains inside the sphere | False |

No methods of its own.

#### SpringBoneCollisionCapsule3D

Inherits `SpringBoneCollision3D`. The capsule stands along the local Y axis.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `radius` | float | 0 .. 1 (no upper bound); step 0.001 | 0.1 |
| `height` | float | 0 .. 1 (no upper bound); step 0.001 | 0.5 |
| `inside` | bool | keeps the chains inside the capsule | False |

Setting the radius larger than half the height raises the height, and setting the height lower than twice the radius lowers the radius.

Methods:

- `set_mid_height(mid_height: float)` - Height of the straight part: `height - 2 * radius`. A negative value is rejected.
- `get_mid_height()` -> `float`

#### SpringBoneCollisionPlane3D

Inherits `SpringBoneCollision3D`. An infinite plane through the node with the local +Y axis as its normal; the chains stay on the side of the normal.

No properties of its own.

No methods of its own.

#### Ragdoll3D

Inherits `Node3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `skeleton` | NodePath (string) | node path, valid types Skeleton3D |  |
| `animation_player` | NodePath (string) | node path, valid types AnimationPlayer |  |
| `blend` | float | 0 .. 1; step 0.01 | 1 |
| `collision_mask` | int | bitmask of physics layers | 4294967295 |
| `ragdoll_scale` | float | 0 .. 100 (no upper bound); step 0.001 | 0 |
| `fatness` | float | 0.01 .. 10 (no upper bound); step 0.01 | 1 |
| `head_size` | float | 0.01 .. 10 (no upper bound); step 0.01 | 1 |
| **Bones** (group) | | | |
| `bones_hips` (`bones/hips`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_spine` (`bones/spine`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_neck` (`bones/neck`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_head` (`bones/head`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_left_arm` (`bones/left_arm`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_left_fore_arm` (`bones/left_fore_arm`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_left_hand` (`bones/left_hand`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_right_arm` (`bones/right_arm`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_right_fore_arm` (`bones/right_fore_arm`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_right_hand` (`bones/right_hand`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_left_up_leg` (`bones/left_up_leg`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_left_leg` (`bones/left_leg`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_left_foot` (`bones/left_foot`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_left_toes` (`bones/left_toes`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_right_up_leg` (`bones/right_up_leg`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_right_leg` (`bones/right_leg`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_right_foot` (`bones/right_foot`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |
| `bones_right_toes` (`bones/right_toes`) | int | -1 .. 65535 (no upper bound); step 1 | -1 |

Methods:

- `get_skeleton()` -> `Skeleton3D`
- `get_animation_player()` -> `AnimationPlayer`
- `set_bone(slot: int (RAGDOLL_BONE_*), bone: int)`
- `get_bone(slot: int (RAGDOLL_BONE_*))` -> `int`
- `set_bone_name(slot: int (RAGDOLL_BONE_*), name: string)` -> `bool`
- `auto_map_bones()` -> `int`
- `build()` -> `bool`
- `is_built()` -> `bool`
- `start()` -> `bool`
- `stop()`
- `is_active()` -> `bool`
- `get_facing()` -> `float`
- `get_part_bone(part: int (RAGDOLL_PART_*))` -> `int`
- `get_part_mass(part: int (RAGDOLL_PART_*))` -> `float`
- `apply_impulse(part: int (RAGDOLL_PART_*), impulse: Vec3, offset: Vec3 = Vec3(0, 0, 0))`
- `apply_blast(origin: Vec3, speed: float)`: gives every part an impulse of `speed` m/s times its mass, pointing from `origin` to the part
- `get_part_velocity(part: int (RAGDOLL_PART_*))` -> `Vec3`

#### Dismember3D

Inherits `Node3D`. Pulls a `Ragdoll3D` apart: each loose part flies on its own with no joint to its parent part. Each time a part comes loose, the skinned meshes of the ragdoll's skeleton are swapped for a cut copy of the original: every part that is still attached stays with the part it hangs from in the physics ragdoll (the head and the arms hang from the spine), each triangle goes to the group of the bone segment nearest to it in the bind pose, so the cut falls on a plane through each joint whatever the skin weights are, and keeps its blend weights inside that group (a vertex left without weights is bound to the nearest bone of its group), and vertices on a seam between groups are duplicated, so only the real cuts open and the joints of the body still bend smoothly; each cut is closed with a cap: every ring of the cut is followed in order, laid on its mean plane and filled by ear clipping between its own corners, on vertices of the group that carry its weights (the texture is the one of the cut). The original meshes come back when the node leaves the tree. Stopping the ragdoll joins the parts again.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `ragdoll` | NodePath (string) | node path, valid types Ragdoll3D |  |
| `radius` | float | 0 .. 100 (no upper bound); step 0.01 | 3 |
| `force` | float | 0 .. 1000 (no upper bound); step 0.01 | 8 |

Methods:

- `get_ragdoll()` -> `Ragdoll3D`
- `detach(part: int (RAGDOLL_PART_*))` -> `bool`: starts the ragdoll if needed and frees the part from its parent part; the pelvis has no parent and returns false
- `is_detached(part: int (RAGDOLL_PART_*))` -> `bool`
- `get_detached_count()` -> `int`
- `explode(origin: Vec3)` -> `bool`: starts the ragdoll if needed, frees every part and pushes each one away from `origin` at `force` m/s times its mass, fading to nothing at `radius` (0: no fade)

#### NavigationRegion3D

Inherits `Node3D`. The walkable surface of a level, for agents that walk. It is built from the static collision under `source` (the concave meshes and boxes of every `StaticBody3D`, such as a model loaded with `physics=concave`) the way Recast does: the triangles go into columns of solid spans, the top of a span is floor where an agent of `agent_height` fits under the next span above (so the floor under a desk is not), the floor is pulled back by `agent_radius` from walls and drops, and the cells left become the navigation mesh. With `seed`, only the floor that can be walked to from that node is kept. A node in the group `navigation_ignore` is left out with everything under it, so a door that can be opened or blown away does not cut the floor. With `bake_on_ready` the region bakes once the models being streamed in have loaded. The region joins the group `navigation`. Every query is in global space and searches 2 m sideways and 4 m up and down from the points it is given.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `source` | NodePath (string) | node path, valid types Node; empty: the parent |  |
| `seed` | NodePath (string) | node path, valid types Node3D |  |
| `cell_size` | float | 0.05 .. 2; step 0.01 | 0.2 |
| `agent_height` | float | 0.1 .. 10; step 0.01 | 1.8 |
| `agent_radius` | float | 0 .. 5; step 0.01 | 0.35 |
| `agent_max_climb` | float | 0 .. 5; step 0.01 | 0.4 |
| `agent_max_slope` | float | 0 .. 89; step 0.1 | 45 |
| `bake_on_ready` | bool |  | true |
| `show_debug` | bool |  | false |

Methods:

- `bake()` -> `bool`: collects the collision and builds the surface now
- `clear()`
- `is_baked()` -> `bool`
- `get_triangle_count()` -> `int`
- `find_path(from: Vec3, to: Vec3)` -> `list[Vec3]`: corners of the shortest path on the surface, from the floor below `from` to the floor nearest `to` (as close as it gets when `to` cannot be reached); empty when there is none
- `nearest_point(point: Vec3)` -> `Vec3` or `None`
- `move_along_surface(from: Vec3, to: Vec3)` -> `Vec3` or `None`: where an agent on the surface at `from` ends heading for `to`, stopped by the edges
- `raycast(from: Vec3, to: Vec3)` -> `float`: the fraction of the way along the surface before an edge, 1 when it is clear
- `random_point()` -> `Vec3` or `None`

#### AnimationPlayer

Inherits `Node`. While its skeleton is hidden (not visible in the tree) the player keeps its clock, queue and signals but does not pose the bones, so a hidden character costs no posing or skinning; it is posed again on the frame it shows. Skinned meshes no camera drew on the last frame are not skinned either until one does. The `animations` property is the editor's animation library (list of clips with play, stop, remove and import from file); assigning a model file path to it imports that file, the same as `import_animations(path)`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `root_node` | NodePath (string) |  | ".." |
| `skeleton` | NodePath (string) | node path, valid types Skeleton3D | ".." |
| `animations` | ResourcePath (string) | resource type AnimationLibrary; not stored in scene files; reads give the clip names joined by ", "; assigning a model file path (glb, gltf, fbx, b3d, ms3d, pmesh) imports its animations |  |
| `animation_files` | string | project paths of model files separated by `;`; their animations are imported every time the player enters the tree, in the editor and in the game, and are not written to the scene file |  |
| `autoplay` | string |  |  |
| `speed_scale` | float | -64 .. 64 (unbounded); step 0.01 | 1 |
| `playback_default_blend_time` | float | 0 .. 3600 (no upper bound); step 0.001 | 0 |
| `active` | bool |  | True |
| `callback_mode_process` | int | 0=Physics, 1=Idle, 2=Manual | 1 (Idle) |
| `deterministic` | bool |  | False |

Methods:

- `play(name: string = "", custom_blend: float = -1.0, custom_speed: float = 1.0, from_end: bool = False)` - An empty name plays the assigned animation. Runtime error when the animation does not exist.
- `stop(keep_state: bool = False)`
- `is_playing()` -> `bool`
- `import_animations(path: string, prefix: string = "")` -> `int` - Imports the animations of a model file (glb, gltf, fbx, b3d, ms3d or pmesh) into this player. Bone tracks are matched to the skeleton of the player by bone name and tracks with no matching bone are dropped; imported names get the prefix and a numeric suffix when the name is already taken. Returns the number of animations added, 0 when the file has none and -1 when the file cannot be loaded.
- `remove_animation(name: string)` -> `bool`
- `has_animation(name: string)` -> `bool`
- `get_animation_count()` -> `int`
- `get_animation_name(index: int)` -> `string`
- `get_animation_list()` -> `list of string` - Names of all animations.
- `get_skeleton()` -> `Skeleton3D`
- `set_animation_loop(animation: string, loop: bool)` -> `bool` - Makes the clip repeat (or play once). Imported clips start without looping.
- `get_animation_loop(animation: string)` -> `bool`
- `animation_set_next(animation: string, next: string)`
- `animation_get_next(animation: string)` -> `string`
- `set_blend_time(from: string, to: string, time: float)`
- `get_blend_time(from: string, to: string)` -> `float`
- `play_backwards(name: string = "", custom_blend: float = -1.0)`
- `queue(name: string)`
- `get_queue_count()` -> `int`
- `get_queued(index: int)` -> `string`
- `clear_queue()`
- `pause()`
- `get_current_animation()` -> `string`
- `get_assigned_animation()` -> `string`
- `get_playing_speed()` -> `float`
- `seek(time: float, update: bool = False)`
- `get_current_animation_position()` -> `float`
- `get_current_animation_length()` -> `float`
- `advance(delta: float)`

### 4.13 Audio

#### AudioListener3D

Inherits `Node3D`. The point 3D sounds are heard from. Without a current `AudioListener3D` the current camera of the scene is the listener, so 3D sounds follow the view and are not placed against the world origin.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `current` | bool |  |  |
| `velocity` | Vec3 |  | Vec3(0, 0, 0) |

Methods:

- `make_current()`
- `clear_current(enable_next: bool = True)`
- `get_current()` -> `AudioListener3D` - The current listener of the scene; None when there is none.

#### AudioStreamPlayer3D

Inherits `Node3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `stream` | ResourcePath (string) | resource path, type AudioStream |  |
| `attenuation_model` | int | 0=Inverse, 1=Linear, 2=Exponential, 3=Disabled |  |
| `volume_linear` | float | 0 .. 4; step 0.01 |  |
| `unit_size` | float | 0 .. 100 (no upper bound); step 0.01 |  |
| `pitch_scale` | float | 0.01 .. 4; step 0.01 |  |
| `autoplay` | bool |  | False |
| `loop` | bool |  |  |
| `max_distance` | float | 0 .. 4096 (no upper bound); step 0.01 |  |
| `rolloff` | float | 0 .. 100; step 0.01 |  |
| `bus` | int | 0=Sfx, 1=Music |  |
| `velocity` | Vec3 |  |  |

Methods:

- `play()` -> `bool`
- `stop()`
- `is_playing()` -> `bool`
- `get_playback_position()` -> `float` - Seconds into the sound being played (0 before it starts, after it stops, or without an audio device). Text or effects timed to a track follow it and stay in step even when frames stall.
- `seek(seconds: float)` -> `bool` - Jumps the playing sound to that time; False when nothing is playing.
- `pause()` -> `bool`
- `resume()` -> `bool`

#### AudioStreamPlayer

Inherits `Node`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `stream` | ResourcePath (string) | resource path, type AudioStream |  |
| `autoplay` | bool |  | False |
| `loop` | bool |  |  |
| `volume_linear` | float | 0 .. 4; step 0.01 |  |
| `pitch_scale` | float | 0.01 .. 4; step 0.01 |  |
| `pan` | float | -1 .. 1; step 0.01 |  |
| `bus` | int | 0=Sfx, 1=Music |  |

Methods:

- `play()` -> `bool`
- `stop()`
- `is_playing()` -> `bool`
- `get_playback_position()` -> `float` - Seconds into the sound being played (0 before it starts, after it stops, or without an audio device). Text or effects timed to a track follow it and stay in step even when frames stall.
- `seek(seconds: float)` -> `bool` - Jumps the playing sound to that time; False when nothing is playing.
- `pause()` -> `bool`
- `resume()` -> `bool`

### 4.14 Paths

#### Path3D

Inherits `Node3D`.

No properties of its own.

Methods:

- `get_point_count()` -> `int`
- `set_point_count(count: int)`
- `add_point(position: Vec3, in: Vec3 = Vec3(0, 0, 0), out: Vec3 = Vec3(0, 0, 0), at_position: int = -1)`
- `set_point_position(index: int, position: Vec3)`
- `get_point_position(index: int)` -> `Vec3`
- `set_point_tilt(index: int, tilt: float)`
- `get_point_tilt(index: int)` -> `float`
- `set_point_in(index: int, in: Vec3)`
- `get_point_in(index: int)` -> `Vec3`
- `set_point_out(index: int, out: Vec3)`
- `get_point_out(index: int)` -> `Vec3`
- `remove_point(index: int)`
- `clear_points()`
- `sample(index: int, offset: float)` -> `Vec3`
- `samplef(findex: float)` -> `Vec3`
- `set_closed(closed: bool)`
- `is_closed()` -> `bool`
- `set_bake_interval(tolerance: float)`
- `get_bake_interval()` -> `float`
- `set_up_vector_enabled(enable: bool)`
- `is_up_vector_enabled()` -> `bool`
- `get_baked_length()` -> `float`
- `sample_baked(offset: float, cubic: bool = False)` -> `Vec3`
- `sample_baked_tilt(offset: float)` -> `float`
- `sample_baked_up_vector(offset: float, apply_tilt: bool = False)` -> `Vec3`
- `get_baked_points()` -> `list of Vec3`
- `get_baked_tilts()` -> `list of float`
- `get_baked_up_vectors()` -> `list of Vec3`
- `get_baked_dist_cache()` -> `list of float`
- `get_closest_point(to_point: Vec3)` -> `Vec3`
- `get_closest_offset(to_point: Vec3)` -> `float`
- `get_points()` -> `list of Vec3`
- `tessellate(max_stages: int = 5, tolerance: float = 4.0)` -> `list of Vec3`
- `tessellate_even_length(max_stages: int = 5, length: float = 0.2)` -> `list of Vec3`

#### PathFollow3D

Inherits `Node3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `progress` | float | 0 .. 10000 (unbounded); step 0.01 | 0 |
| `h_offset` | float |  | 0 |
| `v_offset` | float |  | 0 |
| `rotation_mode` | int | 0=None, 1=Y, 2=XY, 3=XYZ, 4=Oriented |  |
| `use_model_front` | bool |  | False |
| `cubic_interp` | bool |  | True |
| `loop` | bool |  | True |
| `tilt_enabled` | bool |  | True |

Methods:

- `set_progress_ratio(ratio: float)`
- `get_progress_ratio()` -> `float`
- `update_transform()`

### 4.15 Particles and trails

#### ParticleSystem3D

Inherits `VisualInstance3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `texture` | ResourcePath (string) | resource path, type Texture2D |  |
| `max_particles` | int | 0 .. 1000000 (no upper bound); step 1 |  |
| `blend_mode` | int | 0=Alpha, 1=Additive, 2=Multiply, 3=Screen | 1 (Additive) |
| `align` | int | 0=Billboard, 1=Y To Velocity | 0 (Billboard) |
| **Emission** (group) | | | |
| `emission_mode` | int | 0=Continuous, 1=Burst, 2=One Shot, 3=Pulse | 0 (Continuous) |
| `emission_rate` | float | 0 .. 100000 (no upper bound); step 0.01 | 10 |
| `burst_count` | int | 0 .. 100000 (no upper bound); step 1 | 10 |
| `burst_interval` | float | 0 .. 3600 (no upper bound); step 0.01 | 1 |
| `one_shot_count` | int | 0 .. 100000 (no upper bound); step 1 | 10 |
| `pulse_rate` | float | 0 .. 100000 (no upper bound); step 0.01 | 1 |
| `particles_per_pulse` | int | 0 .. 100000 (no upper bound); step 1 | 5 |
| `emission_offset` | Vec3 |  | Vec3(0, 0, 0) |
| `emission_direction` | Vec3 |  | Vec3(0, 1, 0) |
| `emission_spread_angle` | float | 0 .. 180; step 0.01 | 0 |
| `duration` | float | -1 .. 3600 (unbounded); step 0.01 | -1 |
| `loop` | bool |  | True |
| **Shape** (group) | | | |
| `shape_type` | int | 0=Point, 1=Sphere, 2=Box, 3=Cone, 4=Circle, 5=Ring | 0 (Point) |
| `shape_radius` | float | 0 .. 10000 (no upper bound); step 0.001 | 1 |
| `shape_inner_radius` | float | 0 .. 10000 (no upper bound); step 0.001 | 0.5 |
| `shape_cone_angle` | float | 0 .. 180; step 0.01 |  |
| `shape_box_size` | Vec3 | 0 .. 10000 (no upper bound); step 0.001 | Vec3(1, 1, 1) |
| **Particle** (group) | | | |
| `particle_lifetime_min` | float | 0 .. 3600 (no upper bound); step 0.01 | 1 |
| `particle_lifetime_max` | float | 0 .. 3600 (no upper bound); step 0.01 | 3 |
| `particle_speed_min` | float | -10000 .. 10000 (unbounded); step 0.01 | 1 |
| `particle_speed_max` | float | -10000 .. 10000 (unbounded); step 0.01 | 5 |
| `particle_size_start` | Vec2 |  | Vec2(0.5, 0.5) |
| `particle_size_end` | Vec2 |  | Vec2(0.1, 0.1) |
| `particle_color_start` | Color |  | Color(1, 1, 1, 1) |
| `particle_color_end` | Color |  | Color(1, 1, 1, 0) |
| `particle_rotation_speed_min` | float | -100000 .. 100000 (unbounded); step 0.01 | 0 |
| `particle_rotation_speed_max` | float | -100000 .. 100000 (unbounded); step 0.01 | 0 |
| **Scale Over Velocity** (group) | | | |
| `scale_velocity_enabled` | bool |  | False |
| `scale_velocity_min` | float | -10000 .. 10000 (unbounded); step 0.01 | 0 |
| `scale_velocity_max` | float | -10000 .. 10000 (unbounded); step 0.01 | 5 |
| `scale_velocity_at_min` | Vec2 |  | Vec2(1, 1) |
| `scale_velocity_at_max` | Vec2 |  | Vec2(1, 1) |
| **Physics** (group) | | | |
| `gravity` | Vec3 |  | Vec3(0, 0, 0) |
| `drag` | float | -100 .. 100 (unbounded); step 0.001 | 0 |
| **Collision** (group) | | | |
| `collision_mode` | int | 0=Disabled, 1=Rigid, 2=Hide On Contact | 0 (Disabled) |
| `collision_friction` | float | 0 .. 1 (no upper bound); step 0.01 | 0 |
| `collision_bounce` | float | 0 .. 1 (no upper bound); step 0.01 | 0 |
| `collision_base_size` | float | 0 .. 100 (no upper bound); step 0.001 | 0.01 |
| `collision_use_scale` | bool |  | False |
| **Sub Emitter** (group) | | | |
| `sub_emitter` | NodePath (string) | node path, valid types ParticleSystem3D |  |
| `sub_emitter_mode` | int | 0=Disabled, 1=Constant, 2=At End, 3=At Collision, 4=At Start | 0 (Disabled) |
| `sub_emitter_frequency` | float | 0 .. 100000 (no upper bound); step 0.01 | 4 |
| `sub_emitter_amount_at_end` | int | 0 .. 100000 (no upper bound); step 1 | 1 |
| `sub_emitter_amount_at_collision` | int | 0 .. 100000 (no upper bound); step 1 | 1 |
| `sub_emitter_amount_at_start` | int | 0 .. 100000 (no upper bound); step 1 | 1 |
| `sub_emitter_keep_velocity` | bool |  | False |
| **Atlas** (group) | | | |
| `atlas_enabled` | bool |  | False |
| `atlas_columns` | int | 1 .. 1024 (no upper bound); step 1 | 1 |
| `atlas_rows` | int | 1 .. 1024 (no upper bound); step 1 | 1 |
| `atlas_anim_speed_min` | float | -1000 .. 1000 (unbounded); step 0.01 | 0 |
| `atlas_anim_speed_max` | float | -1000 .. 1000 (unbounded); step 0.01 | 0 |
| `atlas_anim_offset_min` | float | -1000 .. 1000 (unbounded); step 0.01 | 0 |
| `atlas_anim_offset_max` | float | -1000 .. 1000 (unbounded); step 0.01 | 0 |
| `atlas_anim_loop` | bool |  | False |

Methods:

- `play()`
- `stop()`
- `pause()`
- `simulate(dt: float)`
- `emit_burst(count: int)`
- `emit_particle(position: Vec3, velocity: Vec3 = (emitter default))` -> `bool` - Emits one particle at a position. Without a velocity the particle keeps the velocity chosen by the emitter. False when no particle is free.
- `get_state()` -> `int (EMITTER_STATE_*)`
- `get_active_count()` -> `int`
- `set_continuous(particles_per_second: float)` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `set_burst(count: int, interval: float = 1.0)` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `set_one_shot(count: int)` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `set_pulse(particles_per_second: float, per_pulse: int)` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `set_shape_point()` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `set_shape_sphere(radius: float)` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `set_shape_box(size: Vec3)` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `set_shape_cone(degrees: float, base_radius: float = 0.0)` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `set_shape_circle(radius: float)` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `set_shape_ring(outer: float, inner: float)` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `add_gravity(gravity: Vec3)` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `add_drag(strength: float)` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `add_vortex(center: Vec3, strength: float, radius: float)` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `add_attractor(position: Vec3, strength: float, radius: float, repulse: bool = False)` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `add_turbulence(strength: float, frequency: float)` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `add_color_over_lifetime(start: Vec4, end: Vec4)` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `add_size_over_lifetime(start: Vec2, end: Vec2)` -> `ParticleSystem3D` - Returns the particle system itself, so calls can be chained.
- `clear_affectors()`
- `get_affector_count()` -> `int`

#### RibbonTrail3D

Inherits `VisualInstance3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `material` | ResourcePath (string) | resource path, type Material |  |
| `emitting` | bool |  | True |
| `trail_length` | float | 0.01 .. 1000 (no upper bound); step 0.01 |  |
| `max_elements` | int | 2 .. 64; step 1 |  |
| `smoothing` | int | 1 .. 8; step 1 | 4 |

Methods:

- `add_chain(emitter_path: string, start_color: Vec4 = Color(1, 1, 1, 1), end_color: Vec4 = Color(1, 1, 1, 0), start_width: float = 0.7, end_width: float = 0.05, fade_time: float = 0.3)` -> `int` - Returns the chain index.
- `add_blade_chain(base_path: string, tip_path: string, start_color: Vec4 = Color(1, 1, 1, 1), end_color: Vec4 = Color(1, 1, 1, 0), fade_time: float = 0.3, start_span: float = 1.0, end_span: float = 0.0)` -> `int` - Returns the chain index.
- `clear_chains()`
- `get_chain_count()` -> `int`
- `get_element_count(chain: int)` -> `int`

#### ParticleCollision3D

Inherits `Node3D`. Abstract.

No properties of its own.

Methods:

- `collide(position: Vec3, size: float)` -> `list` - None when there is no collision, otherwise a list [normal: Vec3, depth: float].

#### ParticleCollisionSphere3D

Inherits `ParticleCollision3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `radius` | float | 0.001 .. 1024 (no upper bound); step 0.001 |  |

No methods of its own.

#### ParticleCollisionBox3D

Inherits `ParticleCollision3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `size` | Vec3 | 0.001 .. 1024 (no upper bound); step 0.001 |  |

No methods of its own.

### 4.16 Terrain, vegetation, roads and water

#### Terrain3D

Inherits `Node3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `material` | ResourcePath (string) | resource path, type Material |  |
| `max_pixel_error` | float | 0 .. 64 (no upper bound); step 0.01 |  |
| `cast_shadows` | bool |  | True |
| **Maps** (group) | | | |
| `lightmap_enabled` | bool |  | True |
| `lightmap_in_near_shading` | bool |  | True |
| `composite_map_enabled` | bool |  | True |
| `composite_map_distance` | float | 0 .. 100000 (no upper bound); step 0.1 |  |
| `colour_map_enabled` | bool | enables the global colour map that tints the terrain; the Colour brush of the editor turns it on when first used | False |
| `colour_map_size` | int | 64 .. 4096; step 64; texels per side of the colour map (applied when it is enabled) | 1024 |
| **Layers** (group) | | | |
| `layer_normal_mapping_enabled` | bool |  | True |
| `layer_parallax_mode` | int | 0 .. 2; step 1 | 1 |
| `layer_parallax_scale` | float | 0 .. 1 (no upper bound); step 0.001 | 0.04 |
| `detail_modulation` | bool |  | False |
| `detail_strength` | float | 0 .. 1; step 0.01 | 1 |
| `layer_count` | int | 0 .. 5; step 1; not stored in scene files |  |
| **Layer 1** (subgroup) | | | |
| `layer_0_world_size` (`layer_0/world_size`) | float | 0 .. 100000 (no upper bound); step 0.1; not stored in scene files |  |
| `layer_0_diffuse_specular` (`layer_0/diffuse_specular`) | ResourcePath (string) | resource path, type Texture2D; not stored in scene files |  |
| `layer_0_normal_height` (`layer_0/normal_height`) | ResourcePath (string) | resource path, type Texture2D; not stored in scene files |  |
| **Layer 2** (subgroup) | | | |
| `layer_1_world_size` (`layer_1/world_size`) | float | 0 .. 100000 (no upper bound); step 0.1; not stored in scene files |  |
| `layer_1_diffuse_specular` (`layer_1/diffuse_specular`) | ResourcePath (string) | resource path, type Texture2D; not stored in scene files |  |
| `layer_1_normal_height` (`layer_1/normal_height`) | ResourcePath (string) | resource path, type Texture2D; not stored in scene files |  |
| **Layer 3** (subgroup) | | | |
| `layer_2_world_size` (`layer_2/world_size`) | float | 0 .. 100000 (no upper bound); step 0.1; not stored in scene files |  |
| `layer_2_diffuse_specular` (`layer_2/diffuse_specular`) | ResourcePath (string) | resource path, type Texture2D; not stored in scene files |  |
| `layer_2_normal_height` (`layer_2/normal_height`) | ResourcePath (string) | resource path, type Texture2D; not stored in scene files |  |
| **Layer 4** (subgroup) | | | |
| `layer_3_world_size` (`layer_3/world_size`) | float | 0 .. 100000 (no upper bound); step 0.1; not stored in scene files |  |
| `layer_3_diffuse_specular` (`layer_3/diffuse_specular`) | ResourcePath (string) | resource path, type Texture2D; not stored in scene files |  |
| `layer_3_normal_height` (`layer_3/normal_height`) | ResourcePath (string) | resource path, type Texture2D; not stored in scene files |  |
| **Layer 5** (subgroup) | | | |
| `layer_4_world_size` (`layer_4/world_size`) | float | 0 .. 100000 (no upper bound); step 0.1; not stored in scene files |  |
| `layer_4_diffuse_specular` (`layer_4/diffuse_specular`) | ResourcePath (string) | resource path, type Texture2D; not stored in scene files |  |
| `layer_4_normal_height` (`layer_4/normal_height`) | ResourcePath (string) | resource path, type Texture2D; not stored in scene files |  |

Methods:

- `is_loaded()` -> `bool`
- `unload()`
- `update()`
- `update_from_camera()` -> `bool`
- `flush_composite_map()` -> `bool`
- `get_world_height_at(position: Vec3)` -> `float`
- `get_world_slope_at(position: Vec3)` -> `float`
- `get_aabb()` -> `Aabb`
- `get_normal_at(position: Vec3)` -> `Vec3` - Normal at the nearest terrain grid cell to a world position. Runtime error when the terrain is not loaded.
- `ray_intersects(origin: Vec3, direction: Vec3)` -> `Vec3` - First hit position, or None. The direction is normalized. Runtime error when the terrain is not loaded.
- `get_size()` -> `int`
- `get_world_size()` -> `float`
- `get_min_height()` -> `float`
- `get_max_height()` -> `float`
- `add_layer(world_size: float, diffuse_specular: string, normal_height: string)` -> `bool`
- `remove_layer(index: int)`
- `set_layer_world_size(index: int, size: float)`
- `get_layer_count()` -> `int`
- `get_layer_world_size(index: int)` -> `float`
- `brush_height(center: Vec3, radius: float, strength: float, elapsed: float, lower: bool = False)` -> `bool` - Raises (or lowers) the terrain height around a world position. Runtime error when the terrain is not loaded.
- `brush_blend(layer: int, center: Vec3, radius: float, strength: float, elapsed: float, remove: bool = False)` -> `bool` - Paints (or removes) a texture layer. Runtime error when the terrain is not loaded.
- `brush_smooth(center: Vec3, radius: float, strength: float, elapsed: float)` -> `bool` - Runtime error when the terrain is not loaded.
- `brush_flatten(center: Vec3, radius: float, height: float, strength: float, elapsed: float)` -> `bool` - Runtime error when the terrain is not loaded.
- `set_global_colour_map_enabled(enabled: bool, size: int = 0)`
- `set_light_map_direction(direction: Vec3)`
- `set_composite_map_lighting(ambient: Vec3, diffuse: Vec3)`
- `get_node_count()` -> `int`
- `get_nodes_drawn()` -> `int`
- `get_nodes_drawn_at_lod(lod: int)` -> `int`
- `get_nodes_drawn_composite()` -> `int`
- `get_lod_level_count()` -> `int`
- `get_triangles_drawn()` -> `int`
- `get_vertex_mesh_count()` -> `int`
- `get_index_pool_count()` -> `int`
- `get_node_mesh_count()` -> `int`
- `get_vertex_upload_bytes()` -> `int`
- `get_vertex_upload_ranges()` -> `int`
- `get_last_update_owners()` -> `int`
- `reset_edit_stats()`
- `get_uploaded_bytes()` -> `int`
- `get_upload_count()` -> `int`
- `reset_upload_stats()`
- `get_texture_memory_bytes()` -> `int`
- `get_composite_bakes()` -> `int`
- `get_composite_bake_texels()` -> `int`

#### Grass3D

Inherits `Node3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `terrain` | NodePath (string) | node path, valid types Terrain3D |  |
| **Types** (group) | | | |
| `type_count` | int | 0 .. 4; not stored in scene files (the types are saved by the node itself); raising it adds default types, lowering it removes the last ones | 0 |
| `type_N/texture` (N = 0..3) | ResourcePath (string) | resource path, type Texture2D; not stored in scene files; setting any `type_N/...` property creates the types up to N; shown in the editor only for existing types |  |
| `type_N/height` | float | 0 .. 100 (no upper bound); step 0.01 | 1.2 |
| `type_N/width` | float | 0 .. 10 (no upper bound); step 0.01 | 0.7 |
| `type_N/alpha_cut` | float | 0 .. 1; step 0.01 | 0.35 |
| `type_N/colour_a` | Color | editor hides alpha | 0.62, 0.78, 0.42 |
| `type_N/colour_b` | Color | editor hides alpha | 0.95, 0.92, 0.60 |
| `type_N/scale_min` | float | 0 .. 10 (no upper bound); step 0.01 | 0.75 |
| `type_N/scale_max` | float | 0 .. 10 (no upper bound); step 0.01 | 1.25 |
| `type_N/density_scale` | float | 0 .. 10 (no upper bound); step 0.01 | 1 |
| `max_slope` | float | 0 .. 90; step 0.1 |  |
| **Placement** (group) | | | |
| `cell_size` | float | 0.5 .. 1024 (no upper bound); step 0.01 | 8 |
| `density` | float | 0 .. 1024 (no upper bound); step 0.001 | 0.35 |
| `density_falloff_start` | float | 0 .. 1; step 0.01 | 0.35 |
| `draw_distance` | float | 0 .. 100000 (no upper bound); step 0.1 | 60 |
| `release_margin` | float | 0 .. 100000 (no upper bound); step 0.1 | 8 |
| `seed` | int |  | 12345 |
| **Rendering** (group) | | | |
| `cast_shadows` | bool |  | False |
| `alpha_antialiasing` | int | 0=Off, 1=Alpha To Coverage | 0 (Off) |
| `camera_bend` | float | 0 .. 1 (unbounded); step 0.01 | 0.55 |
| `frustum_culling_enabled` | bool |  | True |
| **Wind** (group) | | | |
| `wind` | float | 0 .. 16 (unbounded); step 0.01 | 1 |
| `wind_speed` | float | 0 .. 16 (unbounded); step 0.01 | 1 |
| `wind_direction` | Vec3 |  |  |
| **Mask** (group) | | | |
| `extent_enabled` | bool |  | False |
| `extent_min` | Vec2 |  |  |
| `extent_max` | Vec2 |  |  |
| `mask_resolution_width` | int | 2 .. 16384 (no upper bound); step 1 | 256 |
| `mask_resolution_depth` | int | 2 .. 16384 (no upper bound); step 1 | 256 |
| **Simulation** (group) | | | |
| `stiffness` | float | 0 .. 1000 (no upper bound); step 0.01 | 12 |
| `drag` | float | 0 .. 1; step 0.01 | 0.12 |

Methods:

- `remove_type(type: int)`
- `get_type_count()` -> `int`
- `add_type_region(type: int, x: int, y: int, width: int, height: int, size: float = 1.0, weight: float = 1.0)` -> `bool`
- `clear_type_regions(type: int)`
- `get_type_region_count(type: int)` -> `int`
- `add_region(x: int, y: int, width: int, height: int, size: float = 1.0, weight: float = 1.0)` -> `bool` - Adds the region to grass type 0 (the default type).
- `clear_regions()`
- `get_region_count()` -> `int`
- `set_height(metres: float)`
- `get_height()` -> `float`
- `set_width(multiplier: float)`
- `get_width()` -> `float`
- `set_alpha_cut(cut: float)`
- `get_alpha_cut()` -> `float`
- `set_alpha_antialiasing(mode: int (ALPHA_ANTIALIASING_*), edge: float = 0.3)`
- `set_extent(min_x: float, min_z: float, max_x: float, max_z: float)`
- `clear_extent()`
- `set_terrain(terrain: Terrain3D)`
- `get_terrain()` -> `Terrain3D`
- `clear_exclusions()`
- `get_reseated_count()` -> `int`
- `get_reseat_milliseconds()` -> `float`
- `clear_influencers()`
- `add_influencer(centre: Vec3, radius: float, force: float)` -> `bool`
- `get_influencer_count()` -> `int`
- `create_mask(width: int = 0, depth: int = 0)` -> `bool` - A width or depth of 0 uses `mask_resolution_width` / `mask_resolution_depth`. Returns False when no extent is set.
- `has_mask()` -> `bool`
- `get_mask_width()` -> `int`
- `get_mask_depth()` -> `int`
- `clear_mask()`
- `paint(world_position: Vec3, radius: float, amount: float, type: int)` -> `bool`
- `erase(world_position: Vec3, radius: float, amount: float, type: int = (all types))` -> `bool` - Without a type the mask of every grass type is erased.
- `fill(type: int, density: float)`
- `get_mask_density(type: int, x: float, z: float)` -> `float`
- `save_mask(path: string)` -> `bool` - A relative path is resolved against the open project.
- `load_mask(path: string)` -> `bool` - A relative path is resolved against the open project when the file exists there.
- `invalidate()`
- `get_active_cell_count(type: int = -1)` -> `int` - A type of -1 counts every type.
- `get_visible_cell_count(type: int = -1)` -> `int` - A type of -1 counts every type.
- `get_pooled_cell_count(type: int = -1)` -> `int` - A type of -1 counts every type.
- `get_instance_count(type: int = -1)` -> `int` - A type of -1 counts every type.
- `get_drawn_instance_count(type: int = -1)` -> `int` - A type of -1 counts every type.
- `get_written_instance_count()` -> `int`
- `get_awake_instance_count()` -> `int`
- `get_simulated_instance_count()` -> `int`
- `get_filled_cell_count()` -> `int`
- `get_update_milliseconds()` -> `float`
- `has_cell(type: int, cell_x: int, cell_z: int)` -> `bool`
- `get_cell_clump_count(type: int, cell_x: int, cell_z: int)` -> `int`
- `get_cell_node(type: int, cell_x: int, cell_z: int)` -> `MultiMeshInstance3D`

#### Forest3D

Inherits `Node3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `terrain` | NodePath (string) | node path, valid types Terrain3D |  |
| **Species** (group) | | | |
| **Species 1** (subgroup) | | | |
| `species_0_mesh` (`species_0/mesh`) | ResourcePath (string) | resource path, type Mesh; not stored in scene files |  |
| `species_0_shadow_mesh` (`species_0/shadow_mesh`) | ResourcePath (string) | resource path, type Mesh; not stored in scene files; lower-detail mesh used for the shadow passes of species 1 |  |
| **Species 2** (subgroup) | | | |
| `species_1_mesh` (`species_1/mesh`) | ResourcePath (string) | resource path, type Mesh; not stored in scene files |  |
| `species_1_shadow_mesh` (`species_1/shadow_mesh`) | ResourcePath (string) | resource path, type Mesh; not stored in scene files; lower-detail mesh used for the shadow passes of species 2 |  |
| **Species 3** (subgroup) | | | |
| `species_2_mesh` (`species_2/mesh`) | ResourcePath (string) | resource path, type Mesh; not stored in scene files |  |
| `species_2_shadow_mesh` (`species_2/shadow_mesh`) | ResourcePath (string) | resource path, type Mesh; not stored in scene files; lower-detail mesh used for the shadow passes of species 3 |  |
| **Species 4** (subgroup) | | | |
| `species_3_mesh` (`species_3/mesh`) | ResourcePath (string) | resource path, type Mesh; not stored in scene files |  |
| `species_3_shadow_mesh` (`species_3/shadow_mesh`) | ResourcePath (string) | resource path, type Mesh; not stored in scene files; lower-detail mesh used for the shadow passes of species 4 |  |
| **Species 5** (subgroup) | | | |
| `species_4_mesh` (`species_4/mesh`) | ResourcePath (string) | resource path, type Mesh; not stored in scene files |  |
| `species_4_shadow_mesh` (`species_4/shadow_mesh`) | ResourcePath (string) | resource path, type Mesh; not stored in scene files; lower-detail mesh used for the shadow passes of species 5 |  |
| **Species 6** (subgroup) | | | |
| `species_5_mesh` (`species_5/mesh`) | ResourcePath (string) | resource path, type Mesh; not stored in scene files |  |
| `species_5_shadow_mesh` (`species_5/shadow_mesh`) | ResourcePath (string) | resource path, type Mesh; not stored in scene files; lower-detail mesh used for the shadow passes of species 6 |  |
| **Species 7** (subgroup) | | | |
| `species_6_mesh` (`species_6/mesh`) | ResourcePath (string) | resource path, type Mesh; not stored in scene files |  |
| `species_6_shadow_mesh` (`species_6/shadow_mesh`) | ResourcePath (string) | resource path, type Mesh; not stored in scene files; lower-detail mesh used for the shadow passes of species 7 |  |
| **Species 8** (subgroup) | | | |
| `species_7_mesh` (`species_7/mesh`) | ResourcePath (string) | resource path, type Mesh; not stored in scene files |  |
| `species_7_shadow_mesh` (`species_7/shadow_mesh`) | ResourcePath (string) | resource path, type Mesh; not stored in scene files; lower-detail mesh used for the shadow passes of species 8 |  |
| `terrain_offset` | float |  | 0 |
| `max_slope` | float | 0 .. 90; step 0.1 |  |
| **Distance** (group) | | | |
| `swap_distance` | float | 0 .. 100000 (no upper bound); step 0.1 | 120 |
| `draw_distance` | float | 0 .. 100000 (no upper bound); step 0.1 | 500 |
| **Rendering** (group) | | | |
| `impostors_enabled` | bool |  | True |
| `cast_shadows` | bool |  | True |
| `frustum_culling_enabled` | bool |  | True |
| `shadow_cull_margin` | float | 0 .. 1000 (no upper bound); step 0.01 | 2 |

Methods:

- `remove_species(species: int)`
- `get_species_count()` -> `int`
- `get_mesh_node(species: int)` -> `MultiMeshInstance3D`
- `plant(species: int, position: Vec3, yaw: float, scale: float)` -> `int`
- `plant_on_terrain(species: int, x: float, z: float, yaw: float, scale: float)` -> `int`
- `clear_instances()`
- `set_terrain(terrain: Terrain3D)`
- `get_terrain()` -> `Terrain3D`
- `clear_exclusions()`
- `get_reseated_count()` -> `int`
- `get_reseat_milliseconds()` -> `float`
- `get_instance_count()` -> `int`
- `get_instance_species(instance: int)` -> `int`
- `is_instance_on_terrain(instance: int)` -> `bool`
- `get_instance_centre(instance: int)` -> `Vec3`
- `get_instance_radius(instance: int)` -> `float`
- `get_mesh_instance_count(species: int = (all species))` -> `int`
- `get_impostor_instance_count(species: int = (all species))` -> `int`
- `get_written_instance_count()` -> `int`

#### Tree3D

Inherits `VisualInstance3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `seed` | int |  |  |
| `bark_material` | ResourcePath (string) | resource path, type Material |  |
| `leaf_material` | ResourcePath (string) | resource path, type Material |  |
| `shadow_leaf_step` | int | 1 .. 64 (no upper bound); step 1 | 6 |
| **Trunk** (group) | | | |
| `trunk_length` | float | 0 .. 100 (unbounded); step 0.01 | 2.5 |
| `trunk_kink` | float | -1 .. 1 (unbounded); step 0.01 | 0 |
| `trunk_steps` | int | 0 .. 128; step 1 | 2 |
| `max_radius` | float | 0 .. 10 (unbounded); step 0.001 | 0.25 |
| `taper_rate` | float | 0.01 .. 1 (unbounded); step 0.001 | 0.95 |
| `twist_rate` | float | -100 .. 100 (unbounded); step 0.01 | 13 |
| **Branches** (group) | | | |
| `levels` | int | 1 .. 12; step 1 | 3 |
| `segments` | int | 4 .. 64; step 2 | 6 |
| `branch_factor` | float | 0 .. 10 (unbounded); step 0.01 | 2 |
| `initial_branch_length` | float | 0 .. 10 (unbounded); step 0.01 | 0.85 |
| `length_falloff_factor` | float | 0 .. 2 (unbounded); step 0.01 | 0.85 |
| `length_falloff_power` | float | 0 .. 8 (unbounded); step 0.01 | 1 |
| `radius_falloff_rate` | float | 0.01 .. 2 (unbounded); step 0.01 | 0.6 |
| `clump_max` | float | 0 .. 2 (unbounded); step 0.01 | 0.8 |
| `clump_min` | float | 0 .. 2 (unbounded); step 0.01 | 0.5 |
| `climb_rate` | float | -10 .. 10 (unbounded); step 0.01 | 1.5 |
| `drop_amount` | float | -10 .. 10 (unbounded); step 0.01 | 0 |
| `grow_amount` | float | -10 .. 10 (unbounded); step 0.01 | 0 |
| `sweep_amount` | float | -10 .. 10 (unbounded); step 0.01 | 0 |
| **Texture** (group) | | | |
| `v_multiplier` | float | -10 .. 10 (unbounded); step 0.01 | 0.2 |
| `twig_scale` | float | 0 .. 100 (unbounded); step 0.01 | 2 |

Methods:

- `rebuild()` -> `bool`
- `is_dirty()` -> `bool`
- `get_build_count()` -> `int`

#### Road3D

Inherits `VisualInstance3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `material` | ResourcePath (string) | resource path, type Material |  |
| `width` | float | 0.001 .. 1000 (no upper bound); step 0.001 | 4 |
| `texture_repeat` | float | 0 .. 1000 (no upper bound); step 0.01 | 4 |
| **Sampling** (group) | | | |
| `interval_type` | int | 0=Distance, 1=Subdivide | 0 (Distance) |
| `interval` | float | 0.01 .. 1000 (no upper bound); step 0.01 | 1 |
| `simplify_angle` | float | 0 .. 180; step 0.1 | 0 |
| `rotation_mode` | int | 0=Fixed, 1=Path, 2=Path Follow |  |
| `rotation_accurate` | bool |  | False |
| **Ground** (group) | | | |
| `terrain` | NodePath (string) | node path, valid types Terrain3D |  |
| `ground_offset` | float |  | 0.06 |
| `ground_tilt` | bool |  | True |

Methods:

- `set_profile(profile: list of Vec2)`
- `clear_profile()`
- `get_profile()` -> `list of Vec2`
- `set_point_width(index: int, width: float)`
- `get_point_width(index: int)` -> `float`
- `clear_point_widths()`
- `clear_ground()`
- `has_ground()` -> `bool`
- `set_terrain(terrain: Terrain3D)`
- `get_terrain()` -> `Terrain3D`
- `flush()` -> `bool`
- `rebuild()`
- `is_dirty()` -> `bool`
- `get_build_count()` -> `int`

#### Water3D

Inherits `VisualInstance3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| `size` | Vec2 | 0.001 .. 100000 (no upper bound); step 0.01 | Vec2(10, 10) |
| `uv_tiling` | float | 0 .. 100 (unbounded); step 0.0001 | 0.01 |
| `normal_texture` | ResourcePath (string) | resource path, type Texture2D |  |
| `color` | Color |  | Color(0.032, 0.1, 0.16, 0.17) |
| `roughness` | float | 0 .. 1; step 0.01 | 0.2 |
| `specular` | float | 0 .. 1 (no upper bound); step 0.01 | 1 |
| **Refraction** (group) | | | |
| `refraction_enabled` | bool |  | True |
| `refraction_strength` | float | 0 .. 1 (no upper bound); step 0.001 | 0.05 |
| **Waves** (group) | | | |
| `waves_wind_direction` | Vec2 |  | Vec2(0.1, -2.1) |
| `waves_wind_speed` | float | 0 .. 100 (unbounded); step 0.01 | 1 |
| `waves_big` | Vec2 |  | Vec2(0.3, 0.3) |
| `waves_mid` | Vec2 |  | Vec2(0.3, 0.15) |
| `waves_small` | Vec2 |  | Vec2(0.15, 0.1) |
| `waves_scale` | float | 0 .. 1000 (unbounded); step 0.01 | 25 |
| `waves_chop` | float | 0 .. 10 (unbounded); step 0.001 | 0.15 |
| `waves_bump` | float | 0 .. 100 (unbounded); step 0.01 | 1.5 |
| `waves_bump2` | float | 0 .. 100 (unbounded); step 0.01 | 6 |
| `waves_specular_mix` | float | 0 .. 1 (unbounded); step 0.001 | 0.4 |
| `waves_depth_fade` | Vec3 |  | Vec3(0.06, 0.02, 0.01) |
| `waves_depth_scale` | float | 0 .. 100 (unbounded); step 0.001 | 0.6 |
| `waves_shore_fade` | float | 0 .. 1000 (unbounded); step 0.01 | 16.4 |
| **Reflection** (group) | | | |
| `reflection_mode` | int | 0=Sky, 1=Planar | 0 (Sky) |
| `reflection_resolution` | int | 16 .. 8192; step 1 | 512 |
| `reflection_max_distance` | float | 0 .. 4096 (no upper bound); step 0.01 | 8 |

Methods:

- `set_wind(direction: Vec2, speed: float)`
- `get_planar_reflector()` -> `PlanarReflector3D`

#### Ocean3D

Inherits `VisualInstance3D`.

| Property | Type | Range / values | Default |
|---|---|---|---|
| **Simulation** (group) | | | |
| `simulation_patch_length` | float | 1 .. 10000 (no upper bound); step 0.01 | 50 |
| `simulation_time_scale` | float | 0 .. 100 (no upper bound); step 0.001 | 0.3 |
| `simulation_wave_amplitude` | float | 0 .. 100000 (no upper bound); step 0.1 | 1000 |
| `simulation_wind_direction` | Vec2 |  | Vec2(0.8, 0.6) |
| `simulation_wind_speed` | float | 0 .. 100000 (no upper bound); step 0.1 | 600 |
| `simulation_wind_dependency` | float | 0 .. 1 (no upper bound); step 0.001 | 0.07 |
| `simulation_choppy_scale` | float | 0 .. 100 (no upper bound); step 0.01 | 1.3 |
| `simulation_seed` | int |  | 1 |
| **Surface** (group) | | | |
| `water_color` | Color |  |  |
| `extinction_color` | Color | editor hides alpha | Color(0, 0.9, 1, 1) |
| `surface_detail` | int | 1 .. 8; step 1 | 4 |
| `displacement_tolerance` | float | 0 .. 100 (no upper bound); step 0.01 | 2 |
| **Reflection** (group) | | | |
| `planar_reflection` | bool |  | False |
| `reflection_resolution` | int | 16 .. 8192; step 1 | 512 |

Methods:

- `is_supported()` -> `bool`
- `get_water_height()` -> `float`
- `get_displaced_position(world_position: Vec3)` -> `Vec3`
- `update()`

### 4.17 2D interface nodes

Interface nodes live anywhere in the tree and draw on the HUD layer after every node has processed, in tree order (a child draws over its parent, a later sibling over an earlier one). They are plain `Node`s, not `Node3D`s. Sizes and positions are in pixels of the window (in the editor, of the game panel). The top-level `Control` takes the window as its parent rectangle and lays itself out again whenever the window size changes.

#### Control

Base of the interface nodes: a rectangle placed inside its parent `Control` (or the window) by anchors and offsets.

- `anchors_preset` (int, enum): `0` Top Left, `1` Top Right, `2` Bottom Left, `3` Bottom Right, `4` Center Left, `5` Center Top, `6` Center Right, `7` Center Bottom, `8` Center, `9` Left Wide, `10` Top Wide, `11` Right Wide, `12` Bottom Wide, `13` VCenter Wide, `14` HCenter Wide, `15` Full Rect. Setting it sets the anchors, the offsets (keeping the current size, or the minimum size for Full Rect) and the grow directions in one go; `-1` means custom anchors.
- `anchor_left`, `anchor_top`, `anchor_right`, `anchor_bottom` (float 0..1): fraction of the parent rectangle each edge is attached to.
- `offset_left`, `offset_top`, `offset_right`, `offset_bottom` (float): pixels added to each anchored edge.
- `grow_horizontal`, `grow_vertical` (int, enum `0` Begin, `1` End, `2` Both): where the rectangle grows when its minimum size is larger than the anchored size.
- `position`, `size` (Vec2): the rectangle inside the parent; setting them recomputes the offsets for the current anchors. They are not saved: a scene stores the anchors and the offsets, though one that sets them still loads.
- `custom_minimum_size` (Vec2): a minimum the rectangle never shrinks below; a `Label` also has the size of its text as a minimum.
- `visible` (bool): hides the node and everything under it.
- `modulate` (Color): multiplies the colours of the node and of its children.
- `scale` (Vec2, default 1,1) and `pivot_offset` (Vec2, pixels from the top left of the rectangle): draw the node and its children scaled about the pivot. The layout (anchors, offsets, size) is not affected, so a tween of `scale` makes a button pop without moving its neighbours; set `pivot_offset` to half the size to scale about the centre.

#### Label

A line of text inside its rectangle.

- `text` (string), `font_size` (float, pixels), `font_color` (Color).
- `horizontal_alignment` (int: `0` left, `1` center, `2` right), `vertical_alignment` (int: `0` top, `1` center, `2` bottom): where the text sits when the rectangle is larger than the text.

#### ColorRect

Fills its rectangle with `color` (Color), multiplied by `modulate`. A full-rect `ColorRect` is the background of a menu scene; such a scene needs no camera, the root can be the `Control` itself.

#### TextureRect

Draws an image in its rectangle.

- `texture` (file path): the image, loaded once and shared.
- `stretch_mode` (int): `0` scale to the rectangle, `1` tile (drawn as scale for now), `2` keep the image size at the top left, `3` keep it centered, `4` keep the aspect, `5` keep the aspect centered, `6` keep the aspect and cover the rectangle, cropping the sides.
- `expand` (bool): when off the image size is the minimum size of the rectangle; on, the rectangle may be smaller than the image.
- `flip_h`, `flip_v` (bool): mirror the image.

#### Button

A clickable rectangle with centered text. The mouse hovering and pressing it change its colour; a click (press and release over it) calls the `_button_pressed(self, button)` hook (section 1.3).

- `text` (string), `font_size` (float), `font_color`, `normal_color`, `hover_color`, `pressed_color` (Color).
- `disabled` (bool): drawn dimmed, ignores the mouse.
- `press()`: fires the hook as a click would, for keyboard or gamepad menus.
- `is_hovered() -> bool`: the mouse is over the button.
- The minimum size is the text plus padding; give it a `size` or anchors for a larger button.

#### LineEdit

A single line of editable text. A click on it gives it the focus (a click elsewhere, or ESC, takes it away); while focused it takes the typed characters, BACKSPACE, DELETE, the arrows, HOME and END, and ENTER calls `_text_submitted(self, field, text)` (section 1.3). Keyboard events it takes are marked as handled, so game actions do not fire while typing.

- `text`, `placeholder` (string; the placeholder shows dimmed while the text is empty), `font_size` (float), `font_color`, `background_color`, `focus_color` (Color; the outline while focused).
- `max_length` (int, 0 = no limit, counted in characters), `editable` (bool).
- `grab_focus()`, `release_focus()`, `has_focus() -> bool`, `clear()`, `submit()`.
- Long text scrolls so the caret stays visible. The minimum height is the font size plus padding.

## 5. Enums and constants used by node methods

Methods that take or return an enumerated value use plain integers. The constants below are registered as globals. Properties that are enumerations list their values in the property tables of section 4.

**EmitterState** - used by `ParticleSystem3D.get_state()`.

| Constant | Value |
|---|---|
| `EMITTER_STATE_STOPPED` | 0 |
| `EMITTER_STATE_PLAYING` | 1 |
| `EMITTER_STATE_PAUSED` | 2 |

**PhysicsBodyMode** - used by `PhysicsBody3D.get_mode()`.

| Constant | Value |
|---|---|
| `BODY_MODE_STATIC` | 0 |
| `BODY_MODE_KINEMATIC` | 1 |
| `BODY_MODE_RIGID` | 2 |

**PhysicsGroundState** - used by `CharacterRigidBody3D.get_ground_state()`.

| Constant | Value |
|---|---|
| `GROUND_STATE_ON_GROUND` | 0 |
| `GROUND_STATE_ON_STEEP_GROUND` | 1 |
| `GROUND_STATE_NOT_SUPPORTED` | 2 |
| `GROUND_STATE_IN_AIR` | 3 |

**LightType** - used by `Light3D.get_light_type()`.

| Constant | Value |
|---|---|
| `LIGHT_TYPE_DIRECTIONAL` | 0 |
| `LIGHT_TYPE_OMNI` | 1 |
| `LIGHT_TYPE_SPOT` | 2 |

**Topology** - used by `ManualMesh3D.surface_begin(topology)`.

| Constant | Value |
|---|---|
| `TOPOLOGY_TRIANGLES` | 0 |
| `TOPOLOGY_TRIANGLE_STRIP` | 1 |
| `TOPOLOGY_LINES` | 2 |
| `TOPOLOGY_LINE_STRIP` | 3 |
| `TOPOLOGY_POINTS` | 4 |

`surface_begin` also accepts the raw values 5 to 9 (lines adjacency, line strip adjacency, triangles adjacency, triangle strip adjacency, patches), which have no constant.

**MaterialAlphaAntialiasing** - used by `Grass3D.set_alpha_antialiasing(mode, edge)`.

| Constant | Value |
|---|---|
| `ALPHA_ANTIALIASING_OFF` | 0 |
| `ALPHA_ANTIALIASING_ALPHA_TO_COVERAGE` | 1 |

**AudioBus** - used by `set_bus_pitch` and `bus_pitch`; the same values as the `bus` property of `AudioStreamPlayer` and `AudioStreamPlayer3D`.

| Constant | Value |
|---|---|
| `AUDIO_BUS_SFX` | 0 |
| `AUDIO_BUS_MUSIC` | 1 |

**PhysicsRagdollPart** - used by `Ragdoll3D.get_part_bone`, `get_part_mass`, `apply_impulse`, `get_part_velocity` and `Dismember3D.detach`, `is_detached`.

| Constant | Value |
|---|---|
| `RAGDOLL_PART_PELVIS` | 0 |
| `RAGDOLL_PART_SPINE` | 1 |
| `RAGDOLL_PART_HEAD` | 2 |
| `RAGDOLL_PART_LEFT_UPPER_LEG` | 3 |
| `RAGDOLL_PART_LEFT_LOWER_LEG` | 4 |
| `RAGDOLL_PART_RIGHT_UPPER_LEG` | 5 |
| `RAGDOLL_PART_RIGHT_LOWER_LEG` | 6 |
| `RAGDOLL_PART_LEFT_UPPER_ARM` | 7 |
| `RAGDOLL_PART_LEFT_LOWER_ARM` | 8 |
| `RAGDOLL_PART_RIGHT_UPPER_ARM` | 9 |
| `RAGDOLL_PART_RIGHT_LOWER_ARM` | 10 |
| `RAGDOLL_PART_LEFT_FOOT` | 11 |
| `RAGDOLL_PART_RIGHT_FOOT` | 12 |

Valid values are 0 to 12.

**RagdollBone** - used by `Ragdoll3D.set_bone`, `get_bone`, `set_bone_name`.

| Constant | Value |
|---|---|
| `RAGDOLL_BONE_HIPS` | 0 |
| `RAGDOLL_BONE_SPINE` | 1 |
| `RAGDOLL_BONE_NECK` | 2 |
| `RAGDOLL_BONE_HEAD` | 3 |
| `RAGDOLL_BONE_LEFT_ARM` | 4 |
| `RAGDOLL_BONE_LEFT_FORE_ARM` | 5 |
| `RAGDOLL_BONE_LEFT_HAND` | 6 |
| `RAGDOLL_BONE_RIGHT_ARM` | 7 |
| `RAGDOLL_BONE_RIGHT_FORE_ARM` | 8 |
| `RAGDOLL_BONE_RIGHT_HAND` | 9 |
| `RAGDOLL_BONE_LEFT_UP_LEG` | 10 |
| `RAGDOLL_BONE_LEFT_LEG` | 11 |
| `RAGDOLL_BONE_LEFT_FOOT` | 12 |
| `RAGDOLL_BONE_LEFT_TOES` | 13 |
| `RAGDOLL_BONE_RIGHT_UP_LEG` | 14 |
| `RAGDOLL_BONE_RIGHT_LEG` | 15 |
| `RAGDOLL_BONE_RIGHT_FOOT` | 16 |
| `RAGDOLL_BONE_RIGHT_TOES` | 17 |

Valid values are 0 to 17.


**BoneDirection** - used by `SpringBoneSimulator3D.set_setting_end_bone_direction`.

| Constant | Value |
|---|---|
| `BONE_DIRECTION_PLUS_X` | 0 |
| `BONE_DIRECTION_MINUS_X` | 1 |
| `BONE_DIRECTION_PLUS_Y` | 2 |
| `BONE_DIRECTION_MINUS_Y` | 3 |
| `BONE_DIRECTION_PLUS_Z` | 4 |
| `BONE_DIRECTION_MINUS_Z` | 5 |
| `BONE_DIRECTION_FROM_PARENT` | 6 |

**RotationAxis** - used by `SpringBoneSimulator3D.set_setting_rotation_axis` and `set_joint_rotation_axis`.

| Constant | Value |
|---|---|
| `ROTATION_AXIS_X` | 0 |
| `ROTATION_AXIS_Y` | 1 |
| `ROTATION_AXIS_Z` | 2 |
| `ROTATION_AXIS_ALL` | 3 |
| `ROTATION_AXIS_CUSTOM` | 4 |

**CenterFrom** - used by `SpringBoneSimulator3D.set_setting_center_from`.

| Constant | Value |
|---|---|
| `CENTER_FROM_WORLD_ORIGIN` | 0 |
| `CENTER_FROM_NODE` | 1 |
| `CENTER_FROM_BONE` | 2 |

**ModifierCallbackModeProcess** - values of the `Skeleton3D.modifier_callback_mode_process` property.

| Constant | Value |
|---|---|
| `MODIFIER_CALLBACK_MODE_PHYSICS` | 0 |
| `MODIFIER_CALLBACK_MODE_IDLE` | 1 |
| `MODIFIER_CALLBACK_MODE_MANUAL` | 2 |

