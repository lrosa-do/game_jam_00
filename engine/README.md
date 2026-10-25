# prisma3d

A 3D engine in C++17 on top of the [prisma](https://github.com/akadjoker/prisma) rendering backend.

Status: the library opens a window, creates the rendering driver for it and has a CPU image class that loads and saves PNG, JPG, BMP and TGA and a file system with search folders and zip archives. The demos and their assets live in [prisma3d-demos](https://github.com/akadjoker/prisma3d-demos).

## Layout

```text
libprisma3d/    the engine library
tests/          the tests, run by ctest; no window is opened
tools/          the pack tool and the Blender add-on
external/       submodules
docs/           design notes
```

## Submodules

| Path | Used for |
|---|---|
| `external/prisma` | rendering backend: OpenGL 4.6, OpenGL ES 3 and Vulkan behind one interface |
| `external/containers` | containers, strings, threads and streams (`ct`) |
| `external/math` | vectors, matrices, quaternions and bounding volumes (`mathc`) |
| `external/zen_plataform` | window, input and files |
| `external/iGUI` | user interface: in-game, debug panels and the editor |
| `external/Kinetix` | physics and AI |
| `external/zenpy` | scripting language |

Several of these bring their own copy of `containers` and `math` as nested submodules. The engine uses only the copies in `external/`, so the nested ones do not need to be fetched.

## Getting the source

```sh
git clone https://github.com/akadjoker/prisma3d.git
cd prisma3d
git submodule update --init
```

## Build

Requires CMake 3.21, a C++17 compiler, Python 3 (used by the glslang build) and the OpenGL development files. Shaders are compiled at run time: the GLSL sources are embedded once (about 100 KB compressed), the embedded glslang turns the variant a material needs into SPIR-V on first use, SPIRV-Cross derives the OpenGL text from it, and the results are cached in `~/.cache/prisma3d/shaders` (`XDG_CACHE_HOME` is honoured). A `shaders/` folder in the project or in a pack overrides the embedded sources. The Vulkan backend is built when the Vulkan SDK is found. The editor and the runner start on Vulkan and fall back to OpenGL when it is not available; the `opengl` argument forces OpenGL. `ccache` and `mold` are used automatically when installed (`PRISMA3D_USE_CCACHE` and `PRISMA3D_USE_MOLD` turn them off), and every library is built with position-independent code. Warnings are errors in the engine, editor, runner, tools and tests (`-DPRISMA3D_WERROR=OFF` turns that off, for a newer compiler that brings new warnings).

```sh
cmake -S . -B build
cmake --build build
```

## Blender add-on

`tools/blender/io_prisma3d` adds the prisma3d formats to Blender 4.2 or newer.

- File > Export > prisma3d (.pmesh) writes `<name>.pmesh`, a `.pskel` per skeleton, one `<name>_<action>.panim` per action, the `<name>.mat` material library and the textures beside them. The files are byte for byte what the editor's Convert to .pmesh writes for a glTF export of the same scene. With one armature in the export every action of it is written; mark the actions with a fake user so Blender keeps them.
- File > Import > prisma3d (.pmesh) reads the mesh with its skeleton, every linked clip and the `.mat` with its textures: each clip becomes an action, stashed in the armature's NLA, the first one active.
- File > Import > prisma3d animations (.panim) adds clip files to the selected armature, its bones matched by name; the rest pose comes from the `.pskel` beside the clips that has their bones.

Build the package and install it with Edit > Preferences > Get Extensions > Install from Disk:

```
blender --command extension build --source-dir tools/blender/io_prisma3d --output-dir build
```

`tools/blender/prisma_collision` is a separate add-on for collision meshes. In Edit Mode the Prisma tab of the sidebar marks the selected faces as not colliding (or colliding again), selects the marked faces and clears the marks; the marked faces show in red in solid shading. File > Export > prisma3d collision (.glb), or the button in the same tab, writes the selected meshes without the marked faces and without materials, ready to load with `?physics=concave`. Build it the same way with `--source-dir tools/blender/prisma_collision`.

`tools/blender/flatten_collision.py` prepares a collision mesh from any glb without opening Blender: it drops everything that is not a mesh, every material, texture, UV and colour, joins all the meshes into one (shared meshes included) and can delete everything above a height, either absolute (`--max-height`, the engine's up axis) or above the ground level (`--max-height-above-ground`, measured from the height below which only 1% of the vertices are, so pieces buried under the map do not count). It never writes an empty mesh or over its own input, and adds `.glb` to the output when missing:

```
blender -b -P tools/blender/flatten_collision.py -- level.glb level_collision.glb --max-height-above-ground 30
```

## Editor

The editor (`editor`) is built with the library. The editor, the runner (`runner`) and the tools (`export`, `pack`) are written to `bin/` at the top of the source tree (`-DPRISMA3D_BIN_DIR=<folder>` changes it). On the first start, when there is no project to open (no argument and no remembered project), it opens `projects/Default` next to the binary and creates it when missing: a test scene with a floor, a box, a sun and a camera, and a `rotate.py` script on the box that runs with Run > Play and in `runner`. An existing default project is never overwritten.

### AI panel (MCP)

The **AI** tab next to Console and Assets runs a Model Context Protocol server inside the editor, so an AI client such as Claude Code can inspect and build scenes through the same editor code paths as the UI (undo, redo, dirty flag, selection and the "not while playing" rules apply, and every call is one undo step named `MCP: ...`).

Connect Claude Code with the command the panel copies for you:

```
claude mcp add --transport http prisma3d http://127.0.0.1:8787/mcp
```

Safety:

- The server is **off** until you turn on the *MCP server* switch, and it never starts by itself. It stops when the editor closes.
- It listens on `127.0.0.1` only, has no token, rejects any `Origin` or `Host` that is not `localhost` or `127.0.0.1`, and logs every request in the panel.
- Two more switches, both off by default and remembered in `prisma3d_editor.ini`: *Allow scene edits* (the tools that change the scene, materials and project files) and *Allow scripts & Play* (`write_script`, `attach_script`, `play`, `stop`; scripts run with your user's rights). A tool whose switch is off is not listed, and calling it returns an error that names the switch.
- Resource paths given to tools must be relative to the project, and setting a node's `script` property also needs the second switch.

Read tools: `get_scene_tree`, `get_node`, `list_classes`, `list_assets`, `get_selection`, `get_project_settings`, `get_console`, `screenshot` (Scene or Game view as a PNG). Edit tools: `create_node`, `set_properties`, `delete_node`, `rename_node`, `reparent_node`, `duplicate_node`, `create_primitive`, `create_material`, `assign_material`, `import_asset`, `new_scene`, `open_scene`, `save_scene`, `select_node`, `create_environment`. The protocol (Streamable HTTP, revision 2026-07-28 and the earlier initialize/session revisions) is described in `docs/PORTING.md`.

## Exporting a game

`export` packs a project for shipping: a copy of the runner and, next to it, `data.rpak` with the project's files (the manifest, the asset folder, the folders of the listed scenes and the search paths), the scripts compiled to bytecode and loaded at start in a fixed order. The runner finds the pack next to its own file and starts the project; `--embed` appends the pack to the executable instead, for a small game that should be a single file. File > Export Game... in the editor does the same.

```
export projecto.json -o out/game        # Linux
export projecto.json -o out/game.exe    # Windows
```

The platform follows the output name (`.exe` means Windows) or `--platform`. The runner to stamp comes from `templates/<platform>/` next to the tool (`runner` or `runner.exe`, built on that platform), or, for the host platform, from the runner next to the tool; `--runner <file>` names one. `-k <key>` encrypts the pack (the key is stored in the executable, so it hides the files rather than protecting them); the `export/pack_key` project setting is the default. `--keep-debug`, or the `export/keep_debug` setting, keeps line numbers in the bytecode. `pack -t game` lists what an executable carries.

Scripts may import the native modules (`math` and the others) and other `.py` files of the project: a file that defines no node class goes into the pack as source and is compiled when it is imported. Files reached through a project search path resolve inside the pack as on disk. The exporter runs each scene once, without a window, and packs the shaders they compiled, so the first start of the game does not compile them (`--no-shader-cache` skips this). Files whose name or path matches one of the `export/exclude` patterns of the project (`*.blend;*.psd;...` by default) are left out. A game writes its own files under `user://name`, a folder per user and per project created at start (`user_dir()` in scripts).

## Tests

The tests are off by default, so building the library and the editor does not build them. Turn them on to work on the engine:

```sh
cmake -S . -B build -DPRISMA3D_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build
```

The tests that read assets (terrain, pmesh, forest depth, the asset benchmarks) are not built here, because the assets live in [prisma3d-demos](https://github.com/akadjoker/prisma3d-demos). To build them against another folder pass `-DPRISMA3D_ASSETS_DIR=<folder>`; prisma3d-demos does that for you with `-DPRISMA3D_BUILD_TESTS=ON`.

## Demos

The demos and the assets are in [prisma3d-demos](https://github.com/akadjoker/prisma3d-demos), which uses this repository as a submodule.
