#!/usr/bin/env bash
# Builds Assets/Levels/esquadra_policia.glb from the exported level (bk/esquadra_policia_orig.glb),
# every step in order. Each step reads the previous one's output.
#   1 split_breakables      glass, screens, cars, extinguishers, briefing furniture as breakables
#   2 screen_logo           the CTW logo on the computer screens (their UVs mirror it)
#   3 close_street          walls at both ends of the street
#   4 drop_black_lightmaps  no pitch-black lightmap on the objects the bake left out
#   5 floor_doors           door leaves set on their floor (the export left them off by ~1.4 m)
#   6 lift_decals           floor overlays a few mm over the tiles (no z-fighting)
#   7 fix_vans              the garage vans where they were set in Blender (bk/vans_reference.glb)
#   8 dedupe_faces          no doubled faces on double-sided materials (the fan grille flickered)
#   9 halo_blend            the light halos (projector beam) see-through and glowing, not opaque
# then minimap             the HUD minimap pictures, one per storey (Assets/Textures/Minimap)
set -euo pipefail
cd "$(dirname "$0")/.."
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
python3 tools/split_breakables.py bk/esquadra_policia_orig.glb "$work/1.glb"
python3 tools/screen_logo.py "$work/1.glb" "$work/2.glb" ComputerScreen01 Assets/Textures/Logos/ctw.png 15 162 113 233 --mirror
python3 tools/close_street.py "$work/2.glb" "$work/3.glb"
python3 tools/drop_black_lightmaps.py "$work/3.glb" "$work/4.glb"
python3 tools/floor_doors.py "$work/4.glb" "$work/5.glb"
python3 tools/lift_decals.py "$work/5.glb" "$work/6.glb"
python3 tools/fix_vans.py "$work/6.glb" bk/vans_reference.glb "$work/7.glb"
python3 tools/dedupe_faces.py "$work/7.glb" "$work/8.glb"
python3 tools/halo_blend.py "$work/8.glb" Assets/Levels/esquadra_policia.glb
python3 tools/minimap.py Assets/Levels/esquadra_policia.glb Assets/Textures/Minimap
