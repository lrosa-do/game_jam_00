#!/usr/bin/env bash
# Records the trailer (tools/trailer.py drives the shots) frame by frame with the runner, then cuts
# it with ffmpeg: the narration track under the pictures, a few of the game's sounds on their
# cues (the engine's own sound is not recorded headless), a fade in, and the end card.
# Usage: tools/trailer.sh out.mp4 [work_dir]
set -euo pipefail
cd "$(dirname "$0")/.."
out="${1:-trailer.mp4}"
work="${2:-$(mktemp -d)}"
runner=/media/projectos/projects/cpp/prisma3d/bin/runner
fps=30
seconds=50
mkdir -p "$work/frames"

cp tools/trailer.py Assets/Scripts/zz_trailer.py
python3 - <<'EOF'
import json
d = json.load(open('Scenes/game.scene.json'))
root = d['root']
root['properties']['intro'] = False
root['children'].append({'class': 'Node', 'name': 'Trailer', 'properties': {'script': 'Assets/Scripts/zz_trailer.py'}})
root['children'].append({'class': 'Camera3D', 'name': 'ZCam', 'properties': {'near': 0.05, 'fov': 62.0}})
# A fill light that rides with the free camera (the script sets its energy per shot).
root['children'].append({'class': 'OmniLight3D', 'name': 'ZLight', 'properties': {'omni_range': 9.0, 'light_energy': 0.0, 'light_color': [1.0, 0.93, 0.85, 1.0]}})
json.dump(d, open('Scenes/zz_trailer.scene.json', 'w'), indent=1)
EOF
"$runner" Scenes/zz_trailer.scene.json --headless --fps $fps --frames $((fps * seconds)) --size 1280x720 --record "$work/frames" 2>&1 | grep -i "error\|warn" | head -5 || true
rm -f Scenes/zz_trailer.scene.json Assets/Scripts/zz_trailer.py
echo "frames: $(ls "$work/frames" | wc -l)"

# Title and end cards (tools/trailer_cards.py writes them next to the frames).
python3 tools/trailer_cards.py "$work"

snd=Assets/Sounds
# Sounds on their cues (milliseconds): the car, the gate, the shots of the dive, the cell door.
ffmpeg -y -loglevel error \
  -framerate $fps -i "$work/frames/%05d.png" \
  -i "$snd/voice/intro.ogg" \
  -i "$snd/explosion/car_a.wav" \
  -i "$snd/break/door_metal_break_A.ogg" \
  -i "$snd/weapons/shoot_beretta.ogg" \
  -i "$snd/explosion/explosion_medium_A.ogg" \
  -loop 1 -framerate $fps -i "$work/title.png" \
  -loop 1 -framerate $fps -i "$work/end.png" \
  -filter_complex "\
[0:v]fade=t=in:st=0:d=1.5,fade=t=out:st=$((seconds - 1)):d=1[v0]; \
[6:v]format=rgba,fade=t=in:st=1.0:d=1.0:alpha=1,fade=t=out:st=4.5:d=1.0:alpha=1[title]; \
[7:v]format=rgba,fade=t=in:st=46.5:d=1.2:alpha=1[end]; \
[v0][title]overlay=0:0:enable='between(t,0,6)'[v1]; \
[v1][end]overlay=0:0:enable='gte(t,46.5)',trim=duration=${seconds}[v]; \
[1:a]afade=t=out:st=$((seconds - 3)):d=3,volume=1.0[nar]; \
[2:a]adelay=17000|17000,volume=0.9[car]; \
[3:a]adelay=20500|20500,volume=0.8[gate]; \
[4:a]adelay=27400|27400,volume=0.6[s1]; \
[4:a]adelay=27900|27900,volume=0.6[s2]; \
[4:a]adelay=28400|28400,volume=0.6[s3]; \
[4:a]adelay=29000|29000,volume=0.6[s4]; \
[5:a]adelay=46000|46000,volume=0.9[cell]; \
[nar][car][gate][s1][s2][s3][s4][cell]amix=inputs=8:normalize=0:duration=first[a]" \
  -map "[v]" -map "[a]" -c:v libx264 -preset slow -crf 19 -pix_fmt yuv420p -c:a aac -b:a 160k -shortest "$out"
echo "wrote $out ($(ffprobe -v error -show_entries format=duration -of csv=p=0 "$out") s)"
