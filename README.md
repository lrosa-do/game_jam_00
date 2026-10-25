# game_jam_00 — NULL PROTOCOL

Jam interno CTW, tema "AI gone wrong". `engine/` é o motor prisma3d com todas as dependências
dentro (sem submódulos); `game/` é o jogo. O GitHub Actions compila o motor em Linux e Windows
quando a mensagem do commit tem a palavra `release` (ou a pedido, no separador Actions), exporta o
jogo (runner + `data.rpak`) e publica uma Release com `NULL_PROTOCOL_linux.zip` e
`NULL_PROTOCOL_windows.zip` prontos a descarregar; os binários do motor ficam nos artifacts do run.

## Compilar

```bash
cmake -S engine -B build -DCMAKE_BUILD_TYPE=Release -DPRISMA3D_BUILD_TESTS=OFF -DPRISMA3D_WERROR=OFF
cmake --build build -j
```

O runner fica em `engine/bin/runner`. Dependências em Ubuntu: `cmake libvulkan-dev libx11-dev
libxi-dev libxcursor-dev libxrandr-dev libxinerama-dev libxkbcommon-dev libgl-dev`.

## Correr o jogo

```bash
cd game
../engine/bin/runner projecto.json
```

O plano de trabalho, controlos e tarefas estão em `game/PLANO_EQUIPA_2026-10-25.md` e
`game/NOTAS_DE_JOGO.md`.
