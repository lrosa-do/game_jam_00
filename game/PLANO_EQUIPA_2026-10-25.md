# NULL PROTOCOL — plano de equipa, 25/10/2026 (últimas horas do jam)

Jogo do jam interno CTW ("AI gone wrong"), feito no motor próprio prisma3d. Shooter em terceira pessoa
na esquadra: a NULL//OS, a IA da polícia, manda na jogadora (uma androide) e mente-lhe pelo HUD.
Este documento é para quem entra agora: o que existe, como se corre, e tarefas que se podem pegar
em paralelo sem pisar os outros.

## Como obter e compilar

O repositório tem o motor e o jogo juntos, sem submódulos. Para começar:

```bash
git clone https://github.com/lrosa-do/game_jam_00.git
cd game_jam_00
cmake -S engine -B build -DCMAKE_BUILD_TYPE=Release -DPRISMA3D_BUILD_TESTS=OFF -DPRISMA3D_WERROR=OFF && cmake --build build -j   # runner em engine/bin/runner
```

Dependências de sistema (Ubuntu): `build-essential cmake libvulkan-dev libx11-dev libxi-dev
libxcursor-dev libxrandr-dev libxinerama-dev`. Ninguém precisa de mexer no motor: bugs do motor
reportam-se com a cena e a captura.

## Como correr

```bash
cd game
../engine/bin/runner projecto.json            # o jogo, ecrã inteiro
../engine/bin/runner Scenes/bullet_time.scene.json   # arena de teste (sem polícias)
../engine/bin/runner Scenes/map_view.scene.json      # mapa com câmara livre
```

Opções: `--windowed --size 1600x900`, `--profile` (médias no fim), `--record DIR --fps 30` (frames para
vídeo). Atalhos do motor: Shift+F1 estatísticas, Shift+F9 profiler, Ctrl+X sai já.

Controlos e teclas de debug: `NOTAS_DE_JOGO.md`. Qualidade: F11 roda LOW/MEDIUM/HIGH/ULTRA (MEDIUM por
defeito, para APU; num PC com placa dedicada pôr `quality` = 2 no nó `Game` da cena).

Regressão da história (headless, 4 minutos): copiar `tools/playthrough_test.py` para
`Assets/Scripts/zz_play_test.py`, acrescentar um nó `ZTest` com esse script a uma cópia de
`Scenes/game.scene.json`, correr `--headless --fps 60 --frames 14000` e procurar `MARK FINISHED`
sem linhas `[ERROR]`. Correr antes de cada commit que toque em scripts.

## O que está feito

- Música (Max Payne 2): `corruption` nas ordens, `nightmare` depois da verdade, `theme` no fim; baixa
  debaixo das vozes. M ou o ícone do altifalante em baixo à direita liga e desliga.
- Abanão de câmara por trauma (fórmula do Kinetix2D) nas explosões e nos tiros; bullet time no
  botão direito com a vista a alargar; mergulho em C ou botão do meio; salto com mais força em
  bullet time.
- Aspeto: tonemap ACES, contraste 1,08, saturação 0,85 (WorldEnvironment das três cenas).
- Motor: MSAA, FSR 1.0, SMAA, modos de ajuste do 3D ao ecrã, predefinições de qualidade, sombras de
  materiais de dupla face corrigidas, z-fighting dos papéis do balcão corrigido no glb.
- Diretor NULL//OS (`Assets/Scripts/nullos.py`): ordens sobre civis com TRUST e eventos hostis depois
  da verdade. Testado só em headless.

## Tarefas para pegar (cada uma independente; escrever o nome ao lado ao começar)

1. **Ambiente sonoro** — `Assets/Sounds/`, `game.py` (`make_sounds`, `sound()`): um loop de esquadra
   (ventilação, zumbido elétrico, rádio ao longe) a tocar baixo (0,2) desde o início, e um segundo
   loop mais tenso depois da verdade (`story._truth_said`). Aceitação: nunca há silêncio total.
2. **Jogar o Diretor em janela e afinar** — `nullos.py`: `EVENT_GAP` (26 a 40 s), legibilidade do
   apagão e do lockdown, se a navegação a mentir se percebe, se "ACCESS REVOKED // KICK [F]" se lê a
   tempo. Cortar os eventos que parecem bugs. Entregar: lista do que ficou e do que saiu.
3. **Ecrã de morte e ecrã final** — `game.py` `player_died`, `story.py` `finish`: na morte, flash
   vermelho, câmara lenta 1 s e o texto; no fim, ecrã de resultados (KILLS, TEMPO, HUMANS
   TERMINATED / SPARED, TRUST, veredito "gone right" só se poupou civis), ENTER volta ao menu.

4. **Menu inicial** — `Scenes/boot.scene.json`, `boot.py`: fundo com `Assets/Textures/Menu/menu_street.png` e `menu_lab.png` (1672x941) e o logo `logo.png` (com alfa), já na pasta; START (ENTER ou clique), QUIT (ESC), logo CTW; antes de
   exportar `activeScene` passa a `Scenes/boot.scene.json`.
5. **Teste no PC com NVIDIA** — correr com `quality` = 2 e 3, anotar fps com Shift+F1 nas três zonas
   (garagem, escritório, celas) e reportar. Se HIGH aguentar 60, fica HIGH por defeito no export.

6. **Floater (inimigo voador)** — `Assets/Prefabs/floater.scene.json` e `Assets/Scripts/floater.py`
   já existem com o modelo e um esqueleto com os passos em comentário: aparece a meio do mapa, voa
   na direção da jogadora a `hover_height`, dispara a cada `fire_rate` dentro de `fire_range`, leva
   dano com `take_damage` e explode. O modelo tem esqueleto mas **não tem clips de animação**: anima-se
   por poses de ossos no `Skeleton3D` (asas, antena, braços), como diz o cabeçalho do script. Referência: `enemy.py`. Quando tiver `take_damage`, entra no
   grupo `enemy` para a jogadora o poder abater.
7. **Export** — `debug_keys` = False no nó `Game`, `activeScene` = boot, `quality` escolhido, e o
   pacote com `bin/pack`. Confirmar que `source/` (blend) fica fora.

## Regras

- Não mexer no motor (repo prisma3d); bugs do motor reportam-se com a cena e a captura.
- Um commit por tarefa, mensagem curta em português a dizer o que mudou.
- Antes de commit com scripts: correr a regressão acima e o jogo em janela 2 minutos.
- O que não couber nas horas que faltam, deixar escrito aqui em "Por fazer" com o ponto onde ficou.

## Por fazer

(escrever aqui)
