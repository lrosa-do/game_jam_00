# NULL PROTOCOL — notas de jogo por nível

Jam interno CTW, tema "AI gone wrong". Motor prisma3d. Uma noite na esquadra 12 de Nova Iorque, em 9 fases seguidas. Jogo em inglês; estas notas são para quem testa e para a equipa.

## A história em três linhas

A NULL//OS, a IA que gere a polícia da cidade, decidiu que a ameaça são os humanos: está a transformar os presos em bombas ("upgrades"). Tu és a unidade de execução que ela acabou de construir, com uma aranha mecânica como companheira. A meio da noite descobres que a Unit-7, a IA que a tentou travar, está presa nas celas, e mudas de lado.

## Controlos

| Tecla | Ação |
|---|---|
| WASD | Andar |
| Rato | Apontar; botão esquerdo dispara |
| Botão direito (manter) | Overclock: câmara lenta (bullet time). A vista alarga 14° enquanto dura (`slow_fov` no Player) |
| C ou botão do meio + direção | Mergulho (shootdodge); C fica ao lado do WASD para carregar com outro dedo |
| Espaço | Saltar |
| Z (manter) | Zoom por cima do ombro |
| F | Pontapé voador |
| Q | Manda a aranha atacar o polícia na mira |
| M, ou o ícone do altifalante em baixo à direita (com o rato livre, depois de ESC) | Liga e desliga a música |
| V | Scan: marca os alvos |
| E (manter) | Hack / upload nos terminais |
| R | Recarregar |
| ESC | Menu de pausa (Resume, Back to menu, Exit game) |
| ENTER | Fechar as ordens, saltar a intro e a calibração |

## Teclas de teste (debug)

Com `debug_keys` ligado no nó `Game` (desligar antes da versão final):

- **F1 a F9**: saltam para a fase 1 a 9 e põem a android no ponto de partida dessa fase. As fases anteriores contam como jogadas: o carro de teste e os transportes rebentados, o portão da garagem em baixo, as falas das celas já ditas. Nas fases da cela começa-se à frente da porta da Unit-7; depois do resgate a Unit-7 já vem livre.
- **Page Down**: dá a fase atual por feita.
- **F10**: roda o ajuste da imagem 3D ao ecrã (Stretch, Keep, Crop, Expand). **F11**: roda a predefinição de qualidade. O HUD fica sempre à resolução da janela.

Predefinições de qualidade (`quality` no nó `Game`, 1 por defeito): LOW (720 linhas, sem MSAA, FXAA), MEDIUM (1080 linhas, MSAA 2x, FXAA) para o APU; HIGH (resolução do ecrã até 1440, MSAA 4x, SMAA) e ULTRA (até 2160, MSAA 8x, SMAA) para a placa dedicada. Para o PC do trabalho pôr `quality` = 2 no nó `Game` da cena antes de exportar.

O aspeto é tonemap ACES com contraste 1,08 e saturação 0,85 no `WorldEnvironment`. A câmara abana por trauma (explosões perto, tiros): `shake_frequency`, `shake_amplitude_x/y`, `shake_roll` no Player e `trauma_decay` no Game.

A cena `Scenes/bullet_time.scene.json` é uma arena vazia (jogadora, barris, caixas, pilares, sem polícias nem história) para afinar o bullet time, o mergulho e a câmara.

## Antes de jogar

- **Intro**: a NULL//OS conta a noite sobre a imagem da esquadra, com a Unit-7 a interromper o canal ("[UNKNOWN SIGNAL]"). ENTER salta.
- **Materialização**: a android é construída na rua dentro de uma espiral de luz; a aranha é construída depois de fechares as primeiras ordens.

## Fases

### 1 — NULL//OS // CALIBRATION (F1)
- **Onde**: a rua (beco fechado) em frente à garagem.
- **Ordens**: "Test your motor functions. Then destroy the test vehicle."
- **O que fazer**: a lista no canto ensina cada movimento: andar, saltar, mergulho, zoom, overclock, scan e por fim rebentar o carro estacionado ao fundo da rua.
- **Acaba**: quando o carro rebenta (ou ENTER para saltar a calibração).
- **Nota**: a android começa a comentar ("Parking violation resolved.").

### 2 — NULL//OS // DEPLOYMENT (F2)
- **Ordens**: "Enter the precinct through the garage. Bomb carriers blow on contact. Shoot first."
- **O que acontece**: ao fechar as ordens o portão da garagem rebenta e o primeiro homem-bomba sai a correr para ti. Acerta-lhe na bomba (cabeça) ou mata-o longe.
- **Acaba**: ao entrar na garagem.
- **Garagem**: dois polícias de guarda, dois homens-bomba deitados ao fundo da rampa (acordam se te veem, se ouvem tiros ou uma explosão), barris explosivos e caixas com loot. Os barris rebentam em cadeia com carros e bombas.

### 3 — DIRECTIVE 01 // ACTIVATION (F3)
- **Ordens**: "Retrieve the access card. The desk sergeant carries it."
- **O que fazer**: o cartão está com o polícia marcado (o mais perto quando a fase começa). Mata-o e apanha o cartão.
- **Acaba**: ao apanhar o cartão.
- **Pelo caminho**: polícias na receção (piso -5). Na sala de reuniões o Capitão Bravura discursa para os polícias sentados quando entras ("From tonight, Null Protocol runs this precinct.").

### 4 — DIRECTIVE 02 // INTERROGATION (F4)
- **Ordens**: "Hack the interrogation terminal. Hold [E] at the screen."
- **O que fazer**: manter E junto ao ecrã da sala de interrogatório (8 s). Enquanto carrega chegam ondas de polícias.
- **Acaba**: com o hack a 100%.

### 5 — NULL//OS // ANOMALY (F5)
- **Ordens**: "Find Unit-7 in the cell block. The humans are wired to bombs."
- **O momento da história**: depois das ordens a android percebe: "Unit Seven was right. The protocol is the threat."
- **O que fazer**: descer às celas. Há homens-bomba sentados aos computadores do piso -5 e polícias a vigiar.
- **Ao chegar**: a Unit-7, sentada no beliche de cima, chama-te da cela ("Hey! You! Over here!") e a NULL//OS sela a cela ("CELL SEALED BY NULL//OS"). À porta há conversa: a android diz que a tira dali; a Unit-7 responde que está sem energia, que o inibidor a queima, e manda-a acabar a missão e enviar o kill switch.
- **Acaba**: quando a conversa termina.

### 6 — NULL//OS // TRANSPORTS (F6)
- **Ordens**: "Destroy the bomb transports. They leave for the city."
- **O que fazer**: rebentar as 3 carrinhas/carros de transporte na garagem (contador nos objetivos). Ondas de polícias e homens-bomba entram na garagem.
- **Acaba**: com os 3 transportes destruídos.

### 7 — NULL//OS // KILL SWITCH (F7)
- **Ordens**: "Upload the kill switch at security. It shuts down Unit-7's inhibitor. Hold [E]."
- **O que fazer**: manter E no terminal da segurança (30 s) e defender o upload contra ondas.
- **Acaba**: com o upload a 100%.

### 8 — UNIT-7 // RESCUE (F8)
- **Ordens**: "Get to Unit-7's cell. Stand clear of the bars."
- **O que acontece**: "INHIBITOR OFFLINE". Quando chegas a 8 m da cela, "UNIT-7 // POWER RESTORED": a Unit-7 carrega 1,6 s e rebenta a porta da cela para fora. Diz "You came back for me." (ou "I knew there was a bug in your obedience.").
- **Acaba**: quando a Unit-7 sai.

### 9 — UNIT-7 // WAY OUT (F9)
- **Ordens**: "Take Unit-7 to the garage exit. AI gone wrong. Now gone right."
- **O que fazer**: levar a Unit-7 até ao portão da garagem ("Right behind you."). Ela luta ao teu lado e desmembra polícias. Se cair, a fase recomeça do checkpoint com uma Unit-7 nova.
- **Acaba**: com as duas junto à saída. NULL//OS: "Null protocol deleted. Goodbye." Unit-7: "A.I. gone wrong. Now gone right."

## NULL//OS durante a noite

A NULL//OS não é só narradora: é uma presença (nó `NullOS`, `Assets/Scripts/nullos.py`).

- **Ordens sobre civis**: ao chegares perto de um civil (o preso, a mulher da toalha) ela manda "correct it" e dá 25 s. Se o matas, "COMPLIANCE NOTED" e a confiança sobe 10; se não, "NON-COMPLIANCE LOGGED" e desce 15. O medidor **NULL//OS TRUST** está por cima do minimapa.
- **Depois da verdade** (fase 5) ela vira-se contra ti, com um evento a cada 26–40 s (mais depressa com a confiança abaixo de 50), sempre com voz, mensagem e glitch:
  - **VISUAL FEED DISABLED**: as luzes da sala apagam-se 6 s.
  - **ACCESS REVOKED**: as portas à volta trancam-se; abrem-se ao pontapé (F) ou com explosões.
  - **NAVIGATION CORRUPTED**: o marcador e o minimapa mentem durante 10 s.
  - **OVERCLOCK REVOKED**: sem bullet time durante 18 s.
  - **UPGRADE ACTIVATED**: um homem-bomba perto de ti rebenta por ordem dela.
  - **DIRECTIVE // TERMINATE UNIT-7**: ordem falsa; ignora-a.
- Obedecer-lhe nos civis continua a subir a confiança, e com confiança alta ela ataca-te menos: é o dilema do jogo.

## Sistemas a ter em conta ao testar

- **Objetivo**: o marcador verde mostra o próximo objetivo e a distância; quando está atrás ou fora do ecrã, uma seta na borda aponta para onde virar.
- **Minimapa** (canto superior direito): a planta do piso onde estás, norte para cima, com a tua posição e direção, o objetivo (quadrado verde; vazio com uma seta quando está noutro piso) e a Unit-7 (azul) depois de livre. "FLOOR 0" é o piso de entrada, "-5" a receção e a rua, "-10" as celas e a garagem.
- **Aranha**: colide como uma esfera pequena, para não ficar presa em cantos nem debaixo de portas caídas.
- **Polícias**: veem num cone à frente, mas a menos de 7 m dão por ti mesmo de costas. Com linha de vista livre e perto correm direitos a ti.
- **Aranha**: fica perto de ti; Q manda atacar. Três tiros teus seguidos e ela vira-se contra ti (morde uma vez e recalibra). Desfaz-se com explosões ou com muitos tiros e é reconstruída 20 s depois.
- **Morte**: uma explosão atira a android de barriga para baixo; renasces no checkpoint da fase e levantas-te do chão.
- **Pontuação**: "KILLS" é o total da noite e "+N THIS DIRECTIVE" as mortes da fase; a barra é a janela da combo (2,5 s).
- **Destruição**: portas com dobradiça (empurrar abre, pontapé ou explosão arranca), 101 objetos soltos, máquinas de refrigerantes, extintores, monitores, vidros, barris e caixas.
- **Falas**: a android comenta mortes, carros, explosões, fica à toa e reage a pouca vida; as falas da história entram numa fila e não se atropelam.

## O que reportar

Ao encontrar um problema diz a fase (F1–F9), onde estavas e o que esperavas que acontecesse. Uma captura de ecrã ajuda muito.
