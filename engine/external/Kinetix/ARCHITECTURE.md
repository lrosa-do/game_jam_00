# Kinetix

Física 2D, física 3D e IA como três bibliotecas independentes, sobre um núcleo comum. Núcleo, 2D, IA e 3D (incluindo `Ragdoll.cpp`, via `SkeletonView`/`RagdollBoneMap`) compilam sozinhos fora do motor de origem. As secções abaixo são o registo histórico da migração; onde dizem que algo "falta", ver a secção **Ragdoll sem motor** e o estado atual nesta tabela.

## Estado

| Módulo | Alvo CMake | Namespace | Compila standalone |
|---|---|---|---|
| núcleo | `kinetix-core` | `kinetix::` | sim |
| 2D | `kinetix-2d` | `kx` | sim |
| IA | `kinetix-ai` | `kinetix::AI` | sim |
| 3D | `kinetix-3d` | `kinetix::Physics` | sim (o esqueleto do `Ragdoll` entra por `SkeletonView`, implementado no bridge) |

`grep -rn 'Radion::'` em `core/`, `2d/`, `3d/` e `ai/` não dá nada — o acoplamento ao `Skeleton` do motor foi substituído pela interface `SkeletonView`. Todos os `#ifndef RADION_..._H` viraram `KINETIX_..._H`, todos os alvos CMake são `kinetix-*`. Build de raiz sem flags já compila os quatro.

## Camadas

```
Motor Radion        Motor Kinetix2D        Novo projeto
     |                     |                     |
radion-bridge        legacy2d-bridge             |
(a criar)             (bridges/legacy2d)         |
     |                     |                     |
kinetix-3d            kinetix-2d ---------------- +
kinetix-ai                 |
     |                     |
     +-------- kinetix-core +
                     |
          external/containers (ct)
          external/math (mathc)
```

Regra única: nada dentro de `kinetix-*` inclui header de fora de `kinetix-*` ou de `external/`. Quem precisa de render, animação, ECS ou serialização do motor passa por uma interface, implementada do lado do bridge.

## Núcleo — Math / Types / Log / Containers

`Radion::Math.h` já era um shim GLM-shaped por cima do mathc (`#include <mathc.h>`, `vec3`/`quat`/`dot`/`normalize`/etc.) — não uma matemática diferente. Portar não foi reescrever call sites de 3D/IA, foi **relocar** o shim: `core/include/kinetix/core/Math.h` + `Math.cpp` são cópia direta, namespace `Radion`→`kinetix`, sem `std::`/`<algorithm>` (usa `kinetix::Min/Max/Clamp` de `Algorithm.h` e `<math.h>` bare). `Types.h` (u8/f32/usize/...) e `Containers.h` (`Handle<Tag>`, `Pool<T,H>`, `HashMap`→`ct::HashMap`) migraram do mesmo jeito. `Log.h` manteve a API estática (`Log::info/warning/error`) mas a implementação trocou `SDL_LogMessage` por `fprintf` simples — SDL é do motor, não da lib.

Bug real encontrado no processo: o `Radion::Math.h` original referenciava `::Mathc::Vec2` (maiúsculo, com C no fim), mas a lib em `external/math`/`vendor/mathc` (mesmo commit `868bdd6` nos dois) declara `namespace Math`, não `Mathc`. Ou seja — **esse shim nunca compilou contra o mathc que o Radion tem pinned**, provavelmente por causa de alguma migração de nome do lado da lib que não voltou a acontecer no Radion. Corrigido aqui (`::Math::Vec2`); vale espreitar se o Radion em si compila hoje.

Testado com um smoke test próprio (vec3/quat/AABB/Handle/Pool/HashMap/Log) — valores batem (dot, cross, rotação por quaternion).

`ct::HashMap` só tinha `begin()`/`end()` não-const — quebrava range-for dentro de métodos `const`. Corrigido (`const_iterator`/`cbegin`/`cend`) direto no submódulo `external/containers` (repo próprio); falta subir pro `akadjoker/containers` upstream — o commit fixado atual do submódulo já inclui o fix (`HashMap::begin() const`/`cbegin`), e o `kinetix-ai` compila contra ele.

`ai/` e `3d/` portados: `Radion::`→`kinetix::` (mecânico, só o token, sem tocar `Geometry`/`Physics`/`AI`/`detail`), `Math.h`/`Types.h`/`Log.h`/`Containers.h`/`PCH.h` apontam pro núcleo novo. Verificado por fora (build real, não só o relato das agentes). Fica pendente pra fase seguinte: `Color.h`/`BoundsTree.h`/`Skeleton.h`/`ConvexHullComputer.h`/`VoronoiShatter.h`/`DebugDraw3D.h` (3D) e `ByteArray.h`/`FileSystem.h`/`RadionFormat.h`/Recast+Detour (IA) — esses, quando migrarem, têm de cair direto em `namespace kinetix` (não `kinetix::Physics`), porque hoje código dentro de `kinetix::Physics` referencia `Color`/`Skeleton`/etc. sem qualificar, contando com o lookup por namespace envolvente que existia com `Radion::Physics` dentro de `Radion`.

NavMesh (IA) está a ser reescrito sem Recast/Detour — ver seção IA abaixo.

## Núcleo — submódulos

`external/containers` (`akadjoker/containers`, target `ct`) e `external/math` (`akadjoker/math`, target `mathc`) são submódulos git, não cópias. `kinetix-core` só adiciona `Types.h` e `Log.h` (interface) por cima.

IA já está em `kinetix-core`. 3D ainda usa `Math.h`/`Containers.h` do Radion — falta portar.

## 2D

`2d/` = só o namespace `kx` (colisão, corpos, joints, dynamictree). Compila com `kinetix-core` e nada mais.

O namespace `k2d` (`BoxCollider2D`, `RigidBody2D`, `PhysicsWorld2D`, `Physics2DSerializer`, ...) não é física — é o wrapper de componente ECS do motor Kinetix2D, acoplado a `GameObject.h`/`Component.h`/`CanvasRenderer.h`/`RenderQueue.h`/`Profiler.h`/`Serializer.h` (`/media/ctw04578/data/projects/zen/Kinetix2D/engine/include/k2d/`). Foi movido para `bridges/legacy2d/` — só compila dentro do motor Kinetix2D, passando `KINETIX2D_ENGINE_INCLUDE_DIR`.

`kx/common.h` importava `k2d::Matrix2D` (`using Transform = k2d::Matrix2D`) — física de baixo nível dependendo do wrapper de alto nível. Isso quebrava o build standalone. Agora `kx::Matrix2D` é definido no próprio `common.h`, copiado do `k2d::Matrix2D` original, sem os métodos que só o wrapper usava (`ToMat4`, `AffineInverse`, os construtores em graus).

`Collider2D`/`BoxCollider2D`/`CircleCollider2D`/`PolygonCollider2D`/`EdgeCollider2D`/`ChainCollider2D` deixaram de ser `Component` — um `kx::Body` já é dono das suas shapes (`AddBox`/`AddCircle`/`AddPolygon`/`AddEdge`/`AddChain`, `kMaxShapes = 32`), não precisava de mais uma camada de Component por cima. `RigidBody2D` agora tem `addBoxShape`/`addCircleShape`/`addPolygonShape`/`addEdgeShape`/`addChainShape` (`ShapeDesc2D`, `bridges/legacy2d/include/k2d/ShapeDesc2D.h`) e `PhysicsWorld2D::attachShapes` monta o `kx::Body` direto a partir dessa lista, sem `componentCount<Collider2D>()`. `Physics2DSerializer` serializa `shapes` como array dentro do `RigidBody2D`, não mais como 5 tipos de Component separados. `ComponentType::Collider` fica sem uso — não mexi no `Component.h` do motor Kinetix2D (enum partilhado por todo o engine, fora do escopo daqui).

`kinetix-2d` não usa `std::` em lado nenhum — matemática/string via `<math.h>`/`<string.h>` (globais, não `std::sqrt`/`std::memcpy`), containers via `ct::`. Cada header em `2d/include/kx/` (incluindo `joints/` e `internal/`) compila sozinho, testado isolando cada um numa TU própria. `kx.h` (umbrella) parava de ser só conveniência — puxava `internal/collide.h`/`internal/triangulate.h`/`internal/imageshape.h` (implementação, não API) pra qualquer motor que a incluísse; removido. Um motor que só quer corpo+mundo inclui só `kx/body.h`+`kx/world.h`, sem joints/tilemap/debugdraw/internal.

## 3D

`3d/` (`radion_physics`) depende de `DebugDraw3D.h` (render), `Skeleton.h` (animação — usado em `Ragdoll`), `ConvexHullComputer`/`VoronoiShatter`/`BoundsTree` (geometria), além de `Math.h`/`Containers.h`. É o módulo mais preso e o maior (~13k linhas).

`std::` removido (`std::vector`→`ct::Vector`, `std::string`→`ct::String`, `std::array`→`ct::Array`, `<algorithm>`→`kinetix::Find`/`Sort`/`RemoveIf`/... de `core/include/kinetix/core/Algorithm.h`, matemática→`sqrtf`/`fabsf`/... de `<math.h>`). Único resto: `std::initializer_list` em `Ragdoll.cpp` (tipo do compilador, não dá pra tirar sem perder o `{...}`). `Math.h`/`Containers.h`/`Skeleton.h`/`DebugDraw3D.h` continuam Radion — isso é a próxima fase.

## IA

`kinetix-ai` está 100% livre de motor: `std::` removido, `Math`/`Types`/`Log`/`Containers` do núcleo, e o `NavMesh` reescrito sem Recast/Detour.

`NavMesh` antigo chamava Recast (voxeliza→filtra→contorno→polígono) pra construir e Detour pra `findPath`/`nearestPoint`/`moveAlongSurface`/save-load. Reescrito sem voxelização: um triângulo walkable (passa no teste de slope) já é o polígono do navmesh; dois triângulos são vizinhos se partilham aresta, ou — pra pedaços que não tocam exatamente (degrau) — se as arestas abertas ficam a menos de `edgeWeldDistance` em XZ e `agentMaxClimb` em Y. `findPath` é A* sobre essa adjacência + funnel algorithm (string-pulling) pra suavizar o caminho; `nearestPoint`/`moveAlongSurface` usam closest-point-on-triangle e point-location com clipping na aresta. save/load foi descartado por agora (formato próprio, sem bytes crus do Detour) — `ByteArray.h`/`FileSystem.h`/`RadionFormat.h` saíram do módulo inteiro com isso.

Testado com mesh sintética (retângulo plano, e dois pedaços em alturas diferentes ligados só pelo weld) — `findPath` atravessa o weld corretamente, slope filtra parede.

Erosão pelo raio do agente (`NavMeshConfig::agentRadius`, ver "Erosão pelo raio do agente" mais abaixo) foi a limitação conhecida desta v1 — deixou de o ser.

O A* e a solda já não são os da primeira versão — ver "Otimizações" no fim.

## Build

```
cmake -S . -B build
cmake --build build
```

`KINETIX_BUILD_2D`/`KINETIX_BUILD_3D`/`KINETIX_BUILD_AI` `ON` por default — os quatro alvos compilam de raiz, sem flags. (Este parágrafo dizia que o 3D ficava `OFF` até portar; ficou desatualizado quando o porte acabou.)

## 3D — Color / BoundsTree / DebugDraw3D / geometria

Por decisão explícita, `Skeleton.h` fica de fora (é do sistema de animação do motor, é o `radion-bridge` que vai mapear `Ragdoll` pra ele). Os outros 5:

- **`Color`** — `core/include/kinetix/core/Color.h` — portado direto, zero dependência de motor.
- **`BoundsTree`** — `core/include/kinetix/core/BoundsTree.h` — BVH genérico (AABB/Sphere/Frustum/Ray), zero dependência de motor. Portado direto.
- **`DebugDraw3D`** — `core/include/kinetix/core/DebugDraw3D.h` — só acumula `DebugLine3D`/`DebugTriangle3D` em vetores, nunca toca GPU. O `DebugPass`/`createDebugPass()` do original (que sim usa `AssetManager`/`Batch`/`RenderTechnique`) ficou pra trás — é o motor que lê `lines()`/`triangles()` e desenha. `meshVectors`/`outline`/`outlineRange` (o outline de mesh do editor, precisa de `MeshHandle`) também ficaram de fora — não são coisa de debug de física.
- **`kinetix::Geometry::ConvexHullComputer`** — `3d/include/geometry/` — é o `btConvexHullComputer` da Bullet (Preparata & Hong, aritmética exata Int128/Rational128). 2265 linhas, portado com cuidado extra pra não alterar lógica — só substituições mecânicas (containers, `std::min/max`→`kinetix::Min/Max`, tipos inteiros). `std::align_val_t`/`operator new` alinhado ficou como estava (alocação, não é container/algoritmo).
- **`kinetix::Geometry::VoronoiShatter`** — `3d/include/geometry/` — decomposição de Voronoi pra fragmentação (shatter), usa o ConvexHullComputer acima. Portado.

Os 2 bugs achados na verificação (`Radion::` sem qualificar em `CollisionShape`/`PhysicsWorld`, `ct::HashMap` tratado como `unordered_map` no cache de contactos) estão corrigidos — o cache (`removeBodyImmediate`/`warmStartFromCache`/`emitExits`/`step`) agora usa `find()`→ponteiro, `Entry{key,value}`, `erase(key)` coletado numa passagem antes de apagar (não dá pra apagar enquanto itera um `ct::HashMap`, ao contrário de `unordered_map`). `kinetix-3d` compila de raiz, sem flags — só falta `Ragdoll.cpp`.

`3d/CMakeLists.txt` voltou a incluir `Ragdoll.cpp` depois da passagem para `SkeletonView` (secção seguinte).

## Ragdoll sem motor

`Skeleton`/`LocalPose` do Radion são o sistema de animação inteiro (IK, `AnimationClip`, `AnimationManager`, ficheiro `.rskel`/`.ranim`) — `Ragdoll` só usava três coisas: `boneCount()`, `bone(i).name`, `bone(i).parent`. Em vez de portar o sistema todo (ou continuar dependente dele), `Ragdoll` passou a depender só de:

- **`SkeletonView`** (`3d/include/character/SkeletonView.h`) — interface abstrata com só `boneCount()`/`boneParent(index)`. Um bridge implementa isto por cima do skeleton real do motor.
- **`RagdollBoneMap`** — os 16 índices de osso já resolvidos (Hips, Spine, Neck, Head, ...braços/pernas). Quem resolve nome→índice é o bridge (por nome, por tabela fixa do rig, como quiser); `kinetix-3d` nunca compara strings.
- **`LocalPose`** (mesmo ficheiro) — position/rotation/scale, dado puro, cópia de 3 campos do lado do bridge.

`Ragdoll::build(skeleton, bones)` — antes só `build(skeleton)`, resolvendo por sufixo de nome dentro do próprio kinetix-3d. `bridges/legacy2d`/futuro `radion-bridge` é quem monta o `RagdollBoneMap` a partir do skeleton de verdade.

## Comparação com o Radion atual (`fix/shadow-hotpath`)

O `PhysicsWorld` standalone (959 linhas) não existe mais no Radion — foi fundido dentro de `Scene` (`fe3daa1 physics: fold the simulation into Scene`, RigidBody virou Component). Comparei `Scene::stepPhysics()` com o `PhysicsWorld::step()` daqui: **é o mesmo algoritmo** (broadphase estático+dinâmico, narrowphase, cache de contactos com warm-start, `ContactSolver::solve`, bullet sweeps, sleep) — só mudou de casa, não de lógica. Isso confirma manter `kinetix-3d` com `PhysicsWorld` standalone é a escolha certa — é exatamente o que faltava ao Radion antes de fundir tudo no Scene.

Aplicados os fixes reais encontrados nos commits mais recentes do Radion:
- **Eixo degenerado → NaN** (`f24fe3b`): `dynamics/JointAxis.h` (`detail::normalizedAxisOr`), aplicado nos construtores de `HingeJoint`/`PistonJoint`.
- **Motor não acorda corpo adormecido** (`de79895`): `Joint::wakeBodies()` (novo, em `Joint.h`/`Joint.cpp`), chamado quando o motor está ligado em `HingeJoint`, `SliderJoint`, `PointJoint`, `UniversalJoint`.

`WheelJoint` (`SoftSpring` a par), `ParticleSystem` portados a seguir — direto (construtor + `configure()`), sem `rebuild()`/`setAuthored*`/`mBuilt` (API de reconfiguração do editor Radion, não aplicável a um `Joint` sem ciclo de vida de Component). `ParticleSystem` trocou `Radion::Scene&`/`Scene::raycast()` por `PhysicsWorld&`/`PhysicsWorld::raycast()` — já existia, só não estava sendo usado por nada ainda.

`configure()` nos outros joints (Hinge/Piston/etc, API de reconfiguração dinâmica do editor Radion) continua não portado — só faz sentido se/quando existir um bridge com ciclo de vida de Component parecido.

## Comparação com o Radion atual — IA (`runtime/ai`)

Mesmo padrão do 3D: `2c9019d scene: fold the AI world into Scene and make Agent a component` fundiu o mundo de IA no Scene, também antes dos fixes recentes. Aplicados os que fazem sentido fora do Scene:

- **`ai: make the cruising nudge scale with the speed error again`** — `Behavior.cpp`: `percentDesiredSpeed` era calculado e nunca lido; o nudge usava uma constante. Uma linha.
- **`ai: stop grid paths cutting through the corner between two walls`** — `GridPathfinder.cpp`: passo diagonal só testava o destino, não as duas células ao lado — deixava cortar por um canto entre duas paredes. Um `diagonalCutsCorner()` partilhado pelas 3 buscas (A*, best-first, breadth-first).
- **`ai: let a state machine survive its own callbacks rewriting it`** — `StateMachine.cpp`: `iterate()` percorria transições com range-for enquanto callbacks do usuário (que podem remover o próprio state) corriam por dentro. Reescrito pra pinning + índice + `size()` relido a cada volta.
- **`ai: survive an agent destroyed mid-update, and form up on the right leader`** — só a parte de `FormationBehavior.cpp`: achava o líder varrendo tudo por squadId 0 e ficando com o último match — com 2 esquadrões, formava no líder errado. Virou um link `SquadLeaderEntity* mSquadLeader` de volta no `SquadEntity` (setado em `addSquadMember`/limpo em `removeSquadMember`), O(1) e correto. (A parte "survive destroyed mid-update" do commit é sobre `Scene::updateAgents()` iterando `mAgents` com buraco — não aplicável, `kinetix-ai` não tem esse loop.)

**`AgentAvoidance`** — portado: `ai/src/AgentAvoidance.h/.cpp` (`kinetix::AI::detail::applyAgentAvoidance`), a repulsão agente-a-agente que `PathfindBehavior`/`NavMeshBehavior` tinham cada um com a sua cópia (cinquenta linhas iguais, uma virou bug e a outra não). Ao contrário do `Skeleton`, isto não é acoplamento de motor — só precisa de `Entity::world().groups()`/`entities()`, que já é 100% `kinetix-ai`. `applyAvoidance()` nas duas behaviors agora só chama a função partilhada.

Não aplicado ainda: `ai: stop re-running A* every frame when the goal cannot be reached` (rate-limit de repath em `PathfindBehavior`/`NavMeshBehavior` — o `NavMeshBehavior` daqui já é outro `NavMesh`, sem Detour, precisa olhar com cuidado antes de portar).

## Comparação com o Kinetix2D atual (`feature/scene-is-the-world`)

Mesmo padrão dos dois outros: `230e683 One engine: physics lives in Scene and components` fundiu o `PhysicsWorld2D` no Scene do Kinetix2D, `world.cpp`/`body.cpp` (1523/264 linhas) foram apagados, a lógica foi pro Scene.

Mas desta vez a comparação deu diferente: **diff de cada ficheiro de `2d/` (`kx`) contra o commit imediatamente antes do fold (`230e683^`) — zero diferença de conteúdo**, em todos os 18 `.cpp` e 21 `.h`. As únicas linhas que mudam em cada ficheiro são exatamente as minhas próprias edições desta sessão (`std::sqrt`→`sqrtf` etc., `Matrix2D` relocado pra `kx/common.h`, `internal/*` fora do `kx.h`). `collide.cpp`/`manifold.cpp` foram renomeados pra `engine/src/Collide2D.cpp`/`Manifold2D.cpp` no fold mas sem nenhum commit a tocar-lhes depois — confirmei via diff, também idênticos.

Ou seja: ao contrário do 3D, o `kinetix-2d` que já está aqui **já estava com a física core (`kx`) 100% atual** — não faltava nenhum fix pra puxar. O que ficou pra trás no Kinetix2D é só a camada de Component (`k2d::RigidBody2D`/`PhysicsWorld2D`/Colliders), que também foi fundida no Scene do lado deles — mas isso é exatamente o "bridge", já isolado em `bridges/legacy2d/` e já redesenhado aqui (Collider2D deixou de ser Component, ver seção 2D) antes mesmo de saber que o Kinetix2D tinha ido pro mesmo caminho.

## Próximos passos

1. Comparar Kinetix2D atual com o `kx`/`k2d` já portado aqui, mesmo tratamento do 3D.
2. Criar `radion-bridge/` (paralelo a `bridges/legacy2d/`) para o motor Radion — `RigidBody3D`/`PhysicsWorld3D` do lado do bridge, espelhando `RigidBody2D`/`PhysicsWorld2D`, e o `SkeletonView`/`RagdollBoneMap` reais por cima do `Skeleton` do Radion.
3. Avaliar `WheelJoint`/`SoftSpring` se/quando forem precisos.
4. Se algum dia precisar de novo de I/O no NavMesh, formato próprio — não os bytes crus do Detour.

## Independência das libs e otimizações

Auditoria feita sobre o código, não sobre este documento — que é registo do que se fez, não garantia de que está certo.

**Independência.** `grep` por includes cruzados entre `2d/`, `3d/` e `ai/`: zero. As três já não se conhecem umas às outras; a única dependência partilhada é o `kinetix-core`.

**O 2D não usava o core.** Tinha a sua própria cópia de `Dot`/`Cross`/`Normalize`/`Distance`/`Min`/`Max`/`Clamp` em `kx/common.h` e falava com o mathc diretamente. Passou a `kinetix-core`: `kx::vec2` é `kinetix::Math::vec2` (alias novo em `kx/math.h`, header próprio para não obrigar quem só quer o tipo a arrastar as constantes de `common.h`), os helpers duplicados reencaminham para o core, e o CMake troca `ct mathc` por `kinetix-core`. Diff ficheiro a ficheiro contra a versão anterior, normalizando o rename do tipo: **de 41 ficheiros só `common.h` muda de substância** — o resto é rename puro.

Um helper ficou deliberadamente por reencaminhar: `kx::Normalize` devolve `(0,0)` abaixo de `kEpsilon`, enquanto `kinetix::Math::normalize` divide pelo comprimento sem guarda — para o vetor zero dá `inf`/`NaN`. Trocar um pelo outro metia NaN na física. O `Cross` 2D também fica: o core só tem o 3D.

**Código morto.** Nenhum `.cpp` fora do CMake, nenhum header órfão (o `kx.h` é umbrella pública, não é incluída internamente de propósito). Com `-Wall -Wextra` a árvore dava 2 avisos próprios, ambos reais e ambos corrigidos:
- `incidentFace()` (`3d/src/collision/Narrowphase.cpp`) recebia um `const BoxShape&` que nunca lia — a face só depende do transform e da normal, porque `BoxShape::faceNormal()` é estático. Parâmetro removido, 3 chamadas atualizadas.
- `DynamicTree::DynamicTree()` fazia `memset` sobre `TreeNode`, que tem um `union` anónimo. `new TreeNode[n]()` faz o mesmo sem o aviso, e o loop seguinte já punha `next`/`height`.

As 4 libs compilam agora com `-Wall -Wextra` e **zero avisos** (os 2 que restam são do `mathc`, `memcpy` para SIMD, intencionais).

**A\* do NavMesh: busca linear → binary heap.** A open-list era um vetor varrido do início a cada extração, com um segundo varrimento para o decrease-key. Passou a min-heap binário; o decrease-key desapareceu por completo — um nó melhorado é empurrado outra vez e a entrada obsoleta é descartada na extração pelo teste de `closed[]`, que já lá estava (*lazy deletion*), o que também tornou o `inOpen[]` desnecessário. Os centroides passaram a ser calculados uma vez para a malha toda em vez de três a quatro vezes por nó visitado.

Medido numa grelha de 12.800 triângulos, 200 buscas de canto a canto: **1.332 ms → 0.716 ms por busca (1.86×)**, com o caminho idêntico ao da versão anterior (mesmo número de pontos, mesmo comprimento).

**Solda de arestas: O(B²) → grelha espacial.** Cada aresta aberta era testada contra todas as seguintes. Como o critério de solda já é local (`edgeWeldDistance` em XZ), passou a haver uma grelha em XZ de célula `edgeWeldDistance`, com listas ligadas por índices (sem um vetor por célula), e cada aresta só olha para as 9 células à volta do seu ponto `b`.

Medido em malhas de quads soltos que só a solda liga:

| Triângulos | Antes | Depois |
|---|---|---|
| 1.800 | 2.0 ms | 0.8 ms |
| 7.200 | 13.3 ms | 3.4 ms |
| 16.200 | 41.8 ms | 7.5 ms |
| 28.800 | 98.5 ms | 13.9 ms |

O antigo quadruplica a cada duplicação do lado, o novo cresce linearmente — **7× a 28.800 triângulos**, e a diferença aumenta com o tamanho. Verificado que os casos de fronteira dão o mesmo resultado nas duas versões: solda com folga pequena e degrau baixo, rejeição por folga maior que `edgeWeldDistance`, rejeição por degrau maior que `agentMaxClimb`, e a mesma travessia na grelha soldada.

## Dois bugs no NavMesh que só apareciam com agentes a andar

Os dois estavam no HEAD, nenhum é regressão do trabalho desta sessão — confirmado compilando o teste contra o `NavMesh.cpp` do HEAD, onde reproduzem na mesma. Só apareceram agora porque nunca tinha havido um teste com `Entity`/`Group`/`World` reais a caminhar sobre a malha; os testes de caminho puro (`findPath` entre dois pontos) passavam por cima dos dois.

**Agente preso num vértice.** `moveAlongSurface()` procura a aresta que o segmento atravessa exigindo `t >= 1e-5`. Partindo exactamente de cima de um vértice ou de uma aresta — o que um agente colocado em coordenadas inteiras faz — a saída é em `t == 0`, descartada junto com as travessias para trás. Sem aresta encontrada devolvia a posição de partida, e o agente ficava ali para sempre. Agora, quando isso acontece na primeira iteração, tenta outra vez a partir de um ponto empurrado `kBoundaryNudge` na direcção do alvo.

O candidato certo não é vizinho: em cima de um vértice, o triângulo para onde o movimento vai toca o actual **só nesse ponto**. Por isso a procura é sobre o one-ring do vértice, percorrido por ligações de vizinhança (`collectVertexRing`) — e não com um `findContainingTri()`, que varre a malha inteira e custaria O(n) por agente por frame. Medido: 0.109 ms vs 0.110 ms por chamada em 7200 triângulos, ou seja, o nudge não se nota.

**Funnel em ciclo infinito.** Cada reinício do funnel rebobina `i` para `apexIndex + 1`. Isso só termina enquanto o apex avança: um par de portais que deixe `apexIndex` onde já estava rebobina `i` para o mesmo sítio e repete o passo para sempre. Dois agentes a uma fracção de distância um do outro bastam para produzir um — 1 agente corria 2000 frames, 2 agentes bloqueavam sempre no frame 5, no meio do `World::iterate()`.

Localizado com um handler de `SIGALRM` a imprimir o backtrace (sem `perf` nem `gdb` disponíveis nesta máquina), que apontou directamente para `funnelPath`. A correcção guarda o último `apexIndex` e, quando um reinício não avança, consome o portal (`++i`) em vez de rebobinar para cima dele.

Verificado que os caminhos não mudaram: `findPath` continua a dar o mesmo resultado nos testes de malha plana, weld e travessia, e a equivalência entre `NavMeshQuery` e `findPath` mantém-se em todos os orçamentos.

## Busca de caminho espalhada por frames

`findPath()` procura até ao fim antes de devolver. Para um agente é o correcto; para cinquenta a repathear no mesmo frame é um pico que nenhuma afinação por agente remove, porque o custo de uma busca é limitado pela malha e não pelo frame.

`NavMeshQuery` (`ai/include/NavMeshQuery.h`) guarda o estado da procura entre chamadas: `start()`, `step(nodeBudget)` até deixar de devolver `Searching`, `result()`. É o mesmo A* com um botão de pausa, não uma aproximação — verificado com orçamentos de 1, 7, 64, 1000 e 100000 nós, todos a produzirem exactamente o caminho de `findPath()`, com os mesmos 1007 nós expandidos. `setMaxNodes()` (10000 por omissão) abandona uma busca que ande a passear pela malha.

`findPath()` não mudou: quem o usa continua a usá-lo tal como estava.

Com 50 agentes a partir todos ao mesmo tempo, malha de 12800 triângulos:

| | pico de frame |
|---|---|
| `findPath()` síncrono, 50 agentes num frame | 20.7 ms |
| `NavMeshQuery`, 64 nós/frame, 19 frames | 0.177 ms |

O `NavMeshBehavior` passou a usá-lo: `searchNodesPerFrame` (64) e `searchMaxNodes` (10000) nas `Settings`, com `searchNodesPerFrame = 0` a manter o comportamento antigo de procurar tudo num frame. A rota anterior continua em uso enquanto a nova é procurada, por isso o agente nunca pára à espera. 40 agentes atravessam uma malha de 7200 triângulos com um pico de 9.4 ms por frame — e todos chegam.

Ideias vindas de ler o CWR (Operation Flashpoint, GPL-3.0): a busca em fatias com orçamento e tecto de iterações é o padrão do `PathPlanner.cpp` deles (`ITER_PER_CYCLE 50`, `MAX_ITER 10000`). O que está aqui é escrito de raiz sobre os nossos tipos e a nossa malha de triângulos — algoritmos não são protegidos por direitos de autor, expressão é, e nada de lá foi copiado. Continua por explorar a hierarquia de comando deles (`AICenter → AIGroup → AISubgroup → AIUnit`) e os níveis de planeamento (`LevelStrategic`/`LevelOperative`).

## A origem: Game Institute AI (2006)

`/media/projectos/projects/gametutorials/G.I.Courses - 3D Game Engine Programming/Game.Institute.AI/`, com textbook (5 capítulos), lab projects e o código dos demos em `gameinstitute/`. O `AIFinalDemo/AI_Demo` é de onde `kinetix-ai` descende: `Entity`, `Group`, `Behavior`, `PathfindBehavior`, `FormationBehavior`, `PointOfInterest`, `SquadEntity`, `State`, `StateMachine`, `Transition`, `WaypointNetwork`, `World` — os mesmos nomes, os mesmos campos.

Comparado ficheiro a ficheiro, o que temos cobre o curso e vai além dele. Falta apenas o que é do demo (`PythonScriptEngine`, `stdafx`, `NPC`) ou uma decisão diferente:

- **`VisibilityFunctor`** — interface de linha de vista que o mundo implementava para o `WaypointNetwork` consultar. Do nosso lado é o parâmetro `visibility` de `findPath` — o mesmo padrão de bridge que a regra do projecto já impõe.
- **`SquadState`/`SquadStateMachine`** — carregavam máquinas de estado de ficheiros `.stm` com Python embutido. `SquadAI.h` constrói as mesmas máquinas em código, com lambdas.
- **Depth-first** — o curso tem 5 algoritmos de grelha, nós temos 4. O que falta é o depth-first, que para caminhos serve de contra-exemplo pedagógico, não de algoritmo utilizável.

### A matemática era DirectX

O curso é `D3DXVECTOR3`/`D3DXMATRIX`: row-vector, `v*M`, left-handed. O nosso núcleo é GLM-shaped sobre mathc: column-vector, `M*v`. Isso não é uma tradução de nomes, é uma transposta — e é onde um porte destes se estraga em silêncio, porque o resultado continua a compilar e a parecer plausível.

Verificados os dois sítios onde a convenção entra:

- **`FormationBehavior`** — eles preenchem a matriz por linhas (`m1._11.._13 = rt`, `_21.._23 = up`, `_31.._33 = lk`); nós usamos `Math::mat3(rt, up, lk)`, que são as mesmas três bases por colunas. Está certo. (O original, já agora, escreve `lk` também na quarta linha, a da translação, onde devia estar `0,0,0` — lixo que a conversão para quaternião ignora.) Nós acrescentámos o guard `lkLen > 0.1f` que eles não têm: sem ele o quaternião é reconstruído a partir de ruído de vírgula flutuante e um agente parado oscila entre dois headings.
- **`PathfindBehavior`/avoidance** — o original roda o desired-move com `D3DXMatrixRotationZ(atan2f(toEntity.y, toEntity.x))`, ou seja, trata o mundo como XY. Num mundo com Y para cima isso roda no plano errado. A nossa versão (`AgentAvoidance.cpp`) foi reescrita: repulsão acumulada em XZ (`away.y = 0`), pesada pela distância, com direcções opostas determinísticas para agentes coincidentes — que o original também não resolve.

Ou seja, a matemática do curso não foi traduzida literalmente, e ainda bem: nos dois sítios onde a convenção importa, o que está no `kinetix-ai` está certo para a convenção que o `kinetix-core` usa.

## Testbed: ver a física, e verificá-la

`testbed/`, fora das três libs e `OFF` por omissão (`KINETIX_BUILD_TESTBED`), por isso quem só quer as libs nunca compila SDL. Liga-se a elas, nunca o contrário — a regra de isolamento mantém-se.

SDL2 + OpenGL imediato + Dear ImGui 1.92.5 (vendorizado em `testbed/vendor/imgui`, com os backends `sdl2` e `opengl2`). Não foi preciso escrever nenhum debug draw novo: o 2D já expõe a interface `kx::DebugDraw` que `kx::Draw(world, draw, flags)` alimenta, e o 3D já tem `DebugDraw3D` no núcleo, que `PhysicsWorld::debugDraw()` enche de linhas para quem quiser desenhá-las. O testbed implementa a primeira e lê a segunda; não percorre corpos por sua conta.

**Uma cena não é só uma coisa para olhar.** Cada uma carrega afirmações verificadas a cada passo, e uma afirmação que falhe uma vez fica marcada — não se safa por recuperar a seguir. Com janela vê-se a simulação e o estado de cada afirmação ao lado; com `--headless` corre todas as cenas por 10s e imprime PASS/FAIL, com código de saída, que é o que a torna utilizável a partir de um script de build.

```sh
cmake -S . -B build -DKINETIX_BUILD_TESTBED=ON
cmake --build build
./build/testbed/kinetix-testbed              # janela
./build/testbed/kinetix-testbed --headless   # verdictos, exit != 0 se falhar
```

Onze cenas, todas a passar: caixa em repouso, pilha de 6, círculo em queda, caixa em rampa e tunnelling (2D); caixa em repouso, pilha de 5 e esferas (3D); A\* incremental, multidão para um objectivo e grelha de proximidade (IA).

As três cenas de IA cobrem exactamente o que esta sessão mexeu, e não por acaso: cada uma existe porque houve ali um bug.

- **A\* incremental** corre a mesma busca com orçamentos de 1, 7, 64 e 1000 nós e exige que todos dêem os mesmos cantos e expandam os mesmos nós. Espalhar uma busca por frames não pode mudar o que ela encontra, só quando acaba.
- **Multidão para um objectivo** põe dezasseis agentes em coordenadas inteiras — ou seja, exactamente em cima de vértices partilhados, que é a posição que os pinava. Verifica que todos saem do sítio, que nenhum sai da superfície e que todos chegam.
- **Grelha de proximidade** faz, a cada passo, a mesma pergunta à grelha e a um varrimento exaustivo, e compara os conjuntos. A grelha é um índice, não uma aproximação.

**A suíte foi verificada a falhar.** Reintroduzi os dois bugs do NavMesh, um de cada vez, para confirmar que não passam despercebidos: com o nudge do vértice desligado, a cena da multidão falha com `0 of 16 moved`; com o guard de progresso do funnel removido, o processo pendura. Esse segundo caso mostrou uma lacuna — um `step()` que nunca retorna era indistinguível de uma máquina lenta — resolvida com um alarme por cena no modo headless, que imprime `HUNG in scene: <nome>` e sai com código 2. Verificado a disparar.

### O que escrever os testes revelou

**A física 2D é feita para pixels, não para metros.** `kLinearSlop` é `0.5` (`2d/include/kx/common.h`), e daí `kPolygonRadius = 1.0`: cada forma carrega uma pele de 1 unidade. Num mundo de metros uma caixa de 1m assenta a ~2.5 do chão e parece partida; em pixels, uma caixa de 32px assenta com 1.5px de folga (4.7% do seu tamanho) e uma pilha fica com espaçamento 33.5 para caixas de 32. O valor não veio do porte — o Kinetix2D tem o mesmo `kLinearSlop = 0.5f` em `engine/include/k2d/Geometry2D.h`, e faz sentido num motor que trabalha em pixels. É uma escala, não um defeito, mas é preciso saber-se: **quem usar `kinetix-2d` em unidades de metro vai achar que a biblioteca está avariada.** O `kinetix-3d` não tem esse problema — uma caixa de 1m assenta em y=0.5.

**A penetração transitória é normal, atravessar não é.** Uma caixa largada de 300px afunda até 10px dentro do chão antes de ser empurrada de volta — é a correcção posicional a resolver a sobreposição ao longo de alguns passos, não um bug. Por isso a verificação da cena é "nunca atravessa o chão" e não "nunca penetra". A 4000px/s, porém, um corpo normal passa mesmo para o outro lado; com `SetBullet(true)` é varrido e pára. Medido: queda de 2000px sem bullet chega a y=-9.3 (do outro lado), com bullet pára em 0.5. A cena `2D: tunnelling` mostra os dois lado a lado.

**A pilha 2D não adormece, mas pára.** Seis caixas empilhadas nunca acendem todas o flag de sono ao mesmo tempo; ao fim de 8s, no entanto, a mais rápida move-se a menos de 0.5px/s. A verificação pergunta pelo movimento, que é o que interessa, e não pelo flag.

Nada disto apareceu nos testes de unidade que já existiam, porque nenhum punha corpos a cair.

## O que estava a custar caro numa multidão

Três coisas, encontradas por medição e não por leitura. A primeira foi a que eu esperava; as outras duas contrariaram o palpite.

**Manter um agente no chão custava o tamanho do nível.** `moveAlongSurface()` começava por procurar o triângulo de partida, e essa procura varre a malha inteira. Um agente deslizado todos os frames anda uma fracção de triângulo e já sabe onde estava — não tinha era como o dizer. Nova sobrecarga `moveAlongSurface(from, to, out, tri, extents)`: passa-se o triângulo onde a chamada anterior acabou, e é actualizado para onde esta acaba. A antiga fica, e agora chama a nova com `tri = -1`.

O ganho não é marginal, porque elimina uma dependência do tamanho da malha que nunca devia lá ter estado:

| triângulos | sem hint | com hint |
|---|---|---|
| 800 | 0.015 ms | 0.0001 ms |
| 7.200 | 0.113 ms | 0.0001 ms |
| 28.800 | 0.439 ms | 0.0003 ms |

Com 40 agentes numa malha de 28.800 triângulos: **17.55 → 0.01 ms por frame**. Verificada a equivalência em 400 passos consecutivos, com e sem hint, resultado idêntico. No `NavMeshBehavior` isso levou a média de 6.30 para 0.062 ms/frame.

**O gargalo seguinte não era o avoidance.** A repulsão agente-a-agente varria todos os grupos, o que é O(n²) e parecia o suspeito óbvio; medido o antes/depois com uma grelha espacial, o ganho foi **zero**. Desligar o avoidance por completo também não mudou nada. O tempo estava no `Entity::iterate()` — no *sensing*, que fazia o mesmo varrimento e corre sempre, mesmo sem behavior nenhum.

`ProximityGrid` (`ai/include/ProximityGrid.h`) é uma grelha uniforme em XZ, com tabela de células aberta (só as ocupadas existem) e listas ligadas por índices sobre um array plano — um rebuild são duas passagens lineares e nenhuma alocação depois da primeira. O `World` reconstrói-a no topo de cada `iterate()`, antes de qualquer behavior perguntar. Testada contra a busca exaustiva em cinco raios e casos de fronteira (raio zero, longe de tudo, grelha vazia): mesmos conjuntos, sempre.

**Duas correcções de que a primeira versão precisou.** Ligar a grelha ao sensing *piorou* o caso denso (2.25 → 2.95 ms), e valeu a pena perceber porquê em vez de aceitar a média:

- As duas passagens de sensing faziam **duas consultas idênticas** — mesmo centro, mesmo raio. Agora é uma só, e ambas as listas saem dela.
- `insertSortedDist()` inseria a meio de um vector, O(n) por vizinho. Com 130 vizinhos são ~8500 deslocações por agente, para uma ordenação que nada lê até a passagem acabar. Passou a acumular e ordenar uma vez (`ct::sort`).

O resultado ganha em todos os regimes, e a escala deixou de ser quadrática:

| 320 agentes | antes | depois |
|---|---|---|
| amontoados (espaçamento 0.5) | 2.25 ms | 1.54 ms |
| espaçamento 2.0 | 0.53 ms | 0.077 ms |
| espalhados (espaçamento 5.0) | 0.50 ms | 0.030 ms |

Com espaçamento 3.0: 320 agentes 0.051 ms, 640 agentes 0.112 ms, 1280 agentes 0.219 ms — 4× os agentes, 4.3× o custo. O que se paga passou a ser a densidade local, não o total.

### Recast/Detour como referência

`/media/projectos/projects/cpp/Radion/vendor/recastnavigation` — Recast (10.3k linhas), Detour (9.2k), DetourCrowd (4.2k). **Licença zlib**, não GPL: ao contrário do CWR, aqui reutilizar código seria legal desde que o aviso de copyright fosse mantido. Mesmo assim a `ProximityGrid` foi escrita de raiz, porque a estrutura é elementar e os nossos tipos são outros — o que veio do `dtProximityGrid` foi a ideia de que a pergunta certa é "quem está perto", não "quem existe".

Do que lá está e ainda não temos: o resto do DetourCrowd — corredores de caminho, fronteira local, evitamento por amostragem de velocidade. Fica para quando fizer falta. A erosão pelo raio do agente (`erodeWalkableArea` do lado do Recast) já tem equivalente próprio — ver "Erosão pelo raio do agente" mais abaixo.

## Demos visuais, e como acrescentar mais

O testbed passou a registar-se sozinho. Uma cena declara-se no ficheiro onde vive:

```cpp
REGISTER_SCENE(Vehicle2D, "2D demo: vehicle", Category::Physics2D)
```

e aparece na lista, na UI e no `--headless`. Não há lista central para manter em dia — antes cada ficheiro novo obrigava a declarar, somar contagens e encadear índices no `main.cpp`, em três sítios. O registo é feito antes do `main()` num vector local a uma função (para não depender da ordem de inicialização entre ficheiros), e o nome é passado ao macro em vez de lido de uma instância temporária, porque construir uma cena constrói o mundo dela — perguntar o nome não deve correr física.

Dezasseis cenas: cinco de física 2D, três de 3D, três de IA, mais cinco demos (corda, veículo, pirâmide, corrente 3D, demolição). Na janela há agora agrupamento por categoria e interruptores para desenhar AABBs e contactos por cima das formas, que é o que o `kx::Draw` já sabia fazer e ninguém pedia.

### O que as demos revelaram

Nenhuma destas foi afinação de cenário. Cada uma é um número que a biblioteca impõe e que não estava escrito em lado nenhum.

**O motor de roda 2D precisa de torques na ordem de 1e8.** A demo do veículo não andava, e passei por três hipóteses erradas — atrito, velocidade do motor, densidade das rodas — antes de medir a inércia: uma roda de 22px a densidade 1.5 tem I ≈ 5.5e5, e o impulso para a acelerar num passo pede um orçamento de torque perto de 7e8. Com 9e4 (o valor que parecia razoável) o carro era arrastado pelo atrito, não pelo motor, e o comportamento era binário — ou não andava, ou capotava. Cheguei a suspeitar de um bug no warm-start do `WheelJoint`; comparado com o `b2_wheel_joint.cpp` do Box2D, está idêntico. Não havia bug nenhum, era escala.

**E velocidades acima de ~10 rad/s viram o carro.** Com o torque certo, o que passa a limitar é a velocidade: acima disso as rodas rodam o chassis por cima de si próprio, mesmo em chão perfeitamente plano. `Body::Angle()` acumula em vez de envolver, por isso o "tilt" que a cena mede é "alguma vez deu uma volta", não "quão inclinado está agora".

**O carro caía da borda do mundo.** Antes de perceber isso, andei a suavizar o relevo do terreno a achar que eram os declives. O terreno tinha 1600px e o carro andava 3300 — capotava sempre no mesmo instante porque chegava ao fim. É o género de coisa que só se vê quando se olha para *quando* falha, não só *se* falha.

**`kinetix-3d` não tem colisão contínua.** A demolição atirava uma esfera a 18 m/s contra uma parede de tijolos; a esfera atravessava a parede inteira sem parar e empurrava um tijolo para -670 m. Não é bug do solver de contactos: a 18 m/s a esfera percorre 0.3 m por passo e não há varrimento que a trave. O `kinetix-2d` tem `SetBullet(true)` para isto (ver a cena `2D: tunnelling`); **o 3D não tem equivalente**. A demo passou para 10 m/s, que é o limite abaixo do qual nada é ejectado. Fica registado como limitação, não como afinação.

**Uma verificação avaliada cedo demais nunca mais passa.** Como um check que falha uma vez fica marcado, julgar "a bola derrubou a parede" a partir dos 4s falhava por a bola ainda não lá ter chegado, e o resultado a 14 de 48 tijolos derrubados já não limpava a marca. O critério não estava errado, o instante estava.

## Três testbeds, um por biblioteca

`testbed/` passou de um executável para três — `kinetix-testbed-2d`, `-3d`, `-ai` — sobre uma `testbed-core` estática com a janela, a câmara, o registo de cenas e o modo headless.

A razão não é arrumação. **Cada binário linka só a sua biblioteca**, e isso torna o isolamento verificável em vez de afirmado:

```
kinetix-testbed-2d:  kx=269  Physics=0    AI=0
kinetix-testbed-3d:  kx=0    Physics=318  AI=0
kinetix-testbed-ai:  kx=0    Physics=0    AI=197
```

(`nm -C` sobre cada binário, contando símbolos por namespace.) Um executável único a linkar as três nunca poderia mostrar isto. Obrigou a mover o `GLDebugDraw2D` do `core/` para `2d/`: nomeia tipos `kx`, e no core faria os outros dois linkarem `kinetix-2d` para algo que nunca chamam. Pelo mesmo motivo a barra de overlays passou a ser genérica — a cena declara os nomes (`overlayNames()`), a shell mostra as caixas e põe os bits, e é a cena que os traduz para as flags da sua própria biblioteca.

Acrescentar uma demo é escrever o ficheiro na pasta certa e nomeá-lo no `CMakeLists.txt` dessa pasta; o `REGISTER_SCENE` faz o resto.

Dezoito cenas: oito de 2D, sete de 3D, três de IA.

## O erro que estava em todas as cenas 3D

Escrever a demo do veículo raycast destapou algo que não era da demo. O chassis atravessava o chão e caía para -500 m, e a bissecção deu isto: **uma caixa dinâmica acima de ~40 kg atravessa um chão estático.** Abaixo assenta; acima, o solver diverge. Reproduzido no HEAD, portanto anterior a esta sessão.

Não é bug da biblioteca — é a API a ser usada mal, e eu tinha-a usado mal em todas as cenas 3D que escrevi. `setMass()` define a massa e **não toca no tensor de inércia**, que fica no valor por omissão de 1 em cada eixo. Um corpo de 800 kg que resiste a rodar como um de 1 kg é fisicamente incoerente, e o solver de contactos acaba por o empurrar através do chão. O `RigidBody.h` já documenta a solução — a struct `Inertia` (`box`, `solidSphere`, `cylinderY`, `capsuleY`, `convexHull`) — e o comentário até explica porque é que ela vive lá e não nas shapes. Faltava eu chamá-la:

```cpp
body->setMass(mass);
body->setInertiaTensor(Inertia::box(mass, halfExtents));
```

Com isso, 50/200/800 kg assentam todos em y=0.32.

**Corrigir isto fez a pilha 3D falhar**, o que é a parte interessante. As caixas de 1kg tinham inércia 1.0 quando a correcta é 0.167 — resistiam a rodar seis vezes mais do que deviam, e era isso que segurava a pilha. Com a inércia certa, varri o atrito:

| atrito | 0.40 | 0.45 | 0.50 | **0.55** | **0.60** | 0.65 | 0.70 | 0.80 |
|---|---|---|---|---|---|---|---|---|
| pilha | cai | cai | cai | **fica** | **fica** | cai | cai | cai |

Só a banda 0.55-0.60 aguenta. **Mais atrito a piorar uma pilha não é física** — é o solver de contactos a ser instável fora dessa janela, e fica registado como limitação conhecida do `kinetix-3d`, não como um número a afinar. A cena usa 0.55 e diz porquê na descrição.

## Outras demos, e o que impuseram

**Veículo raycast.** As rodas são raios lançados do chassis, não corpos - a suspensão é o comprimento do raio e a aderência vem de um modelo de atrito, por isso não há colisões de roda para falhar. `setEngineForce()` tem de ser chamado todos os passos, não uma vez. Aqui voltei a tropeçar no mesmo erro da demo do carro 2D: o carro percorria mais de 100 m e o chão tinha 60 m de raio, por isso caía pela borda. O chão passou a 400 m.

**Partículas.** Pontos varridos com tempo de vida e restituição próprios, não corpos rígidos - milhares custam o que uma mão-cheia de corpos custaria. A verificação é que a população estabiliza: emissão constante e vida de 3 s têm de dar um patamar, e uma contagem que só sobe significa que nada é reciclado.

Ambas as demos falharam primeiro por eu avaliar cedo demais - como um check que falha uma vez fica marcado, julgar antes de a situação existir grava uma falha que o resto da corrida já não limpa. É a terceira vez que isto acontece nesta sessão; o padrão é sempre o mesmo e o critério nunca estava errado, só o instante.

## O render do testbed, e porque o primeiro não servia

A primeira versão desenhava só o que `DebugDraw3D` emite - e o que as shapes emitem são arestas: doze linhas por caixa e mais nada. Chega para saber que um corpo existe e não chega para ver o que faz. Uma esfera a rolar era indistinguível de uma a deslizar, uma pilha a cair era um emaranhado, e as partículas **não apareciam de todo**.

**Partículas.** O `ParticleSystem` não emitia nada para o debug draw e as suas posições eram privadas: só `count()` era público. Um sistema de partículas que ninguém consegue desenhar não serve para nada, por isso o `ParticleSystem` ganhou `particle(i)` a devolver posição/raio/tag das partículas vivas. Válido até ao `step()` seguinte - uma partícula que morre é removida por troca, portanto o índice não nomeia a mesma partícula entre passos.

**Geometria sólida.** O testbed passou a desenhar as formas ele próprio, a partir das transformações dos corpos, com faces, iluminação direcional e buffer de profundidade (`testbed/3d/render3d.cpp`). O wireframe do `debugDraw()` continua disponível, agora como overlay ligável.

**Z-fighting no chão.** A grelha e o plano à mesma altura disputavam pixels, o que dava riscas horizontais por todo o chão. Levantar as linhas alguns milímetros em world space "resolve" a uma distância e volta a falhar em todas as outras; a correcção é `glPolygonOffset` a empurrar o plano em profundidade, que é invariante à distância.

**A câmara nunca tinha funcionado.** `beginFrame3D(scene->camera3D(), ...)` lia a câmara da cena a cada frame, aplicava o arrasto do rato a essa cópia, e descartava-a - o estado voltava sempre ao inicial. A câmara passou a viver na shell: a cena dá a vista inicial e continua a mandar no alvo (para uma câmara que persegue um carro), o utilizador manda na órbita e no zoom. Com limite no pitch, porque na vertical exacta a base do look-at é indefinida e a vista inverte.

**Agentes de IA, no estilo do OpenSteer.** Cruzes de três linhas não mostram direcção, e a cena da multidão desenhava por cima a navmesh *e* uma grelha de referência desalinhada - dois planos sobrepostos. Agora a navmesh é sólida com as arestas visíveis, e cada agente é um triângulo apontado ao longo da sua velocidade dentro do círculo do seu raio, que é a convenção que o `drawBasic2dCircularVehicle` do OpenSteer usa. Mais uma seta de velocidade com comprimento proporcional à rapidez, e cor por progresso, para um agente encalhado se destacar sem ser preciso ler números.

## Referências em `tmp/`

`tmp/` está no `.gitignore` de propósito: é material para estudar, não código a versionar.

- **`tmp/OpenSteer`** - MIT, do Craig Reynolds (o autor dos boids). É de onde o nosso `Steering.h` descende: seek, flee, wander, pursuit, evasion, separation, alignment, cohesion e avoidObstacles já lá estão dos dois lados. Por explorar: `Pathway`/`SegmentedPathway` (seguir um corredor com raio em vez de uma linha de cantos) e a `lq` (localização espacial, que resolve o mesmo problema que a nossa `ProximityGrid`).
- **`tmp/recastnavigation`** - zlib. Já comparado noutra secção; o que continua por ter é o resto do DetourCrowd (a erosão pelo raio do agente já tem equivalente próprio, sobre a adjacência de triângulos em vez de um voxel grid).

Ambas as licenças permitem reutilizar código directamente, ao contrário do CWR. Mesmo assim vale reimplementar quando a estrutura é simples e os nossos tipos são outros - a `ProximityGrid` foi esse caso.

## Capturas sem ninguém a olhar

`--shot=<ficheiro>` renderiza noventa frames e escreve um PPM do framebuffer; `--scene=<n>` escolhe a cena. Sem isto, "o render está mau" não é verificável a partir de um terminal - e foi assim que os problemas acima foram vistos, um a um, em vez de discutidos.

## Multidão a zumbir à volta do objetivo

Agentes que chegavam ao destino não paravam: ficavam a vibrar em cima dele, a maioria colada exactamente a `dist == goalRadius`. Medido, depois de terem chegado, cada um dos doze continuava a percorrer ~3.6 m.

Um só limiar era a causa. Ao entrar no raio o agente travava e o `applyAvoidance()` continuava a correr - que é o que se quer, para um monte de agentes no mesmo ponto se espalhar - mas isso empurrava-o um pouco para fora do raio, onde deixava de estar "chegado" e voltava a perseguir o objectivo. Entrar, ser empurrado, voltar, para sempre.

Duas correcções, ambas em `NavMeshBehavior::iterate()`:

- **Histerese.** `arrived` fica pegajoso: entra-se com `goalRadius` e só se sai com o dobro. O avoidance continua a espalhar a multidão sem reacender a perseguição.
- **Travagem proporcional.** `setDesiredMove(-velocity)` era pedido todos os frames mesmo com o agente praticamente parado; agora só abaixo de uma velocidade residual é que se deixa de travar.

Resultado, mesma cena: **3.6 m → 0.2 m** de movimento residual por agente, e as distâncias ao objectivo passam a espalhar-se entre 0.24 e 2.49 em vez de se acumularem no raio. Um aglomerado, não um anel a vibrar.

## Cenas de navmesh que não se mexiam

As quatro cenas novas desenhavam o caminho e mais nada - `step()` só incrementava o relógio. Uma linha desenhada prova que a busca devolveu alguma coisa e não prova que ela se pode percorrer, por isso cada uma passou a ter um agente que segue os cantos à velocidade real e volta ao início quando chega.

## Rampas ao contrário

A demo do veículo tinha as rampas com o lado alto virado ao carro. `angleAxis(+0.18, X)` leva +z para +y pela regra da mão direita: a aresta que o carro encontra primeiro sobe para +0.97 e a de saída desce para -0.28 - uma parede, não uma rampa. O sinal certo é negativo.

Corrigido o sinal, ficou o segundo erro: dois cantos da laje enterrados e a aresta de entrada a +0.068, ou seja um degrau. A altura do centro não é arbitrária - **o carro anda na face de cima**, portanto é o canto superior próximo que tem de ficar ao nível do chão:

```
centreY = halfLength * sin(a) - halfHeight * cos(a)
```

(Cheguei lá depois de escrever a fórmula para o canto inferior primeiro, que põe a entrada a meio metro de altura.) Verificado numericamente antes de aceitar: entrada a 0.000, saída a 1.275.

## O funnel cortava através de geometria que não existe

A cena das duas salas mostrou um caminho a atravessar o vazio entre elas: três cantos apenas, e o segmento do meio a passar por (17.2, 21.2), onde não há chão nenhum. O A* estava certo - a sequência de triângulos passava toda pelo corredor - e era o funnel que a deitava fora e devolvia praticamente a linha recta.

Duas hipóteses minhas antes de encontrar a causa, ambas erradas e ambas desfeitas por medição:

1. **Orientação dos portais.** Instrumentei e contei 58 de 59 "invertidos", corrigi, e o caminho passou a oscilar entre dois pontos. O teste estava mal: media a orientação contra o portal anterior, que é circular. Verificado à mão, o portal 1 estava correcto desde o início. Revertido.
2. **Portal degenerado no índice 0.** É o apex inicial (`startPos` dos dois lados), que o algoritmo exige. Não era isso.

A causa apareceu ao comparar com o `findStraightPath` do Detour (`tmp/recastnavigation`), duas diferenças no reinício do funil:

- **`i = apexIndex + 1` em vez de `i = apexIndex`.** O Detour repõe `i` no apex e deixa o `for` incrementar; nós saltávamos um portal a cada reinício. O portal onde o apex acabou de aterrar ainda tem de ser testado contra o funil novo.
- **Os lados não voltavam ao apex.** O Detour faz `portalLeft = portalRight = portalApex`; nós mantínhamos os antigos, o que fecha o funil contra si próprio e ele deixa de emitir cantos.

Com as duas corrigidas, o mesmo percurso passa de 3 cantos (56 m em linha recta pelo ar) a 6 cantos que entram no corredor em (18,20), percorrem-no até (18,28) e só depois atravessam a segunda sala.

Isto explica também porque é que os caminhos em malha plana pareciam sempre certos: sem obstáculos, a linha recta *é* a resposta, e um funnel que colapsa para ela acerta por acidente. Foi preciso geometria com um corredor obrigatório para o expor - que é exactamente o que as cenas novas de navmesh existem para fazer.

## Ginásios de joints, steering com FSM, e três benchmarks

Faltavam quatro coisas pedidas mais do que uma vez: joints 2D e 3D com uma demo própria em vez de aparecerem só embutidos num veículo ou numa corrente, uma cena de steering que não fosse sobre navmesh, e um número de desempenho comparável entre as três bibliotecas. Seis cenas novas, uma por peça (steering e o pick que a acompanha saíram na mesma cena):

- **`2D demo: joint gym`** — os seis tipos (`distance`, `revolute`, `motor`, `wheel`, `gear`, `mouse`) lado a lado, cada um com o seu próprio ponto de ancoragem estático para não se influenciarem.
- **`3D demo: joint gym`** — os sete tipos (`distance`, `fixed`, `hinge`, `point`, `slider`, `universal`, `wheel`) na mesma disposição; o `mouse` vem de borla do picking que `Demo3D` já tinha.
- **`AI: steering FSM`** — uma entidade, três `Behavior` de `Steering.h` (`Wander`, `Seek`, `Flee`) e uma `StateMachine` a sério a decidir qual está ligado, sobre um alvo arrastável com o rato.
- **`2D: benchmark`**, **`3D: benchmark`**, **`AI: benchmark`** — carga crescente (pirâmide, grelha de caixas, bando com separation/alignment/cohesion), ms por passo a cada patamar.

### O hinge que só é estável dentro de um arco

A primeira versão do `hinge` no ginásio 3D era um braço a rodar continuamente em torno de um eixo vertical, motor a 2.5 rad/s. Sem carga nenhuma a não ser a sua própria inércia, e mesmo assim **explodia** - a velocidade angular subia de forma limpa durante meio segundo e depois saltava para dezenas de rad/s, com picos de milhares quando o braço completava uma volta. Tirando o motor por completo, o mesmo aconteceu ao fim de ~1.5 s: `w` passa de 63 para 278824 para `inf` em três passos.

Não é o motor - é o próprio `HingeJoint` sem motor nenhum, o que descarta imediatamente qualquer hipótese ligada a `setMotor`/torque. Trocar o eixo para o mesmo que o `Chain3D` já usa de forma estável (perpendicular à gravidade, braço a pendurar) melhorou mas não resolveu - picos ocasionais de 20+ rad/s continuavam a aparecer ao fim de alguns segundos. Todos os hinges existentes no codebase antes desta sessão (`Chain3D`, e os do próprio `HingeJoint.h` testados no Radion) só o exercitam dentro de um arco limitado; nenhum roda uma volta completa. Com `setLimits(-1, 1)` a demo passou a oscilar como um portão em vez de rodar, e a instabilidade desapareceu por completo.

Fica como limitação conhecida do `kinetix-3d`, não como afinação da cena: **`HingeJoint` com motor e sem limites, a completar voltas inteiras, diverge.** Vale a pena revisitar `calculateAxisAndAngle()`/`rotationAngleAroundAxis()` (`3d/src/dynamics/HingeJoint.cpp`) à procura de um problema de wraparound perto de ±π antes de confiar nisto para, por exemplo, uma roda de moinho ou uma hélice - qualquer coisa que precise de rodar livremente, não só oscilar.

### Os três números, no mesmo grão

As três cenas de benchmark usam o mesmo relógio (`clock_gettime(CLOCK_MONOTONIC, ...)`, não `SDL_GetTicks` - as cenas não linkam SDL e não deviam passar a linkar só para medir tempo) e o mesmo formato de saída, para serem comparáveis entre si na mesma máquina, não entre máquinas nem contra nenhuma outra biblioteca:

```
2D: benchmark    10 → 300 corpos   0.057 ms → 2.262 ms/step
3D: benchmark    10 → 122 corpos   0.206 ms → 2.869 ms/step
AI: benchmark    20 → 500 agentes  0.011 ms → 0.303 ms/iterate
```

Nenhuma das três é a mesma carga - a 2D é uma pirâmide (contactos entre si), a 3D uma grelha de caixas empilhadas, a IA um bando sob `separation`/`alignment`/`cohesion` (que é o que realmente percorre a `ProximityGrid` a cada frame). Comparar os números entre si diz mais sobre o que cada carga pede do que sobre qual biblioteca é "mais rápida" - a IA escala melhor porque a grelha de proximidade mantém o custo por agente quase constante à medida que a área cresce com a contagem, exactamente o que [[o que estava a custar caro numa multidão]] já tinha mostrado.

### Uma shell que engolia os seus próprios números

O `--headless` só imprimia o `detail` de um check quando ele **falhava** - `runHeadless()` em `testbed/core/main.cpp` fazia `if (!c.everFailed) continue;` antes de imprimir. Para as cenas de verificação isso é o comportamento certo (silêncio quando está tudo bem), mas para um benchmark o `detail` *é* o resultado, e a única forma de o ver era abrir a janela e ler a UI à mão. Passou a imprimir sempre que há `detail`, falhe ou não o check - o resto das cenas não notou a diferença porque nenhuma delas põe `detail` num check que passa.

### Um `Group` a apanhar um ponteiro morto

`BenchmarkAI::rebuild()` fazia `delete mGroup` directamente para trocar de fase. `World` guarda um ponteiro cru para cada grupo adicionado (`World::add`), e apagar o grupo sem primeiro `World::remove(*mGroup)` deixa esse ponteiro pendurado - o `World::iterate()` da fase seguinte segfaultava a tentar percorrê-lo. `Group.h` documenta a posse (*"the world OWNS the groups added via add()"*) mas não avisa que o inverso também é preciso: quem apaga um grupo tem de o tirar do `World` primeiro.

## Erosão pelo raio do agente

Era a limitação conhecida do `NavMesh` desde a extracção (linhas 81 e 319 mais acima): um caminho passa rente a paredes e a buracos porque o funnel dá o mais curto *dentro* dos triângulos caminháveis, e sem encolher essa superfície primeiro esse mais-curto corta literalmente em cima da geometria - um agente com corpo, a segui-lo, roça na parede.

O Recast resolve isto com `rcErodeWalkableArea` (`RecastArea.cpp:75-287`) sobre um heightfield voxelizado: chamfer distance transform em duas passagens sobre a grelha (peso 2 por passo cardinal, 3 por diagonal), depois `area = RC_NULL_AREA` em toda a célula cuja distância à fronteira for menor que `raio * 2` voxels. Não há grid nenhum no `NavMesh` do Kinetix para portar isso para cima - `build()` nunca voxeliza, um triângulo caminhável *é* o polígono, sem etapa intermédia - por isso a erosão teve de ser reescrita sobre a própria adjacência de triângulos em vez de adaptada.

A peça que já existia e tornou isto barato foi perceber que já havia duas metades da solução escritas, só que para outro problema:

- **`NavTri::active`** - o campo que `pruneUnreachable()` já usa para desligar triângulos inalcançáveis, e que as nove leituras espalhadas por `NavMesh.cpp` (`findContainingTri`, `moveAlongSurface`, `raycast`, `distanceToWall`, ...) já tratam como "isto não conta". Marcar um triângulo erodido como `active = false` significa que toda a travessia já sabe ignorá-lo - zero código novo do lado do consumo.
- **`distanceToWall()`** - já fazia exactamente a medição que a erosão precisa, só que a partir de um ponto (BFS por adjacência, cauda com distância real projectada na aresta de fronteira mais próxima, poda por `centroide > melhor + raio`).

`erodeByAgentRadius(agentRadius)` (`NavMesh.cpp`, chamada no fim de `build()`) generaliza a segunda ideia para todas as arestas de fronteira ao mesmo tempo em vez de uma só busca: semeia cada triângulo de fronteira com a sua distância real medida (mesma projecção ponto-aresta do `distanceToWall`), propaga por salto centro-a-centro entre triângulos vizinhos até nada melhorar (Bellman-Ford sobre fila, não Dijkstra - não há heap na árvore, a mesma limitação já registada do `GridPathfinder`), e desliga `active` em tudo cujo resultado fique abaixo do raio. `pruneUnreachable()` corre primeiro e sobrescreve `active` por inteiro, por isso a erosão tinha de vir depois - e só faz AND lógico, nunca atribuição directa, exactamente para não ser ela a apagar o trabalho do prune.

O salto centro-a-centro é 3D, não só XZ: numa rampa ou solda de degrau isso sobrestima ligeiramente a distância horizontal, o que só torna a erosão mais conservadora - nunca deixa activo um triângulo genuinamente mais perto da parede que o raio.

Testado com uma sala 30×30 e um buraco quadrado central [10,20]×[10,20] (a mesma geometria de `AI: hole in the floor`), `agentRadius = 1.5`: antes da erosão o caminho passa a 0.00 m do buraco (rasa a aresta); depois, a 2.00 m do buraco e 3.00 m da parede exterior - ambos acima do raio, como esperado. Um raio extremo (20, maior que a sala inteira) apaga a malha toda e `findPath` devolve `false` de forma limpa, sem travar nem estourar - confirmado antes de reverter o valor de teste. Fica registado em `AI: agent clearance`, categoria AI.

**Substituído depois (revisão de bugs):** a erosão por triângulo inteiro dependia da triangulação - um corredor de 2 m em quads de 1 m com raio 0.5 ficava partido, e uma sala de 10 m feita de 2 triângulos não erodia nada. `erodeByAgentRadius` deu lugar a `computePortalSpans(agentRadius, agentMaxClimb)`, ao estilo do Detour: todos os triângulos ficam, e cada portal é cortado ao maior troço a pelo menos `agentRadius` de todas as paredes (paredes encontradas por uma grelha XZ; paredes acima de um degrau escalável são ignoradas). Portais sem troço livre fecham dos dois lados; o A* (`findPath` e `NavMeshQuery`) salta-os e o funnel usa os portais cortados. `keepClearOfWalls` afasta o objectivo de uma parede mais perto que o raio. Consequência: `agentRadius` passou a ser só de encaminhamento - `nearestPoint`, `moveAlongSurface` e `raycast` veem todos os triângulos. `findPath(..., bool* outPartial)` e `NavMeshQuery::partial()` dizem quando o caminho devolvido é parcial.

`TwoRooms`/`Courtyard`, as duas cenas mais antigas com o mesmo defeito, ficaram sem `agentRadius` na primeira versão da erosão - a funcionalidade existia mas não tinha sido propagada às cenas que a motivaram. Corrigido: `TwoRooms` usa `0.4` (o corredor tem 2m de largura, metade disso é o tecto antes de o fechar por completo) e `Courtyard` usa `0.5`.

## Chegada em rampa, não travão seco - e o heading que faltava desenhar

Reportado directamente ao correr `AI: crowd to a goal`: os dezasseis agentes chegavam ao objectivo e ficavam "a rodar como malucos, uns em cima dos outros" em vez de assentar. Duas causas, ambas em código antigo, nenhuma delas no que esta sessão tinha acabado de escrever:

**Travão seco sem rampa antes dele.** `NavMeshBehavior::iterate()` já tinha histerese de chegada (`arrived`/`releaseRadius`, evita que um agente empurrado para fora do `goalRadius` por um segundo reacenda a perseguição), mas nada reduzia a velocidade *antes* de cruzar esse raio - o agente vinha a `turnRate` inteiro e só travava (`setDesiredMove(-velocity)`) no instante exacto em que entrava. Com dezasseis agentes a convergir no mesmo ponto ao mesmo tempo, todos cruzavam essa linha ainda em movimento, travavam, o avoidance entre eles empurrava-os de volta para fora, e recomeçavam - o "anel a zumbir" que foi reportado. Adicionado `NavMeshBehavior::Settings::arrivalRadius`: entre ele e `goalRadius`, `turnRate` decai linearmente até zero, o mesmo princípio do `steerForArrival` que o `tmp/OpenSteer` de referência **não tem** (confirmado por leitura directa - o `steerForSeek` de lá é seek puro, sem rampa; o amortecimento que existe é `adjustRawSteeringForce`, que limita o ângulo da força a baixa velocidade, não a distância ao alvo). Sem precedente para copiar, escrito de raiz para este único consumidor.

**Dezasseis agentes com um só ponto de chegada.** Mesmo com a rampa, um `goalRadius` de 1.5 m para agentes de 0.55 m de raio convergindo todos no mesmo ponto não tem espaço físico para mais de um par ficar lá dentro ao mesmo tempo - os restantes ficavam permanentemente a ser empurrados pelo avoidance. `Crowd` (`testbed/ai/scenes_ai.cpp`) passou a dar a cada agente um slot próprio (mesmo padrão da grelha de arranque, deslocado à volta do objectivo partilhado) em vez de um alvo único; `goalRadius` subiu para 3.0 m e `arrivalRadius` para 8.0 m.

**O heading nunca existiu, só a velocidade.** `Entity::alignWithVelocity()` (que gira `orientation()` para acompanhar `velocity()`) está escrito e documentado mas **nunca é chamado** em código nenhum de `ai/` - confirmado por busca na árvore inteira. `NavMeshBehavior` só move posição, nunca orientação; o próprio `Entity.h` já documenta que isto é deliberado (linhas 210-217, sobre a convenção D3DX-era que não bate certo com `GameObject::forward()` do motor) e que quem precisa de uma direcção visível tem de a tirar de `velocity()` directamente. É exactamente o que `Crowd::draw()` já fazia - mas sem suavização nenhuma, e a velocidade muda de direcção bruscamente quando o travão e o avoidance se alternam frame a frame perto do objectivo, o que lido do lado de fora é indistinguível de "o triângulo do agente roda ao acaso". Corrigido só do lado do desenho, não da simulação: um heading próprio por agente (`ct::Vector<M::vec3> mHeading` na cena, não no `Entity`), actualizado por *blend* exponencial contra `velocity()` normalizada (~90% de captura em 0.15s) e só desenhado, nunca lido de volta pela física. A mesma técnica foi aplicada ao `mWalkerHeading`/`mDrawHeading` de `NavScene` (`scenes_navmesh.cpp`) - aí com um cuidado extra: o heading que **move** o walker (`toCorner/distance`, recalculado exacto a cada canto) tinha de ficar separado do heading **desenhado** (suavizado), porque suavizar o primeiro desviava a trajectória do caminho em vez de só amaciar o que se vê.

## `mElapsed` que nunca volta a zero

No mesmo `NavScene::advanceWalker()`, o reinício ao fim do percurso estava condicionado a `mElapsed > 1.0f` - mas `mElapsed` é o relógio da cena inteira, nunca reposto. Passado o primeiro segundo de vida da cena, essa condição fica verdadeira para sempre, e assim que o walker chega ao objectivo `repath()` corre em **todo frame seguinte**, teleportando-o de volta ao início a cada frame em vez de ficar parado um instante e só depois recomeçar. Lido de fora: "o agente fica maluco perto do alvo". Trocado por `mTimeSinceArrived`, reposto em cada `repath()` e só incrementado enquanto `mArrived` é verdadeiro.

## Duas cenas sem nada para ver

`AI: incremental A*` e `AI: proximity grid` nunca tiveram bug - ambas são cenas de validação numérica (a busca incremental dá o mesmo resultado que a busca completa; a grelha de proximidade concorda com uma varredura por força bruta) cujo `draw()` desenhava um estado estático e deixava toda a prova nos `checks()` de texto. Reportado como "não faz nada"/"não percebo o que é suposto mostrar" - queixa justa: não havia nada no ecrã que mudasse enquanto o `step()` fazia o trabalho de facto interessante.

`QueryEquivalence` passou a desenhar o caminho do orçamento que acabou de correr (`mLast`, já calculado, nunca desenhado) por cima do de referência numa linha mais grossa, mais um ponto amarelo por orçamento já verificado - as duas linhas nunca se separam é agora algo visível, não só um booleano interno. `Proximity` passou a destacar o agente-sonda da vez (cor própria, anel ao raio de sensação real) e a colorir a verde só os vizinhos que a grelha devolveu naquele passo, deixando o resto da nuvem esbatido - o que estava a ser comparado contra o quê passou a estar no ecrã, não só no painel de texto.

## Aerodinâmica: asas, hélice e um avião ligeiro

`dynamics/Aerodynamics.h` deixou de ser o modelo tensorial (`AeroSurface`/`AeroControlSurface`/`Airplane`, uma `mat3` a mapear velocidade em força) e passou a ser o umbrella de um módulo aerodinâmico escrito de raiz a partir da teoria de livro: perfil fino, faixas (strip theory) com downwash de linha sustentadora, BEMT para a hélice e a ISA 1976. A API antiga não tinha nenhum utilizador na árvore e foi **substituída**, não mantida ao lado. Tinha também um bug de sinal: a velocidade relativa ao ar era `v + vento` em vez de `v - vento` — indistinguível com ar parado, o dobro da velocidade com vento de frente. Há agora um teste (`3d_aero_wing`, `windIsSubtracted`) que apanha exatamente isso: parado com 40 m/s de vento de frente tem de dar a mesma sustentação que voar a 40 m/s em ar parado, e derivar com o vento tem de dar zero.

Um header por peça em `3d/include/dynamics/`, nenhuma depende das que vêm depois:

- **`Atmosphere`** — ISA 1976 até 32 km (troposfera a 6.5 K/km, isotérmica até 20 km, inversão de 1 K/km), altitude geométrica convertida em geopotencial, densidade/pressão/temperatura/velocidade do som; ou densidade constante. Vento constante mais um `WindCallback(posição, userData)` para rajadas (a atmosfera não guarda relógio; o tempo vive no `userData`). Altitude = `y` do mundo + `originAltitude`.
- **`Airfoil`** — Cl/Cd/Cm em ±180°, contínuos. Regime colado: `Cl = a0 (α − α0)` (a0 = 2π por defeito), arredondado por um *smooth-min* quadrático só nos últimos ~2° antes de `Cl_max`, para o declive linear ficar exato; `Cd = Cd0 + k Cl²`; Cm constante no quarto de corda. Regime separado: placa plana com força normal `Cd_max sin α` (`Cl = Cd_max sin α cos α`, `Cd = Cd0 + Cd_max sin² α`) e centro de pressão a andar de c/4 a c/2 a 3c/4 com o escoamento por trás. Perda: smoothstep entre os dois ao longo de `stallWidth`. Alternativa: tabela (α, Cl, Cd, Cm) interpolada, misturada na mesma placa plana fora do seu intervalo. Flap pela teoria do perfil fino: `τ = 1 − (θf − sin θf)/π`, `ΔCm = −sin θf (1 − cos θf)/2 · δ`, com um fator empírico que reduz a eficácia acima de 10° (0.4 a partir de 60°); os limites de perda sobem só 70% do ΔCl, por isso o perfil com flap perde a um α menor. `Airfoil::propeller()` é o perfil arqueado com a perda espalhada por 20° (a pá a rodar perde mais devagar — sem isso, as estações interiores de uma hélice estática a "desperder" uma a uma faziam a tração ondular com a velocidade).
- **`Wing`** — painel reto (afilamento, torção, diedro implícito na geometria raiz/ponta, opcionalmente espelhado) cortado em faixas. Por faixa: ar relativo `vento − (v + ω×r)`, remove-se a componente ao longo da envergadura, `q = ½ρV²`, α entre o escoamento e a corda, sustentação perpendicular ao escoamento no plano da corda, arrasto ao longo dele, momento no quarto de corda. Envergadura finita: um único ângulo induzido `αi = CL/(π e AR)` para a asa toda (downwash uniforme da linha sustentadora elíptica), resolvido por Newton em cada avaliação — o ponto fixo simples diverge sempre que `a0 > π e AR`, que é qualquer deriva — e aplicado inclinando a sustentação de cada faixa para trás, por isso o arrasto induzido sai da mesma força. No regime linear o declive da asa é exatamente `a0/(1 + a0/(π e AR))`. Superfícies de comando por intervalo de envergadura, com `mix` por canal (aileron/profundor/leme/flaps). `setInducedFlow` recebe o downwash de outra superfície (o `Aircraft` liga a esteira da asa principal à cauda: `ε = 1.7 αi`, `dε/dα ≈ 0.5`).
- **`Propeller`** — BEMT por anéis com perdas de Prandtl na ponta e no cubo. A velocidade induzida axial é encontrada por *false position* (Illinois) entre `−V0/2` e um limite superior, em vez da forma clássica `a = v/V0`, que divide por zero a velocidade nula; o swirl é atualizado por substituição em três passagens (dentro de ~1% do convergido). Modo rpm ou modo binário com inércia do rotor (integração linearmente implícita no binário aerodinâmico). O avião sente a tração, a reação ao binário do veio e o binário giroscópico `H × ω`.
- **`PistonEngine`** — potência a fundo `P/Pmax = x + x² − x³` (x = rpm/rpm nominal), acelerador entre ralenti e máximo, atrito linear com a rotação, corte acima de `maxRpm`, perda com a altitude por Gagg-Ferrar.
- **`Aircraft`** — não é dono do `RigidBody`; tem as asas, motores, hélices e `DragElement`s (fuselagem como corpo rombo com CdA por eixo), uma `Atmosphere` própria ou partilhada, `ControlInputs` (aileron/profundor/leme −1..1, flaps/acelerador 0..1, mais trims), telemetria (TAS, EAS, Mach, α, β, altitude, atitude, taxas p/q/r, forças) e `trim(TrimTarget)`: Levenberg-Marquardt nos seis equilíbrios de força e momento, para voo reto nivelado (resolve α, β, três trims e acelerador) ou planeio/subida com acelerador fixo (resolve o ângulo de trajetória em vez do acelerador). `buildLightAircraft()` monta um monomotor de asa alta de 1000 kg, 11 m de envergadura, 16 m² de asa e 120 kW; `addLightAircraftGear()` dá-lhe um trem triciclo sobre um `RaycastVehicle` (rodas livres, só suspensão e aderência lateral).

**Quando chamar `applyForces`.** O `StepCallback` do `PhysicsWorld` corre no fim de cada passo, já com as poses finais; as forças que lá se acumulam são consumidas pelo `integrateForces` do passo **seguinte** — calculadas a partir do mesmo estado sobre o qual vão atuar, tal como chamar antes de `step()`. Por isso: a conduzir `step()` à mão, chamar `applyForces(dt)` antes de cada `step()`; com `update()` (que pode correr vários passos fixos), registar `Aircraft::stepCallback` e chamar `applyForces` uma vez à mão antes do primeiro `update()`. Chamar uma vez por `update()` só alimentava o primeiro passo — a integração limpa os acumuladores. O teste `stepCallbackTiming` confirma que os dois caminhos dão trajetórias idênticas ao bit.

**Números do avião por defeito** (todos medidos pelos testes): perda limpa a ~27 m/s (1.31 g de pico a 30 m/s, a 16° de α), cruzeiro trimado a 55 m/s com 1.6° de α e 78% de acelerador, velocidade máxima nivelada ~61 m/s, subida a 4.4 m/s, planeio a 11:1, margem estática ~23% da corda média. Descolagem em ar parado: 192 m e 11.8 s até sair do chão a 29 m/s, igual a 60 e 120 Hz; com 10 m/s de vento de frente, 93 m. Mão fora dos comandos depois do trim: altitude dentro de ±0.6 m em 30 s; com +6 m/s de perturbação a fugóide oscila ±55 m e amortece.

**Limites conhecidos.** Escoamento incompressível (sem Prandtl-Glauert, é para Mach < 0.3). Downwash uniforme: a carga ao longo da envergadura e o amortecimento de rolamento são os da strip theory, um pouco sobrestimados em alongamento baixo. A hélice só vê o escoamento axial (sem P-factor nem força normal do disco) e não há esteira da hélice sobre as superfícies; escoamento por trás do disco é tratado como estático, e o estado de turbilhão em moinho é mantido no limite `v = −V0/2`. A esteira da asa chega à cauda sem atraso. Sem efeito de solo, sem histerese de perda, sem momento desestabilizador da fuselagem (por isso a margem estática do avião por defeito é generosa). O motor não tem arranque nem pára.

**Testes** (`tests/3d_aero_*.cpp`): ISA a 0/5/11/15/25 km contra a tabela (<0.5%); declive 2π, ângulo de sustentação nula, pico em `Cl_max`, continuidade em todo o círculo (o maior salto entre amostras cai para metade quando o passo cai para metade — um salto não cairia); CL da asa retangular a −0.04% da linha sustentadora, escala com V² e ρ, força perpendicular ao escoamento, amortecimento de rolamento a 0.2% da strip theory, efeito diedro, vento; hélice com figura de mérito 0.69 estática, tração nula a J = 0.95 (perfil arqueado) e 0.84 (simétrico) para P/D = 0.84, reação contrária à rotação, potência = binário × ω, rpm/velocidade nulas sem NaN; e o avião acima, incluindo três minutos de acrobacia e um tombo de costas a 60 e 120 Hz.

**Testbed.** `3D demo: airplane` (`testbed/3d/demo_airplane_3d.cpp`): pista estática, o avião por defeito sobre o trem `RaycastVehicle`, 10 m/s de vento de frente e arranque a fundo depois do *run-up* (hélice já a 2500 rpm) — é o que põe a descolagem dentro dos 10 s do modo headless com margem para ver a subida. Um piloto automático simples segura o eixo da pista, roda a 20 m/s, sobe, recolhe os flaps e entra num circuito a 150 m em volta de 20°. As faixas das asas são desenhadas a partir dos próprios elementos da física e ficam vermelhas quando perdem. C reinicia. A shell não passa teclado às cenas (só F e C), por isso não há pilotagem manual.

## Hidrodinâmica: água, casco, apêndices e um barco a motor

Um módulo novo em `3d/include/marine/` (fontes em `3d/src/marine/`), em vez de mais seis headers em `dynamics/`: tem um ambiente próprio (a `WaterSurface`, o equivalente da `Atmosphere`) e **depende** do módulo aerodinâmico — `Airfoil`, `Wing`, `Propeller`, `PistonEngine` — e não o contrário; uma pasta à parte deixa essa direção visível no próprio include (`marine/Boat.h`). `marine/Marine.h` é o umbrella. Tudo escrito a partir da teoria de livro (hidrostática, Arquimedes, estabilidade metacêntrica, linha ITTC-57, ondas de Gerstner/Airy). Do lado da aerodinâmica só mudaram duas coisas, sem mudar nenhum número dos testes `3d_aero_*`: o `DragElement` passou para `dynamics/DragElement.h` (o barco usa-o para a resistência do ar nas obras mortas), e a `Wing` ganhou um `computeForces(body, densidade, FlowField, userData, escala por faixa)` — o de `Atmosphere` passou a ser um caso dele — para poder voar dentro de água.

- **`WaterSurface`** — interface com `height(x, z, t)`, `normal`, `orbitalVelocity(ponto, t)`, `sample()` (as três numa só resolução), corrente uniforme, `WaterProperties` (ITTC a 15 °C: mar 1025.9 kg/m³ e ν = 1.19e-6 m²/s, doce 999.1 e 1.14e-6), `gravity()` (a mesma para ρgh e para a dispersão — tem de ser a do `PhysicsWorld`) e um relógio próprio (`advance`), para todos os corpos verem as mesmas ondas. `FlatWater` e `GerstnerWaves`: soma de até 32 ondas trocoidais com amplitude, comprimento, direção, fase e declividade `s ∈ [0, 1]` (1 = trocoide exata, 0 = senoide de Airy), dispersão de águas profundas `ω² = g k`. A partícula em repouso em `p0` vai para `x = x0 − Σ s A d sin θ`, `y = Σ A cos θ`; a superfície dobra-se quando `Σ s k A ≥ 1`, por isso os `s` são escalados em conjunto para `Σ s k A ≤ 0.8` (as alturas mantêm-se, só se reduz o aperto horizontal). **A altura em (x, z) é a da partícula cuja posição deslocada é (x, z)**: o ponto de repouso é resolvido por ponto fixo `p0 ← (x, z) − D(p0)`, uma contração de constante `L = Σ s k A`, com erro `L^n Σ s A` no ponto e `Σ k A` vezes isso na altura (oito iterações, paragem antecipada a 0.1 mm; num mar típico o erro fica abaixo de 1e-5 da amplitude — o teste mede 2e-6 m). Velocidade orbital da teoria linear por componente, `A ω e^(−k h)` com `h` medido a partir da superfície real (Wheeler), avaliada no ponto de repouso — igual à derivada temporal da trocoide quando `s = 1`.
- **`HullMesh`** — malha triangular fechada em espaço do corpo, independente da forma de colisão. Volume, centroide e área pelo teorema da divergência (em double), `isClosed()`/`openEdgeCount()` (cada aresta usada por exatamente dois triângulos em sentidos opostos), `weldVertices()` para malhas de render, enrolamento invertido corrigido em bloco. Geradores: caixa subdividida, esfera UV, `fromConvexHull()` e `boat(BoatHullParameters)` — V de quina viva ou de bojo redondo (arco tangente de raio dado), boca a fechar até à roda de proa, quilha a subir na proa, deadrise da popa à proa.
- **`BuoyancyBody`** — por passo: vértices para o mundo (relativos ao centro de massa), altura e velocidade da água por vértice, `clipTriangleToWater` (Sutherland-Hodgman contra `profundidade > 0`, corte exato onde a profundidade linear se anula, 0/1/2 triângulos, sem divisão por zero com vértices em cima da linha de água). Em cada pedaço: **hidrostática exata para pressão linear**, `F = −ρ g A h̄ n` no centro de pressão `(Σh Σx + Σ hᵢxᵢ) / (4 Σh)` (de `∫λᵢλⱼ = A(1+δᵢⱼ)/12`) — com o centroide em vez do centro de pressão a força estaria certa mas o momento não. Volume submerso, centro de carena e área de flutuação pelo teorema da divergência sobre os mesmos pedaços, fechados pelo plano de flutuação (exato em água plana). Em ondas é Froude-Krylov com a superfície linear por troços entre os vértices do casco. **Hidrodinâmica** com a velocidade relativa `v_ponto − u_orbital − corrente`: face a entrar na água `p = ρ (c_lin |v| + ½ c_quad |v|²) cos^f θ` (força `−p A n`), face a sair uma sucção da mesma forma com coeficientes próprios, limitada pela pressão hidrostática local (ventilação — é por isso que o painel de popa corre seco); `f = 1` dá a sustentação de planeio linear no ângulo de caimento. Atrito `½ ρ C_f (1+k) |v_t| v_t` com `C_f = 0.075/(log Re − 2)²` (ITTC-57, Re = U L/ν, piso em 1e5). Slamming de von Kármán/Wagner nas faces que atravessam a superfície e cuja área molhada cresce: `F = C ρ ℓ v_n dA/dt` (C = π/4, ℓ = meia boca), limitado a `maxSlamPressure` × área. Limite de dissipação: a componente da força dissipativa ao longo da velocidade relativa (e do binário ao longo de ω) não pode mais do que parar o movimento num passo — sem isso uma tábua leve de través a 30 Hz inverte o movimento e explode (o teste mostra-o).
- **`Hydrofoil`** — uma `Wing` em água (densidade da água, velocidade da água em cada faixa, escala por faixa = fração imersa da sua envergadura, medida nas duas pontas e esbatida em ±0.1 corda). `setDeflection()` roda o perfil inteiro (leme compensado, patilhão, perna do fora de borda).
- **`MarinePropeller`** — o BEMT da `Propeller` com a densidade da água, atrás de uma caixa de inversão (relação, rendimento, frente/neutro/ré); com mudança engrenada há um só estado, a velocidade do motor, integrado linearmente implícito como no avião. Ré: carga do hélice a rodar para a frente com escoamento de sinal oposto, tração × `reverseEfficiency` (0.6). Fatores de propulsão (esteira w, dedução de tração t), imersão = fração da área do disco abaixo da superfície (segmento circular), direção por rotação do eixo de tração (fora de borda).
- **`Boat`** — como o `Aircraft`: não é dono do corpo nem da forma; casco, apêndices, motores (`PistonEngine` com ralenti baixo), hélices, *windage* (`DragElement` no ar), leme −1..1 (+1 vira a proa para estibordo), acelerador −1..1 com banda morta em neutro, telemetria (velocidade na água, calado, adornamento, caimento, volume, centro de carena, força hidrostática, rpm, tração). Mesmo contrato de `applyForces` que o avião; o relógio da água avança depois do `step()` (ou antes de `applyForces` no callback). `floatingHeight()` resolve o equilíbrio estático por bissecção. `buildSmallMotorboat()`: 6 m, 1250 kg, V de 18° na popa e 40° na proa, fora de borda de 75 kW, hélice de 3 pás 0.36 m × 0.43 m de passo, redução 2:1, perna orientável ±30°. Tem um segundo modelo, `BoatHydrodynamics::Prism`, em que agem o `PrismBuoyancy`, os `ScrewPropeller` e os apêndices (motores, hélices e *windage* ficam de fora); a telemetria passa a dar o calado como a profundidade do vértice mais fundo e as rpm como a percentagem de potência do hélice.
- **`PrismBuoyancy`** — por passo: vértices para o mundo e a velocidade do corpo em cada um; cada triângulo do casco dividido em seis à volta do centro e dos meios das arestas, cada pedaço cortado na linha de água (um ou dois triângulos). Num pedaço submerso a pressão é o volume do prisma que vai do pedaço até à superfície — cada vértice empurrado ao longo da normal pela sua profundidade × ρ g, oito tetraedros à volta do centroide dos seis pontos — ao longo da normal; o arrasto é `½ ρ A |v|² |cos θ|` ao longo da normal contra o escoamento, com `v` relativa à velocidade orbital da água, o que dá a sustentação de planeio num fundo com caimento e trava o painel de popa. A força de cada pedaço vai para o vértice de que pende e é aplicada aí no corpo. Com `setEmitEffects` (uma vez por *frame*) manda para um `WaterEffects` o *splash* e o *ripple* dos vértices a correr até 0.2 m abaixo da superfície a mais de 2 m/s, e o *spray* das faces que empurram com força (`|v| cos θ A > 1.5`) na aresta que está à superfície. `setSink` deixa só o arrasto.
- **`ScrewPropeller`** — três pontos do corpo: o cubo, onde a força age e cuja profundidade diz se o hélice morde; um ponto atrás no eixo, para onde corre a tração (ao contrário em marcha-atrás); e um ponto em cima, cuja vertical pelo cubo é o eixo do leme, até ±45° (ou o que `setMaxRudderDegrees` disser; 30° no barco pequeno). Tração = acelerador × máximo, em newtons, nada fora de água. Com efeitos ligados e acelerador acima de 0.1, *splash* e *ripple* no cubo, mais fortes com o hélice perto da superfície. O `Boat` entrega-lhe o leme com o sinal trocado, para que +1 continue a virar a proa para estibordo.

**Números** (todos medidos pelos testes): Arquimedes em cubo, esfera, envoltória convexa e casco do barco totalmente submersos com orientação arbitrária — erro de impulsão ≤ 2e-5, binário em torno do centro de carena ≤ 4e-6 (peso × dimensão); caixa a flutuar com calado `m/(ρLB)` a 0.000%; período de arfagem a −0.05% (60 Hz) de `2π√(m/(ρ g A_wp))`; momento de endireitamento a 0.001% da fórmula *wall-sided* `ρgV sin φ (GM + ½ BM tan² φ)` até 20° (e a ~3% de `ρgV GM sin φ` até 5°), período de balanço a 0.02%; caixa com GM = −0.69 m acaba deitada a 90.0°. Ondas de 80 m e 0.3 m: arfagem 0.31 m e avanço 0.30 m (a caixa é uma partícula); ondas do comprimento da caixa: 0.4 mm. Barco: calado 0.38 m em repouso, velocidade máxima 21.0 m/s (41 nós) a 5540 rpm, círculo de giro de 16 m (2.7 comprimentos) a 60% e leme todo, simétrico a 1%, a adornar para dentro da curva; ré a 3.9 m/s; fora de água tração nula e o motor no limitador; 120 s à deriva em ondas a 60 e 120 Hz sem passar de 18° de adornamento.

**Limites conhecidos.** Sem massa acrescentada: os períodos naturais são os de um corpo num fluido sem inércia, mais curtos do que os reais por `√(1 + m_a/m)` (numa caixa larga e rasa como a do teste, uns 20–40%); é por isso que o teste de arfagem compara com a fórmula e não com uma medição. Sem radiação nem difração de ondas (a onda atravessa o casco sem mudar), sem resistência de ondas explícita, sem esteira do casco, sem spray. A pressão dinâmica é por face (centro de pressão no centroide da área molhada, não junto à raiz do spray), por isso o caimento a planar sai baixo (~0.6° contra 3–5° reais) e a máxima um pouco alta. O slamming depende da discretização do casco junto à linha de água. Ondas só de águas profundas, sem rebentação; a altura em ondas muito íngremes perde exatidão (`L` perto de 0.8 converge devagar). Hidrofólio sem perda de sustentação junto à superfície, sem ventilação nem cavitação; hélice sem cavitação nem binário giroscópico, ré por espelho do regime de marcha à frente. A 80% de acelerador contra mar de 0.7 m o barco salta das cristas e aterra num bordo — sem ninguém a tirar acelerador, o balanço cresce (o teste conduz a 60%).

**Testes** (`tests/3d_marine_*.cpp`): `water` (água plana, Gerstner vazio ≡ plano bit a bit, dispersão, altura contra a superfície deslocada e contra o limite teórico com duas iterações, normal contra diferenças finitas, velocidade orbital contra a derivada da partícula, decaimento, corrente, limite de declividade sem dobras, relógio), `hull` (corte analítico, vértices na linha de água, conservação de área em 2000 triângulos aleatórios, geradores fechados com os integrais certos, solda e reparação), `hydrostatics`, `hydrodynamics` (ITTC, atrito numa placa, pressão e sucção num cubo e a queda com cos^f, ventilação, desaceleração monótona, limite de dissipação, slamming limitado, corrente), `waves` (ondas longas e curtas, doca estática), `appendages` (leme = asa em ρ = 1000, imersão parcial, deflexão, corrente, hélice em água vs ar, ré, imersão do disco, direção, motor no ponto fixo e a disparar fora de água) e `boat`.

**Testbed.** `3D demo: boat` (`testbed/3d/demo_boat_3d.cpp`): o barco por defeito num mar moderado (ondulação de 32 m e duas vagas mais curtas) ao lado de um cais estático; colide como a envoltória convexa da própria malha do casco (o casco não está centrado no centro de massa e uma `BoxShape` não aceita desvio). Deriva 2 s, sai a 80% e a seguir vira a estibordo. A água é desenhada como uma grelha das próprias partículas de Gerstner à volta do barco, translúcida; a linha de água calculada pelo corte aparece a vermelho e a tração a amarelo. Verificações headless: calado são, avança, vira com o leme, fica direito e finito.

## Veículos terrestres: pneus, transmissão, carro, camião e trator

Um módulo novo em `3d/include/ground/` (fontes em `3d/src/ground/`), pela mesma razão do `marine/`: tem um vocabulário próprio (pneu, superfície, motor, embraiagem, caixa, diferencial, reboque) e depende só de `dynamics/` (`RigidBody`, `PhysicsWorld`, juntas) — nem da aerodinâmica nem da marinha. `ground/Ground.h` é o umbrella. Tudo escrito a partir da dinâmica de veículos de livro (eixos SAE J670 adaptados aos do motor: +Z frente, +Y cima, direita em −X; fórmula mágica de Pacejka, elipse de atrito, comprimento de relaxação, geometria de Ackermann, modelo de bicicleta para o gradiente de subviragem).

**O `RaycastVehicle` não mudou.** É o trem de aterragem do avião e a base do `MotorcycleController`, e o seu atrito (impulsos com `frictionSlip`, `engineForce` por roda) está nos números dos testes `3d_aero_*` e na cena `3D demo: raycast vehicle`. O `GroundVehicle` usa a mesma ideia — um raio por roda ao longo da suspensão, rodas fora do broadphase — mas com outra formulação de pneu e de transmissão, e mudar a classe antiga para isto mudaria tudo o que dela depende. Ficam lado a lado.

Peças, cada uma num header:

- **`Tyre`** — sem estado. Forças em escorregamento puro pela fórmula mágica `F0(x) = D sin(C atan(B x − E (B x − atan(B x))))`, com `B` resolvido por Newton a partir do escorregamento de pico configurado (o pico cai exatamente lá: `κ_pico`, `α_pico`), `D = μ(Fz) Fz`. Sensibilidade à carga `μ(Fz) = μ0 (1 + s (Fz − Fz0)/Fz0)`, limitada a [0.5, 1.3] μ0. Escorregamento combinado pela elipse de atrito: `sx = κ/κ_pico`, `sy = α/α_pico`, `s = |(sx, sy)|`, `Fx = (sx/s) Fx0(s κ_pico)`, `Fy = (sy/s) Fy0(s α_pico)` — como `|Fx0| ≤ Dx` e `|Fy0| ≤ Dy`, `(Fx/Dx)² + (Fy/Dy)² ≤ 1` sempre; entre `s = 1` e `s = 3` a direção mistura-se para a do deslizamento `(κ, tan α)`, para uma roda bloqueada e virada empurrar contra a velocidade de deslizamento (atrito seco) e não de lado. Momento autoalinhante `Mz = −t Fy` com rasto pneumático a desaparecer em `s = 2`; camber como ângulo de deriva equivalente; resistência ao rolamento `C_rr Fz`. Tabelas por `SurfaceType` (asfalto, molhado, gravilha, relva, lama, gelo, terra, neve) de μ e de `C_rr`, e três predefinições: pneu de estrada de carro (205/55 R16), de camião (simples ou duplo) e agrícola com garras (μ mais alto em terra do que em asfalto, pico de escorregamento 0.2). `secantStiffness()` dá `kx = Fx/κ`, `ky = Fy/α` (≥ 0, finitos em zero).
- **`Engine`** — curva de binário a plena carga por tabela (≤ 24 pontos, interpolação linear), **líquida** (o que um dinamómetro mede): a combustão fornece `acelerador · (curva + atrito)`, por isso acelerador a fundo dá a curva e acelerador fechado dá `−atrito` (travão motor). Atrito `a + b·rpm + c·rpm²`, controlador de ralenti proporcional (abre abaixo do ralenti, a base desaparece acima dele), limitador suave (o combustível desvanece numa banda abaixo do limite: estabiliza onde o binário restante iguala a carga, sem caçar), calado abaixo de `stallRpm`, motor de arranque. Geradores `gasoline()` e `diesel()` (patamar de binário, queda do regulador até ao ralenti alto, limitador um quarto antes dele).
- **`Transmission.h`** — `Clutch` (capacidade = `maxTorque · engagement`), `Gearbox` (relações com marcha-atrás e neutro, desmultiplicação final, rendimento como binário de perda `(1 − η)|T|` + arrasto — dissipativo para qualquer sentido do fluxo de potência —, modos manual e automático com pontos de passagem por rpm interpolados com o acelerador, histerese, intervalo mínimo e tempo de passagem em neutro; o automático decide pela velocidade da estrada (a velocidade sem escorregamento que o `GroundVehicle` calcula pela árvore de diferenciais), nunca engrena uma relação que faria o motor disparar), `Differential` (aberto, bloqueado, autoblocante por discos com pré-carga + sensibilidade ao binário, viscoso; `split` para diferenciais centrais 40/60; bloqueável) e `DriveOutput` (roda ou outro diferencial).
- **`GroundVehicle`** — o veículo: eixos (duas rodas espelhadas cada), suspensão por raio, pneus, direção, transmissão, controlos (acelerador, travão, travão de mão, direção −1..1 com +1 à direita, pedal de embraiagem, `shiftUp`/`shiftDown`/`selectGear`, bloqueios de diferencial, motor de arranque), telemetria por roda (carga, Fx, Fy, Mz, escorregamentos transitório e cinemático, ω, ponto e superfície de contacto, compressão) e do veículo. Mesmo contrato de `applyForces(step)`/`stepCallback` que `Aircraft` e `Boat`.
- **`TrailerCoupling`** — bola/gancho (`PointJoint`) ou quinta roda (`ConeTwistJoint` com o eixo de torção na vertical: articulação ±80°, cabeceio ±15°, rolamento ±2.3°), ignora o par na colisão e liga a linha de travão: os travões do reboque seguem o pedal do trator.

**Como um passo é resolvido.** Cada roda lança um raio pela sua suspensão (do topo da roda em compressão total até além da extensão total). Cada roda no chão dá três linhas a uma única resolução em velocidade sobre o chassis (6 GL) e todos os veios (motor, entrada da caixa, entrada de cada diferencial, cada roda):

1. **suspensão** — mola + batente progressivo + amortecedor (compressão/extensão diferentes) + barra estabilizadora, linearizados no fim do passo (Euler implícito), `λ = A − B u`, `λ ≥ 0`; para além da compressão total a linha passa a contacto rígido, com o impulso nunca abaixo do da mola em compressão total (senão um carro que bate no batente fica lá sentado);
2. e 3. **pneu** — um estado de deflexão da carcaça `u` por direção, `du/dt = −(velocidade de deslizamento) − |vx| u / σ`, com o modelo avaliado nos escorregamentos transitórios `κ' = u_x/σ_x`, `tan α' = u_y/σ_y`. A velocidade alta é o atraso de primeira ordem habitual do escorregamento; parado (`|vx| → 0`) a deflexão é uma mola, por isso nada divide pela velocidade e um carro estacionado numa rampa não desliza. A força é a rigidez secante vezes a deflexão no fim do passo, com amortecimento da carcaça a baixa velocidade — implícita, e um pneu cuja rigidez de escorregamento pediria passos de 0.1 ms explicitamente é estável a 60 Hz. As duas direções ficam presas à elipse de atrito. **A deflexão integra o que a zona de contacto fez de facto**: cada passo começa por a avançar com o escorregamento medido depois do passo do mundo (juntas e contactos incluídos), não com o que a resolução do veículo previu — foi assim que se descobriu um camião travado a deslizar a 0.056 m/s empurrado pelo semirreboque: cada veículo previa parar, a junta empurrava os dois, e a deflexão nunca "sentia" o movimento;
4. **atrito da roda** — travão, travão de mão, resistência ao rolamento e rolamento como um único elemento de Coulomb limitado pela soma, por isso uma roda travada bloqueia em exatamente zero, sem oscilar à volta dele.

A transmissão junta o atrito do motor (limitado), a embraiagem (limitada pela capacidade), a engrenagem da caixa, a cinemática de cada diferencial (`ω_in/R = p ω_A + (1−p) ω_B`, que dá `T_A = p R T_in`), bloqueios (rígidos), autoblocantes de discos (limitados) e viscosos (moles). A gravidade e tudo o que já foi somado aos acumuladores do corpo neste passo entram nas velocidades livres (o `RigidBody` ganhou `forceAccumulator()`/`torqueAccumulator()` só de leitura para isso), por isso o que a resolução segura fica seguro depois da integração.

O conjunto é o problema quadrático convexo `min ½ λᵀ(W + D)λ − bᵀλ` com caixas, `W = J M⁻¹ Jᵀ`, `D = 1/B` nas linhas moles (uma regularização relativa de 1e-9 nas rígidas, para conjuntos redundantes — um diferencial bloqueado entre duas rodas bloqueadas — continuarem fatorizáveis). Resolvido por um método de conjunto ativo primal (`VehicleSolver`, interno): λ sempre admissível, cada iteração vai ao mínimo das linhas livres por Cholesky em double — uma cadeia de 300:1 entre um volante leve e rodas pesadas fica exata, onde um método iterativo precisaria de milhares de varrimentos — ou pára no primeiro limite no caminho e prende-o; um limite cujo gradiente diz que a linha quer sair é solto, um de cada vez. A primeira versão era um conjunto ativo "monótono" que fixava tudo o que violasse a solução com todos os atritos colados: o atrito de rolamento minúsculo "travava" uma roda a patinar e o pneu ficava preso do lado da travagem — força a trocar de sinal todos os passos. A elipse é tratada por fora: um par fora dela é encaixado numa caixa com o canto na elipse na direção do impulso que o par quer, e repetido (≤ 4 rondas). Por passo: começa do impulso do passo anterior quando as linhas são as mesmas (chaves por linha); os limites dos pneus usam as cargas do passo anterior e a resolução repete-se se as cargas encontradas diferirem mais de 5%. Custo em Release: ~0.02 ms por passo para o carro (21 linhas), ~0.08 ms para camião + semirreboque (31 + 24).

**Embraiagem automática** (sempre no automático, opcional no manual): engata com a mais rápida das velocidades do motor e da entrada da caixa, entre `engageRpm` e uma velocidade que sobe com o acelerador até `launchRpm` (o motor fica na banda enquanto a embraiagem patina — não cala e um arranque a fundo é feito em regime alto); quando a entrada da caixa sozinha passa de `engageRpm`, o resto do engate faz-se em ~1 s, como um condutor acaba de soltar o pedal; abre em cada passagem, fecha em `engageTime` depois dela, com o acelerador a voltar ao mesmo ritmo (o motor não dispara durante o reengate). Largar a embraiagem ao ralenti em primeira **sem** automatismo cala o motor em 0.05 s (o carro move-se 9 cm) — documentado e testado.

**Direção.** Ângulo de referência (bicicleta) no eixo dirigido mais afastado da linha de referência (média dos eixos não dirigidos — o meio do tandem num 6x4). Com `ackermann = 1` cada roda aponta para o mesmo centro nessa linha: `tan δ = d / (x + R)`, `R = d_p / tan δ_ref` com sinal, por isso a roda interior vira mais e um segundo eixo dirigido (8x4) vira pela sua própria distância. Limite de velocidade de direção.

**Eixo dianteiro oscilante do trator — modelo de partilha de carga, não corpo + `HingeJoint`.** O eixo é uma viga sem massa num pivô longitudinal: dentro de ±10° as duas rodas têm a mesma compressão (a do pivô), por isso partilham a carga em partes iguais e o eixo não transmite binário de rolamento ao chassis; os seus raios alcançam mais fundo pelo que a viga pode rodar; no batente passa a eixo rígido e a diferença vai para uma roda. Escolhido em vez de um corpo extra com junta porque é exato para um eixo leve, entra na mesma resolução implícita (as tensões de 300:1 em binário passam pelos pneus, não por uma junta resolvida noutro sítio com 8 iterações) e não deriva. A inércia de rolamento do eixo fica de fora.

**Predefinições** (`GroundVehiclePresets.h`; massa, inércia, amortecimento e sono do corpo; caixa de colisão, altura de repouso e pontos de engate no `GroundVehicleLayout`; alturas dos eixos calculadas para o corpo ficar nivelado à altura pedida com as cargas estáticas): carro de 1400 kg, 100 kW, cinco velocidades, tração dianteira/traseira/integral (central 40/60); camião 6x4 de 9 t, 300 kW / 2000 N m, 12 velocidades automatizadas, diferencial entre eixos e de eixo bloqueáveis, quinta roda; 8x4 de 12 t com dois eixos dirigidos; semirreboque de três eixos de 7 t + carga (~40% no pino); trator 4x4 de 5 t, 100 kW, dez velocidades com redução final de 25:1 (300:1 em primeira, 2.7 km/h ao ralenti alto), ligação rígida às rodas da frente com a relação certa para as duas medidas de pneu, traseiro bloqueável, dianteiro autoblocante bloqueável, sem molas (a suspensão é o pneu), engate de barra a 0.5 m; reboque agrícola de dois eixos com barra (~8% no engate).

**Um bug das juntas que isto apanhou.** Todas as juntas que invertem uma matriz de massa efetiva 2×2 ou 3×3 (ponto, dobradiça, cone-torção, universal, fixa, deslizante, pistão, roda, rato) decidiam "singular" por `|det| > 1e-9` absoluto. O determinante escala com a potência N das massas inversas: para um corpo de mais de ~1 t (o semirreboque de 19 t tem ~5e-5 por eixo, det ~1e-13) a junta recebia massa efetiva zero e não fazia nada — o reboque caía do trator. Agora `detail::invertible()` (`JointAxis.h`) compara com `1e-9 · escala^N`, o mesmo limiar para entradas unitárias. `3d_joints_heavy_bodies` pendura 20 t de cada junta: com o limiar antigo separa-se 19.8 m, com o novo 1.6 mm como o controlo de 1 kg.

**Números** (todos medidos pelos testes): pneus com pico exatamente no escorregamento configurado, antissimetria exata, elipse ≤ 0.999998; travagem a fundo de 20 m/s com rodas bloqueadas em 0.18 s, ω nunca troca de sinal, 25.86 m contra `v²/(2μg)` = 26.32 m com o μ de bloqueio do próprio modelo (−1.7%); travão de mão bloqueia só a traseira; ABS mantém a direção (51° de rumo contra 5° bloqueado); parado em 20% com travão de mão: 0.13 mm em 20 s; velocidade máxima por relação contra `rpm_limite · 2πr / (60 · relação · final)`: carro −0.5/−0.65/−0.97% (1.ª–3.ª), trator −0.53…−0.78% (1.ª–10.ª); 0–100 km/h em 11.4 s (manual) e 11.0 s (automático); embraiagem bloqueada em terceira: 43.9 N m com desvio de 0.41 N m e variação máxima por passo de 0.01 N m; rendimento medido 0.9479 contra 0.9488 esperado; automático sobe 1-2-3-4 sem descer; trator 300:1 com tudo bloqueado: 2329 rpm constantes, desacerto entre veios 7e-7 rad/s; diferencial aberto com uma roda em gelo (μ 0.05): 1.2 m/s em 5 s com a roda do gelo a 27.6 m/s, autoblocante de discos 2.7, viscoso 3.4, bloqueado 3.8 m/s; razões de velocidade das rodas em curva iguais às dos raios das trajetórias a 0.05%; repartição 40/60 do central medida em 0.400; Ackermann exato (erro < 1e-5 rad); raio de viragem a baixa velocidade +0.17%/+0.30% de `L/tan δ`; gradiente de subviragem 0.44°/g no carro 60/40 e 0.09°/g no 40/60 — a mudança (−0.35°/g) segue a dos termos W/C dos pneus (−0.27°/g); a parte que sobra (rasto pneumático, repartição da rigidez ao rolamento, Ackermann com transferência de carga) é toda subviradora; barras estabilizadoras ×0/×1/×4: rolamento 2.36°/1.19°/0.48° a 3.2 m/s²; 8x4 +4.9% (arrasto do tandem); camião + semirreboque em círculo a 6 m/s: articulação 24.8° contra 25.0° cinemáticos (`asin(L/R) − asin(d/R)`), separação máxima 0.12 mm; meia travagem de 10 m/s: 13.1 m com os travões do reboque na linha, 23.6 m só com os do trator; trator sobe 30% de terra em primeira com os diferenciais bloqueados (0.65 m/s, 1.6% de escorregamento), com o lado esquerdo em barro (μ ≈ 0.25) sobe 10.2 m bloqueado e 5.5 m aberto (60% de escorregamento); lomba de 0.3 m de um lado: carga mínima por roda 3.1 kN com o eixo a oscilar, roda no ar com o eixo rígido; puxa 8.5 t de reboque em terra a 2.30 m/s (2.33 no limitador), separação do engate < 0.01 mm; um minuto de condução com os três conjuntos a 60 e 120 Hz em chão plano e em trimesh ondulada: finito, direito, separação ≤ 0.22 mm.

**Limites conhecidos.** Um raio por roda: o contacto é um ponto por baixo do cubo ao longo da suspensão, sem envolvente do pneu — um degrau é sentido de uma vez e uma roda não sobe um degrau vertical da altura do seu raio (as lombas dos testes são suaves por isso). Sem massa não suspensa: o salto das rodas e o "wheel hop" vêm só da rigidez dos pneus; a suspensão é vertical pura (sem curvas de camber/convergência; o camber é estático + rolamento do corpo); as forças do pneu atuam no ponto de contacto (anti-afundamento/anti-mergulho efetivamente totais) e o binário de reação do motor não rola o chassis. O corpo onde as rodas assentam é tratado como de massa infinita dentro da resolução (a reação é aplicada depois). Cada veículo resolve-se sozinho e as juntas de engate pelo mundo (separação de operadores, estável, com a deflexão a fechar o ciclo). Mapa de acelerador linear (a 35% o carro arranca a ~0.8 m/s²); sem conversor de binário (o automático usa a embraiagem automática); sem temperatura, desgaste nem pressão dos pneus; raio de rolamento = raio livre; ABS simples por limiar, sem controlo de tração nem estabilidade; eixo oscilante sem inércia própria. Com tudo bloqueado num trator em aderência assimétrica forte, o lado com aderência guina-o para o lado sem ela mais depressa do que a direção corrige — real, e por isso o teste de rampa assimétrica usa barro a μ ≈ 0.25 e não gelo.

**Testes** (`tests/3d_ground_*.cpp`, `tests/ground_rig.h` com o cenário comum): `tyre`, `brakes`, `drivetrain`, `differential`, `steering`, `suspension` (sag estático, período de arfagem, queda nos batentes, compressão total, rodas no ar), `truck`, `tractor`, `stability_car`, `stability_truck` (plano), `stability_truck_bumpy`, `stability_tractor` (cada um < 30 s com ASan); mais `3d_joints_heavy_bodies`.

**Testbed.** `3D demo: ground vehicles` (`testbed/3d/demo_ground_vehicles_3d.cpp`): três faixas no mesmo mundo — o carro automático acelera pelas mudanças, trava a fundo para uma curva à direita e sai a acelerar; o 6x4 arranca com o semirreboque e vira; o trator em quarta passa uma lomba de 4 m × 0.25 m debaixo das rodas esquerdas (o eixo da frente oscila) e sobe uma rampa de 15% em terra (uma trimesh). As linhas mostram as forças dos pneus (amarelo) e as cargas (ciano). C troca a câmara de veículo. Verificações headless: velocidade, travagem e viragem do carro; arranque, viragem, articulação e engate do camião; quatro rodas carregadas na lomba e topo da rampa do trator; tudo direito e finito.

## Atuadores: servos nas juntas, sensores, hidráulica, guincho, gancho e motor elétrico

A base para as fases seguintes (máquinas pesadas — grua, escavadora, lagartas —, as ligações de comando de um helicóptero e robótica). Duas camadas, pela mesma regra de sempre: o que é de uma junta fica na junta, o que é um aparelho com vocabulário próprio vai para um módulo novo.

- **Nas juntas** (`3d/include/dynamics/`): um servo de posição em cada eixo acionável e sensores em todas.
- **`3d/include/actuators/`** (fontes em `3d/src/actuators/`, umbrella `actuators/Actuators.h`): `HydraulicCylinder`, `Winch` + `Rope`, `Hook`, `ElectricMotor`. Dependem só de `dynamics/` e de `collision/` (a pesquisa do gancho). Um módulo à parte e não `dynamics/` porque é o que as próximas fases vão usar e alargar (bombas partilhadas, válvulas de contrapeso, lagartas), e porque um cilindro ou um guincho não são juntas genéricas: têm caudal, pressão, tambor, travão.

Escrito a partir de mecatrónica e controlo de livro: PD implícito numa linha de restrição, cilindro de duplo efeito com caudal e válvula de alívio, tambor com motor e travão, e restrições criadas e removidas em tempo de execução.

### `JointDrive`: um motor ou um servo por eixo

`dynamics/JointDrive.h` é a linha de acionamento que as juntas partilham — `HingeJoint` (ângulo), `SliderJoint` (posição), `PistonJoint` (posição e ângulo, um de cada), `UniversalJoint` (os dois eixos), `WheelJoint` (direção e rotação). Por eixo, **um** modo de cada vez:

- **Velocidade**: o motor de sempre (velocidade alvo, força/binário máximo), agora com um ganho viscoso opcional: `F = c (v_alvo − v)`, limitado — a reta binário-velocidade de um motor elétrico, resolvida implicitamente.
- **Posição**: o servo de `ServoSettings`. Três maneiras de dizer a rigidez: `frequency` (Hz) + `dampingRatio`, escaladas todos os passos pela inércia que o eixo realmente move (ζ = 1 é sem sobre-elongação, qualquer que seja a carga); `stiffness` + `damping` absolutos (kp, kd em N/m e N·s/m ou N·m/rad e N·m·s/rad); ou nada (rígido). `maxForce` é o limite do atuador; `maxVelocity` > 0 limita a velocidade de aproximação a um alvo longe. O alvo pode mudar todos os passos (`setServoTarget(alvo, velocidadeFeedForward)`: o alvo é o ponto de partida do passo, a mover-se à velocidade dada). Ângulos dão a volta pelo lado curto, a não ser que os limites só deixem o comprido; o alvo é limitado aos limites.

`setMotor()`/`setServo()` substituem-se um ao outro; `disableMotor()`/`disableServo()` desligam a linha. `drive()` dá a linha em si (para o `ElectricMotor`, por exemplo). `ConeTwistJoint` ganhou um servo de orientação de três eixos (`setServo(quat, ServoSettings)`, torção e balanço juntos, pelo caminho mais curto; `orientationFromTwistSwing()`, `twistAngle()`, `swingAngles()`).

**A mola é implícita.** O PD é a mola-amortecedor do `SoftSpring` dobrada na linha: `F = −k x_fim − c v_fim` dá `v + k e/(c + h k) + γ λ = 0` com `γ = 1/(h (c + h k))`. Incondicionalmente estável: 1 kHz ou `kp = 1e9 N·m/rad` a 60 Hz tornam-se o servo rígido, não uma explosão. Preço: a integração implícita amortece por si — o servo de ζ = 0.3 sobe 30.8% em vez dos 37.2% contínuos.

**`maxVelocity`.** Quando a velocidade que a mola pediria (contando o termo `γ λ` do binário que já segura — é ele que deixa uma carga a descer fugir à mola) passa o limite, a linha passa a ser um motor de velocidade rígido a essa velocidade até a mola pedir menos. Medido: 0.5000 rad/s exatos rígido e mole; a baixar uma carga, 0.4000 com limite 0.4 (antes da correção, 0.418).

**O servo rígido** fecha um quinto do erro por passo na passagem de velocidade e o resto na passagem de posição da junta (`JointDrive::positionError`), como as outras linhas duras da junta. Fechar tudo na velocidade, num passo (o servo *deadbeat* que a direção da `WheelJoint` tinha), é exato numa junta sozinha mas numa cadeia as iterações de velocidade não convergem o bastante: cada junta corrige a mais pelas vizinhas e a cadeia vibra, o binário a trocar de sinal todos os passos. Com o quinto, três elos com servos rígidos param exatamente no alvo (erro < 1e-4 rad).

**A inércia que o eixo move.** A linha angular de uma dobradiça sozinha vê a inércia do braço no centro de massa; preso no pivô, o braço roda com a inércia no pivô (4× para uma barra uniforme), e uma frequência calculada com a primeira saía o dobro. `detail::coupleDriveToLocks` (`JointAxis.h`) calcula o complemento de Schur `J_d M⁻¹ J_dᵀ − cᵀ K⁻¹ c` contra as linhas de bloqueio da junta (ponto + rotação na dobradiça; laterais + rotação no deslizante e no pistão; ponto + perpendicularidade na universal — sem esta, um servo num eixo de um veio inclinado empurra-o em torno do seu próprio eixo longo e leve) e escala sempre os ganhos de frequência por ele. A forma de resolver depende do par:

- **um lado imóvel** (estático/cinemático — a primeira junta de uma cadeia): a linha é resolvida em bloco com os bloqueios — cada impulso da linha leva os impulsos de bloqueio `−K⁻¹ c` que os deixam quietos, e a massa da linha é a reduzida. O servo obtém o que pede numa iteração: um servo a 0.5 rad/s com limite de velocidade fica em 0.5000, e não 8% acima;
- **os dois dinâmicos** (dentro de uma cadeia): a linha sozinha, sem compensação. Os impulsos de bloqueio do bloco empurram o elo-pai ao comprido e, com todas as juntas a fazê-lo, a cadeia deixa de convergir (servos rígidos a vibrar com 8 iterações, exatos com 16).

### Sensores

`Joint::reactionForce()`/`reactionTorque()` (virtuais, zero por defeito): força e binário que a junta aplicou ao corpo B no último passo resolvido — a soma dos impulsos acumulados de todas as linhas a dividir pelo passo — em coordenadas do mundo, N e N·m, a força na âncora de B e o binário em torno dela; A recebe o simétrico. Uma massa pendurada lê `+m g` para cima. Implementado em todas as juntas 3D. Por junta e por eixo: posição/ângulo (já existiam), velocidade (`currentAngularVelocity()`, `currentVelocity()`, ...), e a força de cada linha — motor/servo (`motorTorque()`, `motorForce()`, ...), limite (`limitTorque()`, `limitForce()`), bloqueios (`lockForce()`, `lockTorque()`) — com sinal positivo no sentido em que a coordenada cresce. Uma junta a dormir guarda a última leitura.

### `HydraulicCylinder`

Um cilindro de duplo efeito entre um pino num corpo (a camisa) e um pino noutro (o olhal da haste): **uma linha ao longo da linha entre os pinos**. Porque não um deslizante: o cilindro da lança de uma escavadora não está na junta da lança — a lança roda na sua dobradiça e o cilindro, preso nos dois pinos, empurra de través, um membro de duas forças cujo comprimento fixa o ângulo pela geometria da ligação. Assim as dobradiças continuam dobradiças e a força chega à lança com o braço que os pinos lhe dão; um corpo de cilindro num deslizante juntaria dois corpos e três juntas por cilindro para dizer o mesmo.

Parâmetros: diâmetros de êmbolo e haste (áreas `A_cap` e `A_anel`), curso e comprimento fechado, caudal da bomba (L/min), pressão de alívio (Pa), amortecimentos de fim de curso (comprimento e fração de velocidade no batente), banda morta e tempo de resposta da gaveta (atraso de 1.ª ordem, 50 ms — uma gaveta instantânea bate nos corpos com um degrau de velocidade a cada movimento da alavanca), e compressibilidade opcional (módulo volumétrico, volume morto, amortecimento). Por passo:

- válvula proporcional u ∈ [−1, 1]: a sair, `v = u Q / A_cap`; a recolher, `v = u Q / A_anel` (mais rápido para o mesmo caudal);
- **a coluna de óleo é a posição**: o curso que o óleo já bombeado diz, integrado do caudal; a linha segura o êmbolo lá. Rígida: motor de velocidade à velocidade medida + correção do erro na passagem de posição (como as dobradiças com que trabalha — corrigir na velocidade num passo lutava com a passagem de posição delas até a cadeia vibrar entre as válvulas de alívio). Compressível: a mola do óleo `k = β (A_cap²/V_cap + A_anel²/V_anel)`. Válvula fechada: zero deriva;
- a força está limitada a `[−p A_anel, p A_cap]`. Uma carga que precise de mais abre o alívio: o êmbolo cede e a coluna segue-o (o óleo que passou pelo alívio foi-se). Contra um batente, o cilindro para a empurrar exatamente `p A_cap`;
- os amortecimentos estrangulam os últimos centímetros; os fins de curso são batentes duros;
- uma carga que arrasta (a lança a descer com o seu peso) vai à velocidade medida, como as válvulas de contrapeso de uma máquina real fazem.

Leituras: curso, velocidade, força (+ a empurrar), pressões das câmaras, caudal, alívio aberto, força nos batentes.

### `Winch` e `Rope`

`Rope` é uma restrição de distância unilateral sobre o **comprimento total do caminho**: de um ponto num corpo, por cima de roldanas (pontos fixos em corpos, estáticos ou não), até um ponto noutro corpo. Folgada não faz nada (alvo especulativo `−C/h`, como um contacto); esticada segura o comprimento (dura, com correção de posição, ou elástica: `k = EA/L`, implícita). `setFalls(N)`: o último troço conta N vezes — um moitão de N cabos sobe 1/N do cabo recolhido e cada cabo leva 1/N da carga. Como o comprimento é a soma dos troços, o cabo corre livre sobre as roldanas com a mesma tensão dos dois lados (sem atrito nem raio).

`Winch` é o tambor no primeiro ponto: um grau de liberdade próprio (a velocidade do cabo), resolvido em bloco 2×2 com a linha do cabo. O motor leva-o à velocidade comandada com no máximo `maxLinePull`; o travão (automático com o comando a zero, ou à mão) segura-o com no máximo `brakeLinePull`; o cabo tira-lhe cabo com a tensão. Por isso a carga sobe à velocidade do tambor até precisar de mais tração do que o motor tem e aí para (ou desce) exatamente em `maxLinePull`. `setFreeSpool(true)` desembraia o motor (guincho de queda livre). Limites de comprimento (`setLengthLimits`) param o tambor — o fim de curso de içar.

### `Hook`

Prende e solta cargas em tempo de execução: um `PointJoint` (a carga baloiça, como num gancho) ou um `FixedJoint` (íman, garra) entre o corpo do gancho e a carga, **feito na pose relativa do momento** — a junta nasce sem erro, nada salta. `attachNearest(raio)` usa o `overlapSphere` do mundo e escolhe pela distância à **superfície** (uma esfera do raio de captura no ponto do gancho, a penetração mais funda), depois pelo centro: uma viga cuja ponta está debaixo do gancho ganha a uma caixa com o centro mais perto. Pode ser chamado entre passos, do `StepCallback` ou de um evento de contacto — o mundo adiciona e remove juntas imediatamente e nenhum destes sítios está dentro de uma passagem sobre elas. Acorda os corpos, ignora a colisão gancho-carga enquanto preso (e repõe-na). Se a carga sair do mundo enquanto presa, o mundo larga a junta com ela; o gancho repara (`attached()` passa a falso, por *handle* com geração) e o `detach()` seguinte é seguro.

### `ElectricMotor`

Motor DC de ímanes permanentes e redutor, pelos números do catálogo (binário de bloqueio, velocidade em vazio, correntes de bloqueio e em vazio, tensão, relação, rendimento), em qualquer `JointDrive`. Na saída: `Ts = T_bloqueio · i · η`, `w0 = w_vazio / i`, `T = Ts (u − w/w0)` limitado a ±Ts (o limite de corrente). Em tensão é uma linha de velocidade com ganho viscoso `Ts/w0` — a reta exata a qualquer carga; em posição, um servo cuja saída fica dentro do envelope da reta a u = ±1. Leituras: binário e velocidade na saída e no motor (o rendimento tirado no sentido em que a potência passa), corrente `I = I0 + (Is − I0) T/Ts`, potências elétrica e mecânica. Não modelado: inércia do rotor refletida (`i² J`), indutância, aquecimento.

### Sono

Um atuador com trabalho para fazer acorda os corpos e mantém-nos acordados — mesmo a mover-se mais devagar do que o limiar de sono do mundo deixaria (um servo a 0.3 rad/s, um cilindro a 6 mm/s, um guincho a 5 cm/s); um parado deixa-os dormir. O servo pergunta-se se está em equilíbrio com o que segura: `e + F/k` (a distância ao equilíbrio; o erro todo num servo rígido) acima de 1 mm/1 mrad acorda, a não ser que esteja saturado (parado contra a carga, nada mais a dar); um alvo novo acorda sempre.

### O que mudou nas juntas existentes

- Os motores de `HingeJoint`, `SliderJoint`, `PistonJoint`, `UniversalJoint` e `WheelJoint` passaram para `JointDrive`. A API antiga mantém-se (`setMotor`, `disableMotor`, `setLinearMotor`, ..., `setSteeringServo(alvo, binário, velocidade)` e os getters da roda).
- **Um motor de velocidade com alvo zero já não acorda os corpos todos os passos** (um travão a zero pode deixar dormir); alvo não nulo ou uma mudança de alvo acordam, como antes.
- **Linhas de acionamento de uma junta com um lado imóvel são resolvidas em bloco com os bloqueios** (ver acima): o motor de uma dobradiça presa ao chão atinge a velocidade alvo no passo, com a inércia no pivô, em vez de convergir ao longo das iterações. Entre dois corpos dinâmicos, os números de antes.
- **O servo de direção da `WheelJoint`** é agora o servo rígido comum: um quinto do erro por passo em velocidade + passagem de posição (era o erro todo por passo), e dá a volta a ±π sem limites.
- `Joint` ganhou `reactionForce()`/`reactionTorque()` virtuais e inclui `Math.h`.

### Números (todos medidos pelos testes)

Servo numa barra de 10 kg e 1 m: ζ = 1 sem sobre-elongação (−0.0002%), erro de 1.2 mrad a 1.5 s para 1 rad a 1 Hz; ζ = 0.3 → 30.8%, ζ = 0.5 → 13.3% (contínuo 37.2/16.3%); período a 1 Hz 1.0037 s (o discreto do integrador implícito é 1.0036); a segurar na horizontal com 2 Hz, −0.0926 rad, igual a `k e = m g L cos e` com `k = I_pivô ω²`; rígido a 0.3 rad, erro 6e-8 rad com 46.86 N·m (= m g L cos θ); limitado a 30 N·m contra 49 N·m de carga, satura em 30.000 e nunca passa −0.46 rad; voltas por ±π sem passar perto de zero; 1 kHz e 1e9 N·m/rad estáveis a 60 Hz; seno seguido a 1 mrad (rígido) e 7 mrad (8 Hz); três elos rígidos exatos, juntas a 1e-6 m paradas (a abrir 1.4 cm a 1.5 rad/s — a cadeia livre a cair abre 1.8 cm, é a precisão das próprias juntas a 60 Hz). Deslizante, pistão (os dois eixos), universal (os dois) e cone-torção (torção e balanço juntos) no alvo.
Sensores: m g a 0.000% em oito juntas; servo e limite = m g L cos θ ao 1e-4; batente do deslizante = m g; bloqueios de rotação = m g L.
Hidráulica: velocidade = Q/A_cap e Q/A_anel a 1e-5, com 50 kg e 1 t; bloqueio a p A_cap = 196 349.5 N e p A_anel = 147 262.2 N exatos; 3 t seguras 10 s com deriva 0 (rígido e compressível); 7.85 kN de alívio debaixo de 1 t cede até ao batente, debaixo de 500 kg segura; a escavadora de três elos (900/450/300 kg) segue três poses a < 5 mrad, com juntas abertas no máximo 6.5 mm em movimento e 0.2 mm paradas (2.9 e 0.05 mm com 16 iterações), deriva 1e-4 rad com as válvulas fechadas.
Guincho: 1/2/4 cabos → 1.0000/0.5000/0.2500 m/s com o cabo a 1 m/s, tensão = carga/N; 20 kN em 2 cabos sobem 3.5 t e não 4.5 t (o motor para em 20.09 kN e a carga desce); cabo folgado: queda livre exata, tensão 0; Atwood 10/5 kg sobre duas roldanas a 3.2700 m/s após 1 s (g/3 exato), comprimento a 5e-7 m; 10 t em 30 m a 60 Hz: tensão = m g, nem estiramento nem deriva, içado 1.198 m em 4 s a 0.3 m/s; elástico (EA = 2e8 N) estica 3.41 cm contra 3.44 de `m g L/EA`; travão segura a 0, escorrega acima da capacidade a 30.1 kN; tambor livre: `a = g m/(m + J/r²)` exato.
Gancho: a carga mexe 1e-12 m ao prender; sobe com o gancho a 0.5000 m/s; pesa m g no gancho; solta-se e cai; tudo do `StepCallback` limpo com ASan.
Motor: vazio 8.0000 rad/s, bloqueio 80.000 N·m e 30.000 A, a 25/50/75% do binário 6/4/2 rad/s exatos, corrente na reta.

**Testes** (`tests/3d_actuators_*.cpp`, `tests/actuator_rig.h`): `servo`, `sensors`, `hydraulics`, `winch`, `hook`, `motor`, `sleep` (cada um < 3 s com ASan).

**Testbed.** `3D demo: actuators` (`testbed/3d/demo_actuators_3d.cpp`): à direita, um braço de escavadora num pedestal — lança, braço e um balde em L (parede e chão presos por uma `FixedJoint`) — com três cilindros entre os pinos dos elos. O operador transforma o caminho do pivô do balde em ângulos (cinemática inversa de dois elos) e abre cada válvula para a velocidade de curso que os segue (o braço do cilindro em torno do pivô converte uma na outra) mais uma correção proporcional: o balde desce, desliza o chão por baixo de uma caixa pousada em duas travessas, enrola e levanta-a. À esquerda, um guincho no chão com o cabo por cima de uma roldana no topo de um mastro e dois cabos até um moitão: desce até à carga, prende-a por raio de captura, iça-a 2 m, segura-a no travão e larga-a. Verificações headless: o pivô segue o guião a 2.9 cm e o balde a 0.024 rad, a caixa sobe a 1.6 m dentro do balde, a carga é presa aos 3.1 s e içada 2.06 m, volta ao chão depois de largada, tudo finito, juntas abertas < 5 mm e cabo < 8 mm.

**Limites conhecidos.** As roldanas não têm raio nem atrito, e o cabo não tem massa nem catenária. Uma bomba por cilindro: caudal partilhado entre funções (a prioridade e a soma das exigências de uma bomba real) fica para a fase das máquinas. O balde da escavadora é direto (sem a ligação de quatro barras que o tira do ponto morto). Corpos de roldanas a meio do cabo que sejam dinâmicos têm de estar presos por juntas a uma das pontas (é isso que os põe na mesma ilha de sono) e o cabo tem de sair do mundo antes deles. As linhas da `WheelJoint` (direção e rotação) não entram no bloco com os bloqueios: a roda pende de um chassis dinâmico, onde o bloco não se usa de qualquer forma. A frequência dos servos é a do par de corpos da junta: elos mais à frente numa cadeia acrescentam inércia e peso que ela não vê (três elos a 8 Hz descaem até 0.064 rad). (O raio e o atrito das roldanas, a bomba partilhada, as quatro barras do balde e a inércia de fora nos ganhos dos servos foram resolvidos na fase seguinte — ver «Máquinas pesadas».)

## Máquinas pesadas: juntas articuladas, sistema hidráulico, lagartas, escavadora, grua e bulldozer

Três camadas novas. **`dynamics/ArticulationSolver`**: o solver direto para cadeias e malhas pesadas (o braço de uma escavadora, a lança de uma grua, uma ligação de quatro barras), que tira o erro das juntas de 1.8 cm para ~1 µm. **`actuators/`** ganhou `HydraulicMotor` e roldanas com raio e atrito no `Rope`. **`3d/include/machinery/`** (fontes em `3d/src/machinery/`, umbrella `machinery/Machinery.h`): `HydraulicSystem`, `Terramechanics` + `TrackedUndercarriage`, `SoilCutting`, `LoadMomentIndicator`, `AntiSway` e as máquinas `Excavator`, `MobileCrane`, `Bulldozer`. Escrito de engenharia de livro: cinemática de gruas e tabelas de carga, circuitos hidráulicos de centro aberto e *load sensing* com partilha de caudal, Bekker e Janosi-Hanamoto, a equação fundamental do movimento de terras (McKyes), ligações de quatro barras, pêndulos e *input shaping*.

### Precisão das juntas em cadeias pesadas: o que se mediu e o que se escolheu

O problema: o Gauss-Seidel resolve cada junta sozinho. Numa cadeia pesada as linhas de bloqueio precisam de muitas iterações para concordarem e a passagem de posição, a 0.2 do erro por iteração, deixa a maior parte da deriva que a integração cria. Medido (60 Hz, 8 iterações de velocidade, 3 de posição, Release):

| | cadeia livre de 3 elos (abre) | braço de escavadora 1.5/0.7/0.65 t a mexer (parado) | custo/passo (cadeia; braço) |
|---|---|---|---|
| iterativo, como estava | 1.83e-2 m | 6.54e-3 m (2.2e-4) | 11–16 µs; 18–19 µs |
| 16 / 32 / 64 iterações de velocidade | 1.83e-2 (igual) | 2.9e-3 / 1.3e-3 | — |
| 10 / 30 iterações de posição | 3.9e-3 / 7e-4 | — | — |
| 2 / 4 / 8 subpassos do mundo | 5.5e-3 / 1.6e-3 / 4.3e-4 | 4.5e-4 (4) | 25 / 44–64 / 106–124 µs |
| **articulado (escolhido)** | **1.4e-6 m** (120 Hz: 1.2e-6) | **2.4e-6 m (1.5e-6)** | 18–25 µs; 21 µs |

Mais iterações de velocidade não mudam nada na cadeia livre (o erro é de posição); subpassos fecham-no 4× por 4× o custo. A solução direta fecha-o ~10 000× por 1.2–1.7× o custo, só nas juntas que o pedem (`Joint::setArticulated(true)`); as outras não mudam (o resto da suíte dá os mesmos números, e as cenas do testbed os mesmos resultados).

O que faz, por grupo de juntas articuladas ligadas por corpos dinâmicos (`ArticulationSolver.h` tem o pormenor):

- **Velocidade**: as linhas de bloqueio de todas as juntas do grupo num sistema `K = J M⁻¹ Jᵀ` fatorizado uma vez por passo (LDLᵀ denso no envelope de cada linha, as juntas ordenadas das folhas para o centro — o corpo onde mais juntas se encontram — para encher pouco; linhas dependentes, como as de uma malha fechada, são detetadas no pivô e largadas). Cada iteração resolve-as exatamente. As linhas limitadas (motores, servos, batentes, cilindros, o cabo, e os contactos do grupo com o chão) são resolvidas no espaço que os bloqueios deixam: cada uma leva os impulsos de bloqueio `−K⁻¹c` que mantêm os bloqueios onde estão, e entre si por Gauss-Seidel projetado no seu acoplamento `A_ij = J_i · r_j` (até 12 varrimentos ou 1e-4), cada junta com a sua regra e os seus limites. Se nada tocou nos corpos do grupo desde a última iteração (nenhum contacto ou junta iterativa), a iteração é saltada — já não daria nada.
- **Posição**: uma projeção de Newton em todos os bloqueios e nas linhas que a junta quer seguras (servos rígidos, limites passados, batentes, o cabo esticado, contactos com o chão), na primeira passagem de posição; na última, a mesma fatorização com os erros da nova pose (quasi-Newton) se as linhas são as mesmas (Jacobiano a 2%; uma que desapareceu fica, segura onde está; uma nova obriga a fatorizar).
- **A inércia de fora nos ganhos**: um servo de frequência num eixo articulado é escalado por `1/(K⁻¹)_mm` do sistema com todas as outras linhas — a massa do espaço das juntas, com tudo o que está para fora a mover-se com ele. Um servo a 1 Hz com um elo de fora: período 1.0028 s articulado, 2.8268 s iterativo (`√(I_c/I_próprio)` = 2.828).
- **Contactos absorvidos**: um corpo do grupo contra o chão (estático/cinemático) ou contra outro do mesmo grupo passa a ser linhas limitadas do grupo (normal + atrito, `Contact::articulated`), para que as sapatas de uma grua não lutem com a lança.
- **Juntas suportadas**: `HingeJoint`, `SliderJoint`, `FixedJoint`, `PointJoint`, `DistanceJoint`, `HydraulicCylinder`, `Rope` (linhas de N corpos — até 4 — para o caminho por roldanas em corpos móveis, com o tambor), `Hook::setArticulated`. Um grupo com mais de 192 linhas volta ao iterativo (uma corrente de 40 elos testa-o).

### `HydraulicSystem`: bombas partilhadas

Um motor diesel (`Engine`, regulador de todas as velocidades com banda de 80 rpm, 4 subpassos por passo) acciona bombas (cilindrada × rpm × ηv; binário `D_ef p / (2π ηm)` devolvido ao motor). Cada bomba alimenta secções de válvula (caudal nominal, banda morta, atraso da gaveta, alívio de porta, prioridade), cada secção um ou mais consumidores (cilindros, motores hidráulicos, com sinal).

- **Load sensing**: a bomba fica `margem` acima da carga mais alta (stand-by sem nada a trabalhar), limitada por `maxPressure` (corte). Cada secção recebe o caudal da sua gaveta, qualquer que seja a carga (o compensador). Quando a soma passa o que a bomba dá: **partilha de caudal** — todas reduzidas na mesma proporção, por níveis de prioridade (a prioridade mais alta servida primeiro). Limite de potência `p Q ≤ P`, com o prato a seguir o regulador com atraso (0.08 s: sem ele o caudal limitado oscilava). **Anti-stall**: com o motor puxado mais de `antiStallDroop` abaixo do regulador, o limite de potência cai em proporção (a zero ao dobro) — a 900 rpm o motor tem menos binário do que os limites, calculados à velocidade nominal, pedem.
- **Centro aberto**: bomba de cilindrada fixa, a galeria de centro aberto em série (fluxo `Π(1−|u|)`) e cada gaveta como orifício `Q = K u √Δp`; a pressão resolvida por bisseção. A carga leve leva o caudal todo, a pesada fica parada (o comportamento real que o *load sensing* corrige).
- Um consumidor contra o batente não leva caudal (corte), um no alívio leva o que passa pelo alívio.

Os cilindros ganharam `setFlow()` (comandado por caudal), `setPressureLimit()`, `atStrokeEnd()` e `count` (cilindros iguais em paralelo como um só — os dois da lança: dois rígidos em paralelo deixariam ao solver qual leva a carga). `HydraulicMotor`: cilindrada, ηv, ηm, redutor, raio de saída, alívio, travão; velocidade `Q ηv / (D i)`, binário `Δp D i ηm / 2π`, numa `JointDrive`.

### `Rope`: roldanas com raio e atrito

`addSheave(corpo, ponto, raio, rendimento)`. O caminho é a geometria dos pontos de tangência (cada troço tangente às duas roldanas, o arco enrolado `r φ` somado ao comprimento) e cada roldana divide a tensão por `η` no sentido em que o cabo corre (`η^−tanh(s/0.02)`, suave na inversão): o Jacobiano da força deixa de ser o da velocidade (linha assimétrica). Com mais de um cabo no moitão a última roldana é um ponto (a geometria de tangentes pesada pelos cabos deixava 2 mm de folga). `hookLoad()` lê a tensão do último troço. Defaults (raio 0, η = 1) dão o cabo de antes.

### Lagartas: `TrackedUndercarriage`

Duas correias contínuas sob o casco, sem elos nem rodas: uma `Joint` no casco com um grau de liberdade por lagarta (a velocidade da correia), amostrada ao longo do ramo de baixo. Por amostra: a pressão de Bekker `p = (kc/b + kφ) zⁿ` como uma linha mole implícita (linearizada em z, com amortecimento do solo); o deslocamento de corte j (2D, ao longo e de través) advectado com a correia (semi-Lagrangiano: `j = i x` em deslizamento permanente) e a tensão de Janosi-Hanamoto `τ = (c + σ tanφ)(1 − e^−|j|/K)` (rigidez secante para o j previsto, limitada a `c + σ tanφ`); a resistência de compactação de Bekker por lagarta e a resistência interna na correia. Integrado dá o `H(i)` da literatura. Direção por diferença de velocidades (*skid steer*), rotação no lugar com velocidades opostas. Presets de solo: areia seca, franco (*loam*), argila, argila pesada, neve, superfície dura (Coulomb). Cada correia é uma `JointDrive` (o motor de translação hidrostático liga-se a ela).

### Corte do solo: `SoilCuttingTool`

A borda de uma ferramenta (os dentes do balde, a lâmina) num `SoilBed`: força de McKyes `F = w (γ d² Nγ + c d Nc) + w ρ v² d`, com o plano de rotura β que minimiza F (procurado numericamente); para uma ferramenta vertical lisa dá o Rankine passivo exato. Aplicada como um elemento de Coulomb no meio da borda (velocidade levada a zero com no máximo F): empurrada com menos, para; com mais, corta e o resto acelera o braço. Não modelado: o solo cortado sai do leito (o leito fica plano), a carga no balde, a sobrecarga do monte à frente da lâmina.

### As máquinas

- **`Excavator`** (20 t): lagartas, superestrutura a rodar numa dobradiça com motor hidráulico (90 cc, i = 170), lança/braço/balde em dobradiças com cilindros (os dois da lança como um, `count = 2`), o balde pela ligação de quatro barras (balanceiro e biela H; geometria procurada para Grashof, 168° de rotação, ângulo de transmissão ≥ 33.5°), tudo articulado. Duas bombas LS de 120 cc com 45 kW cada num diesel de 110 kW: lança, balde e lagarta esquerda numa, braço, rotação (prioridade) e lagarta direita na outra. `ExcavatorKinematics`: comprimentos dos cilindros ↔ ângulos, fecho das quatro barras, cinemática direta e inversa (cotovelo para cima). Assistente: alvo para os dentes (ponto + atitude) → comprimentos → gavetas (com compensação da banda morta); alvo de rotação.
- **`MobileCrane`** (50 t, todo-o-terreno): o chassis (a caixa vai ao chão: rodados dobrados nela), quatro estabilizadores (viga a sair e macaco a descer, servos em deslizantes; o macaco como mola de 2e7 N/m, a flexão da viga, para os quatro partilharem a carga pela compatibilidade; a sapata é uma esfera — um ponto de contacto, como numa rótula), superestrutura a rodar (motor hidráulico 60 cc, i = 830), lança de 11 m com três telescópicos (até 36.5 m) a luffar num cilindro, guincho de 140 kN com 4 cabos sobre a roldana da cabeça (raio 0.35 m, η = 0.98), moitão. A tabela de carga é calculada da estabilidade (75% do basculamento sobre a linha das sapatas, limite estrutural de 3e5 kg·m e 50 t). O **LMI** lê a carga do cabo (filtro de 1 s, histerese de 3%) e corta o que aumenta o momento (içar, baixar a lança, sair o telescópio), deixando o resto; o anti-two-block corta o içar a 1.5 m da cabeça. O moitão dorme só quase parado (o limiar do mundo pô-lo a dormir no fim de um balanço lento).
- **`Bulldozer`** (18 t): lagartas hidrostáticas (uma bomba LS e um motor por lagarta, 1 m/s), quadro de empurrar em munhões com dois cilindros de elevação (como um) e a lâmina numa dobradiça longitudinal com um cilindro de inclinação; a borda da lâmina é uma `SoilCuttingTool` (ângulo de corte 55°); `bladeForce()` soma os contactos e o corte.
- **Pêndulo e anti-sway**: o período do moitão com a carga é o do pêndulo composto em torno da roldana; `InputShaper` (ZV: metade do comando agora, metade meio período depois, `T = 2π√(L/g)`) no comando de rotação tira o balanço residual.

### Números (medidos pelos testes)

Articulação: ver a tabela; quatro barras (quatro dobradiças e com biela) fecham a 5e-7 m e seguem a analítica a 1e-6 m; sensores exatos (490.500 N). Hidráulica: LS 60.00/60.00 L/min com cargas de 0.62 e 6.94 MPa, velocidades = Q/A exatas, pressão = carga + margem; partilha 82.01/82.01/41.01 de 205.03 L/min (e 100/70.02/35.01 com prioridade); centro aberto: leve 118 L/min, pesada 0; alívio a 20.000 MPa (LS desce o prato, centro aberto descarrega tudo); motor a 560 N·m baixa a 1863 rpm onde a curva dá 560.0, a 780 N·m cala-se; LS limitado a 60.0 kW estável (133.43 L/min sem oscilar); anti-stall: sem ele o motor cala-se a arrancar das 900 rpm, com ele sobe a 2004 rpm com 90.0 kW; motor hidráulico 0.49742 rad/s e 42 972 N·m exatos. Cabo: enrolamentos exatos a 1e-6 m, Atwood com η = 0.9 a = −1.4611 (exato), razão de tensões 0.9000; guincho por duas roldanas de 0.95: 19 620 / 21 740 (W/η²) / 17 707 N (W η²). Lagartas: afundamento de Bekker +0.00% nos cinco solos; tração contra Janosi-Hanamoto +0.02…+0.49%, satura a 115 857 de 116 186 N; rotação no lugar −101° em 8 s sem deriva; 8° de argila sobe, 15° sobe com deslizamento 3%, 22° (limite 18.1°) escorrega. Escavadora: quatro barras contra a forma fechada 1e-5 rad em 167.5° de balde; alvos alcançados a 3–8 mm; profundidade 4.06 m, alcance ao nível 9.22 m, altura 8.13 m; rotação com 1 t a 6 m a 1.079 rad/s, inclinação 0.49°; translação 1.514 m/s (5.4 km/h); em argila pesada a 0.40 m os dentes fazem 37.6 kN de 38.4 kN de limite e o cilindro do braço 158 kN (−15 kN no ar). Grua: os quatro macacos levam 676 860 N de 676 890; carga nominal (22.7 t a 11.37 m) estável, inclinação ≤ 0.45°, macacos = grua + carga a 0.2%; 1.73× sem LMI tomba (65°); o LMI corta a 10.54 m onde a tabela dá 25 815 kg para 25 864 kg (1.002), a lança para (0.0007 m/s), içar não mexe, subir a lança funciona; pêndulo +0.29% de `2π√(L/g)` (L = 5.33 m) e a 0.03% do composto; anti-sway deixa 14% do balanço. Bulldozer: velocidade dos cilindros = Q/A (0.2021/0.1315 m/s), deriva parada 2e-7 m; empurra 4.5 t de caixas 5.16 m, força na lâmina ≤ 67.7 kN (μW = 127 kN), cilindros ≤ 46.6 kN de 475 kN; em argila pesada a 0.11 m a borda faz 64 976 N do modelo (64 957 N). Estabilidade: as três máquinas 60 s com todas as alavancas a ciclar, a 60 e a 120 Hz, finitas, juntas ≤ 4.6e-6 m.

Custo (Release, por passo): grua 0.36–0.40 ms (68 linhas de bloqueio, ~24 linhas limitadas, 4 sapatas), escavadora 0.14 ms, bulldozer 0.1 ms. Do primeiro solver para este a grua passou de ~1.9 ms: o envelope e a ordem das juntas, ganhos só quando um servo de frequência os pede, a fatorização de posição reutilizada, o acoplamento montado corpo a corpo, iterações saltadas quando nada mudou, e as sapatas em esfera (48 → 12 linhas de contacto).

**Testes** (`tests/3d_machinery_*.cpp`, `tests/machinery_rig.h`, `machinery_crane_rig.h`, `machinery_stability.h`): `articulation`, `hydraulics`, `rope`, `tracks`, `excavator`, `crane`, `crane_lmi`, `crane_swing`, `crane_sway`, `bulldozer`, e `stability_{excavator,crane,bulldozer}_{60,120}`; cada um < 25 s com ASan, menos dois: a estabilidade a 120 Hz da grua (44 s) e do bulldozer (32 s) — um minuto a 120 Hz são 7200 passos.

**Testbed** (`testbed/3d/demo_machinery_3d.cpp`): `3D demo: excavator` (um ciclo: dentes no poço de argila pesada, arrasta e enrola contra o corte, levanta, roda 90°, abre o balde — 0.40 m de fundo, 38.6 kN de corte, pinos ≤ 3e-6 m), `3D demo: crane` (prende 5 t, iça 1 m, roda 35° pelo *input shaper*: balanço ≤ 5.1°, LMI lê 5510 kg de 5500, macacos a 0.01%), `3D demo: bulldozer` (empurra 18 caixas 5 m, força ≤ limite das lagartas, levanta a lâmina).

### O que mudou fora de `machinery/`

- `Joint`: `setArticulated()`/`articulated()`/`supportsArticulation()`/`solvedByArticulation()` e as virtuais da articulação (linhas de bloqueio, motores, linhas de posição, linhas largas de N corpos); `JointRow` passou de `JointAxis.h` para `Joint.h` (alias mantido). Juntas não articuladas: o mesmo de antes.
- `JointDrive::prepare` foi partido em `configure()` + `setInverseMass(inv, ganho)` (mesmo comportamento) para a articulação poder dar a massa do grupo.
- `ContactSolver`: a preparação da articulação depois da dos contactos; `Contact::articulated`; as juntas iterativas são as ativas e não articuladas. `PhysicsWorld::solverSettings()` e `articulation()`.
- `HydraulicCylinder`: `setFlow`, `setPressureLimit`, `atStrokeEnd`, `count`, `barrelPinWorld`/`rodPinWorld`. `Rope`: raio e rendimento das roldanas, `segmentTension(i)`; `hookLoad()` é agora a tensão do último troço (igual sem atrito). `Hook::setArticulated`.

**Limites conhecidos.** O solo cortado não sai do leito nem se acumula à frente da lâmina (sem sobrecarga: a resistência real de um bulldozer a empurrar terra é maior), nem enche o balde. As lagartas não têm elos (o ramo de cima e a catenária não existem), o solo não guarda marcas (não há multipassagem) e um chão dinâmico sente a reação um passo depois. O cabo não tem massa nem catenária. Sem pá carregadora (`WheelLoader`): as peças estão todas (`HydraulicSystem`, quatro barras como a do balde, pneus de `ground/`), ficou de fora por tempo. Um grupo articulado com mais de 192 linhas volta ao iterativo. A ordenação das juntas é uma heurística (folhas → centro), não um grau mínimo verdadeiro. O LMI não modela a deflexão da lança nem o vento.

## Helicópteros: rotor por elementos de pá, turbina livre, transmissão e um helicóptero ligeiro

Módulo novo em `3d/include/rotorcraft/` + `3d/src/rotorcraft/` (umbrella `rotorcraft/Rotorcraft.h`), uma pasta própria como `marine/`, `ground/` e `machinery/`: é um domínio de veículo inteiro (rotor, motor, transmissão, montagem, preset, piloto automático) que se apoia no módulo aerodinâmico (`Atmosphere`, `Airfoil`, `Wing`, `DragElement`) sem lhe mexer. Nada fora desta pasta mudou além do registo dos ficheiros em `3d/CMakeLists.txt` e `testbed/CMakeLists.txt`; o `Propeller` e os seus testes ficaram como estavam (o rotor precisa de voo de bordo de ataque, azimutes, cíclico e batimento — é um modelo à parte, não uma extensão do BEMT axial).

### `Rotor`

Elementos de pá numa grelha polar (por defeito 10 estações radiais × 12 azimutes; 2..24 × 4..32), da raiz (`rootCutout`, nunca dentro da articulação) à ponta, corda linear (afilamento), torção linear com o coletivo definido a 0.75 R. Azimute ψ medido a partir de trás no sentido da rotação, por isso ψ = 90° é sempre o lado que avança — à direita com rotação anti-horária vista de cima (`rotation = +1`), à esquerda com a outra. Todas as fórmulas são escritas nesse referencial e avaliadas com os vetores dele em espaço do corpo, por isso os dois sentidos de rotação são o mesmo código (o teste `rotationMirrors` confirma que os dois rotores são imagens espelhadas exatas).

- **Elemento de pá.** Com batimento β(ψ) e normal da pá batida n: `U_T = (Ω + guinada) r + V·e_t`, `U_P = V·n + (ω × r e_r)·n + v_i cos β + (r − e) dβ/dt`. Passo `θ0 + torção (r/R − 0.75) − A1 cos ψ − B1 sin ψ` (prato oscilante); `Airfoil` em `θ − atan2(U_P, U_T)` (o círculo inteiro: o escoamento invertido do lado que recua é só um ângulo perto de 180°); a força normal e a força no plano somam a força no cubo (inclinada com as pás, ou seja com o plano das pontas — é por aí que o cíclico atua), o binário no veio e o momento de batimento na articulação. O termo `(ω × r e_r)·n` é o amortecimento de arfagem e rolamento que o rotor dá à fuselagem.
- **Escoamento induzido.** Cada anel tem a sua velocidade induzida v(r), uniforme em azimute, do balanço de quantidade de movimento de Glauert com perda de ponta de Prandtl, `dT = 4π ρ F r v √(V_xy² + (V_z + v)²) dr` — em pairar é o BEMT do hélice, depressa é o rotor de Glauert. Por cima, o primeiro harmónico estático de Pitt-Peters, `v0 kx (r/R) cos(ψ − ψ_esteira)`, `kx = (15π/32) tan(χ/2)` com χ o ângulo de enviesamento da esteira (mais escoamento atrás, menos — até ascendente — à frente). A raiz de cada anel sai de Newton com a derivada analítica do lado do elemento em regime colado (que sobrestima a inclinação em perda, o que só torna o passo conservador).
- **Descida e estado de anel de vórtice.** Entre o helicóptero e o moinho a teoria de quantidade de movimento não tem solução física. O caudal mássico ganha um termo empírico `(0.37 v)²`, que entra com a razão de descida (de 0.2 a 1 v_h) e sai com a velocidade no plano (0.5 a 1.5 v_h). Acima de `1/√8` a equação do anel é monótona em v para qualquer descida, por isso a raiz é única e contínua desde a subida até ao travão de moinho, e a autorrotação ideal (`V_z + v = 0`) cai a ~1.65 v_h (medido: 1.7–1.8 v_h). A bandeira `vortexRing` acende para descidas entre 0.5 e 2 v_h (v_h = √(T/2ρA) com a tração atual) com menos de 0.9 v_h no plano. É uma indicação, não a perda de tração do VRS, que não é modelada.
- **Efeito de solo.** Cheeseman-Bennett, `T/T∞ = 1/(1 − x)` a potência constante, `x = (R/4z)² / (1 + (V_xy/v_i)²)`. Reproduzido escalando a velocidade induzida que as pás veem, para a mesma tração, por `(1 − x)^{3/2}`: para um rotor ideal (P ∝ T^1.5) é exatamente o rácio a potência constante (derivação no header). z limitado a 0.4 R.
- **Batimento.** Pá rígida articulada a e com mola K_β; I_β e S_β da pá uniforme de `bladeMass` sobre a articulação. A equação `I_β β'' + ((I_β + e S_β) Ω² + K_β) β = M_aero − 2Ω (I_β + e S_β) (ω × e_t)·z` (com o forçamento giroscópico das taxas da fuselagem) é resolvida para o primeiro harmónico permanente por balanço harmónico contra o momento aerodinâmico integrado numericamente, com Newton e o Jacobiano clássico de pairar (o amortecimento de Lock acopla os dois harmónicos cíclicos a 90°). Momento no cubo = `(N_b/2)(e S_β Ω² + K_β)` × inclinação do plano das pontas; o resto vem da tração inclinada. Número de Lock `γ = ρ a c R⁴ / I_β`, frequência de batimento `ν_β² = 1 + e S_β/I_β + K_β/(I_β Ω²)`.
- **Porquê quase-estático com atraso, não batimento dinâmico por pá.** Resolver o 1/rev exige um passo de rotor bem abaixo de um décimo de volta (~2 ms a 400 rpm): uma dúzia de subpassos a 60 Hz para um modo cujo único efeito visível é um atraso. Em `advance()` o plano das pontas aproxima-se da solução quase-estática com a constante de tempo `16/(γΩ)` do modo de batimento regressivo (modelo de rotor de primeira ordem), integrada exatamente (`1 − e^{−dt/τ}` partido pelas varreduras de Newton) — estável para qualquer passo; o teste `flappingLag` mede 60% do caminho ao fim de τ e nenhuma ultrapassagem com passos de 0.1 s. `solve()` converge tudo (testes, trim). Por passo: duas varreduras de Newton (escoamento e batimento) mais a das cargas, ~360 avaliações de perfil no rotor principal.

Não modelado: avanço-recuo, elasticidade, acoplamento passo-batimento (δ3), compressibilidade (a ponta que avança anda perto de Mach 0.8 em cruzeiro, falta o aumento de arrasto), perda dinâmica, atraso do escoamento induzido dinâmico, peso da pá contra a conicidade (g/(Ω²R) ~ 0.3%), termos de aceleração angular na equação de batimento. O momento giroscópico de um disco rígido não é aplicado à fuselagem — um rotor articulado só transmite o momento no cubo acima.

### `TurboshaftEngine`

Gerador de gás (Ng) e turbina livre (Np) sem veio comum. Ng segue a procura de combustível com um atraso de primeira ordem (`spoolTimeConstant` a 100%, dividido por Ng — mais lento em baixo), integrado exatamente. Potência de gás `P = P_max σ^0.8 ((Ng − Ng0)/(1 − Ng0))²` (lapso empírico com a densidade); linha da turbina livre `Q = (P/ω0)(2 − Np/Np0)` (duas vezes o binário nominal parada, a potência toda à velocidade nominal, nada ao dobro) menos ventilação. Regulador: PI no erro de Np mais antecipação de coletivo (a posição da alavanca entra direto na procura — a compensação de queda de rotação de qualquer regulador de helicóptero), com anti-windup; o punho (`throttle`) limita a procura (0 = ralenti, 1 = voo). Consumo `idle + sfc × P`. `running` desligado corta o combustível e Ng cai com `spoolDownTimeConstant`.

### `RotorTransmission`

Rotor principal e de cauda são um sistema rígido (`ω_cauda = tailRotorRatio ω`), inércia referida ao veio principal `I_p + r² I_c`, carga = binários aerodinâmicos + atrito. O motor liga-se pela roda livre a `engineRatio = Np/Nr`: engatado, os dois lados aceleram juntos e o binário no sprag é o da turbina menos o que acelera a própria turbina — se sair negativo (o rotor teria de arrastar o motor) a roda livre solta; solta, cada lado roda com os seus binários; volta a engatar quando a turbina apanha o rotor, com equalização inelástica que conserva o momento angular. Por isso o motor puxa o rotor mas nunca é puxado: motor parado, o rotor fica com a velocidade que o ar a subir pelo disco lhe dá — autorrotação. Cada lado é integrado linearmente implícito no declive dos seus binários.

### `Helicopter`

Como o `Aircraft`, não é dono do `RigidBody`; tem rotor principal e de cauda (`Rotor` também — o de cauda vê o voo de bordo de ataque como o principal), motor, transmissão, asas (`Wing`: estabilizador e deriva), `DragElement`s, contactos de esqui, controlos (`HelicopterControls`: coletivo 0..1, cíclico longitudinal/lateral −1..1, pedais −1..1, punho, motor ligado), aparelhagem (`HelicopterRigging`: amplitudes de passo), SAS opcional (amortecimento de taxas com autoridade limitada), telemetria e `trim()`.

- **Rotor como forças na fuselagem, não um corpo numa junta.** Um rotor de 500 kg m² a 43 rad/s numa junta contra uma fuselagem de 1300 kg seria uma restrição rígida com acoplamento giroscópico no centro de cada passo, e continuaria a precisar de toda a aerodinâmica acima. A solução de batimento já traz o que o cubo transmite (tração inclinada, momento no cubo, amortecimento); a velocidade do rotor é estado interno da transmissão e a reação no veio sai exata do balanço de momento angular: `binário na fuselagem = −(Q_aero + I_rotor dω/dt)` em cada veio.
- **Esquis.** `SkidContact`: mola-amortecedor normal contra o solo e atrito de Coulomb com âncora (estática): um helicóptero pousado com o rotor a rodar fica parado em vez de deslizar. O solo é um plano (`setGroundPlane`), uma consulta do utilizador (`setGroundQuery`) ou um raio no `PhysicsWorld` (`setGround(world, corpo)`); a mesma sonda dá a altura do cubo ao solo (efeito de solo) e a altura dos esquis (telemetria).
- **Esteira do rotor na fuselagem.** Um ponto debaixo do disco, dentro da coluna da esteira inclinada pelo enviesamento, vê a velocidade induzida a crescer de v no disco a 2v um raio abaixo (`v (1 + z/√(R² + z²))`), com bordo suave. Dá a carga para baixo em pairar (+3.7% de tração) e o coice de nariz para cima do estabilizador na transição; em cruzeiro a esteira passa por trás.
- **Ordem por passo** (`applyForces`): mistura dos controlos (+SAS), sonda do solo, os dois rotores avançam um passo, a transmissão integra as velocidades, o motor avança regulador e Ng, e as cargas dos rotores, as reações nos veios, as superfícies, os elementos de arrasto e os esquis vão para o corpo. Contrato de tempo do `Aircraft` (chamar antes de cada `step()`, ou `stepCallback` mais uma chamada inicial; `stepCallbackTiming` dá trajetórias idênticas).
- **Trim.** Levenberg-Marquardt nos seis equilíbrios para voo reto estabilizado a uma velocidade e razão de subida: coletivo, dois cíclicos, pedais, atitude de arfagem e pranchamento, com os rotores resolvidos à rotação regulada (as asas iteradas até o ângulo induzido convergir — o estabilizador pode estar em perda dentro da esteira). Em caso de sucesso deixa o corpo, os controlos, a transmissão e o Ng do motor nesse estado.
- **`HelicopterAutopilot`** (para testes e testbed): cascata PI posição → velocidade no solo (limitada pela velocidade de que ainda se trava na distância que falta) → aceleração (integrador condicional para a atitude de trim, com descarga sempre permitida) → arfagem/pranchamento → cíclico com amortecimento de taxas; altura ou razão de subida → coletivo, que desce quando o rotor cai abaixo de 97% (pedir mais potência do que há); rumo → taxa de guinada limitada → pedais. Pode começar a partir de um trim (`engage(trim, rumo)`) — numa descolagem os esquis seguram a fuselagem nivelada e os controlos atuais não dizem nada sobre a atitude de pairar.
- **`buildLightHelicopter()`**: 1300 kg, rotor tripá articulado de 4.6 m (articulação a 3.9%, pás de 22 kg, γ = 5.5, ν_β = 1.03, 415 rpm, 200 m/s na ponta, veio inclinado 4° para a frente), rotor de cauda bipá basculante de 0.8 m do lado esquerdo do cone de cauda, 5.7 m atrás do mastro (5.75:1), turbina de 300 kW, estabilizador e deriva calçada, 1.05 m² de área plana equivalente, esquis.

### Números (medidos pelos testes)

Rotor principal em pairar com o peso (12.75 kN): velocidade induzida média 8.859 m/s contra 8.849 de `√(T/2ρA)` (+0.11%), figura de mérito 0.683, C_T 0.0039, conicidade 2.6°; tração ∝ ρ(ΩR)² exata a passo fixo; perda de ponta −1.1% de tração. Sustentação de translação: +16% a 10 m/s, +28% a 15 m/s a passo fixo; a 40 m/s a induzida bate com Glauert (1.96 m/s). Blowback a1s = 1.1, 2.1, 3.2, 4.4, 5.9° para μ = 0.05..0.25. Atraso do disco numa taxa de arfagem de 0.3 rad/s: 1.40° contra `16q/(γΩ)` = 1.15°. Efeito de solo a potência constante: z/R = 0.5 → 1.307 (Cheeseman-Bennett 1.333, −2.0%), 0.75 → 1.118 (−0.6%), 1 → 1.063 (−0.3%). Autorrotação axial (binário nulo) a 1.94 v_h. Helicóptero: pairar fora do efeito de solo com coletivo 0.59, pedais −0.18, 189 kW (rotor de cauda 694 N × 5.7 m = 3955 N m contra 3987 N m do principal), nariz +4°, lado esquerdo 2.6° em baixo; curva de potência com o fundo do balde a 27.5 m/s (107 kW, 57% do pairar; o Vy de um ligeiro é 50–55 kt) e 230 kW a 60 m/s; cruzeiro trimado a 50 m/s: coletivo 0.61, cíclico 0.50 / −0.24, pedais 0.31, nariz −1.6°, 166 kW. Regulador: degraus de coletivo de ±0.1 em pairar e a 40 m/s ficam entre 99.0% e 100.9% de Nr. Motor parado a 31 m/s, com o coletivo a segurar o rotor: Nr 100–101.4%, descida 8.2 m/s, turbina livre a 12% da velocidade do rotor (roda livre solta); a recuperação (flare) baixa a descida para 2.3 m/s e sobe o rotor a 107%. Piloto automático: pairar 30 s com erro máximo de 0.19 m (6 m, calma), 0.40 m (vento lateral de 5 m/s), 0.73 m (30 m), igual a 60 e 120 Hz; descolagem dos esquis e pouso a 0.34 m/s, 0.19 m do ponto de partida, nivelado.

### Testes (`tests/3d_heli_*.cpp`)

`rotor` (propriedades derivadas, teoria de quantidade de movimento, escala, perda de ponta, sustentação de translação, blowback, sinais do cíclico nos dois sentidos de rotação, espelho, amortecimento, efeito de solo, VRS, descida axial contínua, atraso do plano das pontas, entradas degeneradas), `engine` (potência vs Ng e altitude, linha da turbina livre, atraso do gerador de gás, corte, consumo, regulador com e sem antecipação, roda livre, reengate com conservação de momento angular), `performance` (trim de pairar e antitorque, balde de potência, trim de cruzeiro e subida, altitude e efeito de solo, carga da esteira, trims impossíveis recusados), `flight` (pousado nos esquis, pairar com piloto automático, descolagem e pouso, sinal de cada comando, cruzeiro de mãos livres com SAS após um dublete, fontes de solo, SAS, contrato do step callback), `autorotation` (regulador em degraus de coletivo, falha de motor, flare, reacendimento, arranque no heliponto) e `stability_60` / `stability_120` (três minutos a cada frequência: esquis com cíclico e pedais a fundo e a 30%, um minuto de comandos a fundo em malha aberta com corte de motor, e um minuto de manobras duras com o piloto automático). Cada um abaixo de 16 s sob ASan (o mais lento, `stability_120`, 15.6 s).

**Testbed.** `3D demo: helicopter` (`testbed/3d/demo_helicopter_3d.cpp`): heliponto num chão estático (o helicóptero sonda-o com raios no `PhysicsWorld`), descolagem, pairar a 5 m, aceleração a 16 m/s, volta e pouso no heliponto, tudo pelo piloto automático. Mostrado a 5× (cinco passos de 1/60 s por quadro — a física é a de 60 Hz; o voo tem ~45 s e o modo headless dá 10 a cada cena). As pás são desenhadas com a conicidade e a inclinação que a solução de batimento lhes dá. Veredictos: sai do chão, pairar dentro de um metro (0.40 m), transição (15.3 m/s), pousa nivelado no heliponto (0.75 m do centro), rotor regulado (94.8–103.5%), finito.

**Limites conhecidos.** Além dos do rotor acima: o solo dos esquis é tomado como parado (sem plataformas móveis), não há rolamento dinâmico nem ressonância de solo, o rotor de cauda não sente a esteira do principal nem o bloqueio da deriva, a fuselagem não tem momento de Munk nem sustentação própria, o motor não tem limites de temperatura nem limitador de aceleração além do atraso do gerador de gás, e não há anti-binário alternativo (NOTAR ou coaxial). Com cíclico e pedais a fundo no chão o helicóptero pode rodar sobre si (o pedal a fundo dá quase o binário que o atrito dos esquis segura) — não capota.
