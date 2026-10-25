# Regras do projeto (prisma3d: engine 3D sobre o prisma)

Alvo: o engine definitivo do utilizador. Cena por nós, servidor de render com handles, materiais PBR, luzes com sombras, reflexos, pós-processamento, física, interface, scripts e editor. C++17 (decidido pelo utilizador a 2026-10-08, para o `mathc`; o `ct` compila em C++14 e C++17). Desktop primeiro (OpenGL 4.6 e Vulkan); web e Android depois.

O plano está em `docs/ENGINE_PLAN.md`.

## Regra principal: NÃO inventar, reaproveitar
- A arquitetura de render e os shaders são **copiados do Godot** (`/media/projectos/projects/cpp/engines/godot`), não desenhados de memória nem reescritos. Num shader copiado mudam só as declarações de recursos; o corpo fica igual.
- Outras fontes aceites: os projetos do utilizador (Kinetix3d para o carregador em threads, Radion para godrays e importadores de animação) e as referências do prisma (`/media/projectos/projects/cpp/prisma/tmp/filament`, `tmp/Ogre-Next`).
- Para cada técnica: registar a origem (ficheiro e linhas) em `docs/PORTING.md`, nunca no código. Se não existir nas referências, dizer e perguntar.
- Portar fielmente: o algoritmo é o do original, verificado na linha; uma aproximação nunca leva o nome do original.

## Repositórios
- `prisma3d` (este): só a biblioteca (`libprisma3d`), os testes e as ferramentas.
- `prisma3d-demos` (https://github.com/akadjoker/prisma3d-demos): os demos e os `assets/`, com este repositório como submódulo em `external/prisma3d`. O `assets/NOTICE.md` e os avisos das fontes copiadas vivem lá.
- Os testes que leem assets só são construídos quando `PRISMA3D_ASSETS_DIR` aponta para uma pasta; sozinho, este repositório não os tem. No `prisma3d-demos` correm com `-DPRISMA3D_BUILD_TESTS=ON`.
- Um demo novo vai para o `prisma3d-demos`; nunca assets neste repositório.

## Submódulos (todos do utilizador, podem ser alterados quando for preciso)
- `external/prisma`: backend de render.
- `external/containers`: `ct`.
- `external/math`: `mathc` (`namespace Math`, column-major).
- `external/zen_plataform`: janela, input, ficheiros.
- `external/iGUI`: interface.
- `external/Kinetix`: física e AI. A cena liga-se a ela por uma ponte; a física não vive dentro da cena.
- `external/zenpy`: linguagem de scripts.
- Uma só cópia de `containers` e de `math` para todos: as de `external/`.

## Código: sem `std`, usar `ct`
- **Proibido** `std::vector`, `std::string`, `std::unordered_map`, `std::unique_ptr`, `std::function`, `std::thread`, etc. Usar `ct::Vector`, `ct::String`, `ct::HashMap`, `ct::SlotMap`, `ct::Arena`, `ct::Pool`, `ct::Function`, `ct::Rc/Unique`, `ct::Thread/ThreadPool`...
- Sem exceções; sem RTTI.
- Se faltar um contentor na `ct`, perguntar ao utilizador em vez de usar `std` ou inventar.
- **Sem comentários no código-fonte** por defeito; só quando o código não se explica sozinho.
- **Só escrever código quando o utilizador pedir.** Planos e documentos não incluem código novo.

## Regras de trabalho (para o Claude)
- **Devagar e em chunks pequenos.** Antes de escrever, explicar em poucas linhas o que o chunk faz e porquê. Escrever só esse chunk e parar para o utilizador rever e limpar.
- **Cada área fica completa** antes da seguinte: em todos os backends, sem partes deixadas para depois.
- **Não criar testes novos** (decidido pelo utilizador a 2026-10-16). A validação é visual: capturas de ecrã enviadas ao utilizador. Os testes que já existem continuam a correr.
- **Sem referências a outros engines** em código, comentários ou mensagens de commit. A origem fica só em `docs/PORTING.md`.
- **Um commit por chunk, sem coautoria.** Nenhuma linha `Co-Authored-By` nem menção ao Claude em commits ou PRs.
- **Commit implica push.** Se o push for recusado: `fetch`, `rebase`, voltar a testar, e só depois push. Nunca `--force`.
- **Poupar custos.** Trabalho mecânico é delegado a um subagente Sonnet; desenho, decisões e revisão ficam no modelo principal, que revê todo o código antes de entrar.
- **Medir em vez de supor.** Cada passo de render tem o seu custo de GPU medido e visível.
- **Respostas curtas.** Sem calendários, cenários ou perguntas repetidas que o utilizador não pediu.
- **Não abrir janelas sem pedir.** Os demos (no `prisma3d-demos`) só correm quando o utilizador pedir; o `ctest` não abre janelas e corre sempre inteiro.
- **Verificar pelo código de saída** de cada comando, não pelo que um `grep` devolve.
- **Formatar com `clang-format`** (o `.clang-format` do repositório: chaveta sempre em linha nova) antes de cada commit.
- **Decidir pelo mérito.** Este é o engine definitivo do utilizador; não se argumenta com o que os engines anteriores dele faziam. Licenças não se discutem na conversa: mantêm-se os avisos nos ficheiros copiados e a origem em `docs/PORTING.md`.
- README em inglês; a conversa é em português.

## Referências quando o PC do utilizador não está disponível
Os caminhos `/media/projectos/...` deste ficheiro só existem no PC do utilizador. Fora dele, as mesmas fontes vêm do GitHub; clonam-se para uma pasta fora do repositório e só se leem.

| Fonte | Onde |
|---|---|
| Godot | https://github.com/godotengine/godot, commit `e7b12e7492` (4.8-dev); os números de linha de `docs/` referem-se a este commit |
| Radion | https://github.com/akadjoker/Radion, ramo `main` |
| Kinetix3d | https://github.com/akadjoker/OriginalKinetix3d (privado), ramo `feature/material-slots` |
| Kinetix | https://github.com/akadjoker/Kinetix (privado), ramo `main`; nesta sessão clonado em `/home/user/kinetix` |
| prisma-samples | https://github.com/akadjoker/prisma-samples |
| Filament, Ogre-Next | https://github.com/google/filament, https://github.com/OGRECave/ogre-next |
| Ogre (terreno) | https://github.com/OGRECave/ogre, etiqueta `v14.5.2`; no PC do utilizador em `/media/projectos/projects/cpp/engines/Ogre/ogre-14.5.2` |
| WickedEngine (impostors) | https://github.com/turanszkij/WickedEngine; no PC do utilizador em `/media/projectos/projects/cpp/engines/WickedEngine` |

O estado do trabalho e os próximos chunks estão na secção "Estado" de `docs/ENGINE_PLAN.md`.
