#include "Core/GameApp.h"
#include <Core/Engine.h>
#include <Core/Input.h>

int main() {
    auto& engine = Runtime::Core::Engine::GetEngine();
    Game::Core::GameApp game;

    if (!engine.Initialize(1280, 720, "MyGame")) {
        return 1;
    }

    if (!game.Initialize(engine)) {
        engine.Shutdown();
        return 2;
    }

    engine.Run(
        [&](float dt) {
            game.Update(dt, Runtime::Core::Input::Get());
            if (game.ShouldExit()) {
                engine.Exit();
            }
        },
        [&]() {
            game.Render(engine);
        }
    );

    game.Shutdown();
    engine.Shutdown();
    return 0;
}
