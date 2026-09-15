#include "Core/GameApp.h"
#include <Core/Engine.h>
#include <Core/Input.h>

#include <fstream>
#include <memory>

namespace {

void WriteBootLog(const char* message) {
    std::ofstream log("mygame_boot.log", std::ios::out | std::ios::app);
    if (log.is_open()) {
        log << message << std::endl;
    }
}

} // namespace

int main() {
    WriteBootLog("main: start");
    auto& engine = Runtime::Core::Engine::GetEngine();
    auto game = std::make_unique<Game::Core::GameApp>();

    if (!engine.Initialize(1280, 720, "MyGame")) {
        WriteBootLog("main: engine.Initialize failed");
        return 1;
    }
    WriteBootLog("main: engine.Initialize ok");

    if (!game->Initialize(engine)) {
        WriteBootLog("main: game.Initialize failed");
        engine.Shutdown();
        return 2;
    }
    WriteBootLog("main: game.Initialize ok");

    auto& input = Runtime::Core::Input::Get();
    const Runtime::Core::Input::EventHandlerId escapeHandler = input.RegisterEventHandler(
        [&](const Runtime::Core::InputEvent& event) {
            if (event.type == Runtime::Core::InputEventType::KeyDown && event.key == Runtime::Core::Key_Escape) {
                engine.Exit();
            }
        }
    );

    WriteBootLog("main: entering run loop");
    engine.Run(
        [&](float dt) {
            game->Update(dt, input);
            if (game->ShouldExit()) {
                engine.Exit();
            }
        },
        [&]() {
            game->Render(engine);
        }
    );
    WriteBootLog("main: run loop exited");

    input.UnregisterEventHandler(escapeHandler);
    game->Shutdown();
    engine.Shutdown();
    WriteBootLog("main: shutdown complete");
    return 0;
}
