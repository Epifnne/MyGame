// Engine/include/Core/Engine.h
#pragma once

#include <cstdint>
#include <memory>
#include "Core/Input.h"
#include "Core/GameLoop.h"

namespace Runtime {
namespace Platform { class Window; }
namespace Graphics { class Renderer; }
namespace Core {

using GameInitFunc = void(*)();
using GameUpdateFunc = void(*)(float);
using GameRenderFunc = void(*)();
using GameShutdownFunc = void(*)();

class Engine {

public:
    static inline Engine& GetEngine() {
        static Engine instance;
        return instance;
    }

    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    bool Initialize(int width, int height, const char* title);

    //register game loop functions
    template<typename UpdateFn, typename RenderFn>
    void Run(UpdateFn&& update, RenderFn&& render) {
        if (!m_isInitialized) return;
        
        auto gameLoop = MakeGameLoop(
            std::forward<UpdateFn>(update),
            std::forward<RenderFn>(render),
            [this](float deltatime) { this->FixedUpdate(deltatime); }
        );
        
        gameLoop.Start();
        MainLoop(gameLoop);
    }

    template<typename UpdateFn, typename RenderFn, typename FixedUpdateFn>
    void Run(UpdateFn&& update, RenderFn&& render, FixedUpdateFn&& fixedUpdate) {
        if (!m_isInitialized) return;
        
        auto gameLoop = MakeGameLoop(
            std::forward<UpdateFn>(update),
            std::forward<RenderFn>(render),
            std::forward<FixedUpdateFn>(fixedUpdate)
        );
        
        gameLoop.Start();
        MainLoop(gameLoop);
    }

    //close the engine and clean up resources
    void Shutdown();

    //set callback functions for game lifecycle events
    void SetGameInit(GameInitFunc init) { m_gameInit = init; }
    void SetGameShutdown(GameShutdownFunc shutdown) { m_gameShutdown = shutdown; }

    void Exit() { m_isRunning = false; }

    bool IsRunning() const { return m_isRunning; }
    bool IsInitialized() const { return m_isInitialized; }

    Platform::Window* GetWindow() { return m_window; }
    Graphics::Renderer* GetRenderer() { return m_renderer; }

private:
    Engine() = default;
    ~Engine() = default;

    bool InitializeSubsystems(int width, int height, const char* title);
    void CleanupSubsystems();
    void FixedUpdate(float deltatime);

    template<typename GameLoopType>
    void MainLoop(GameLoopType& gameLoop) {
        auto& input = Input::Get();
        
        while (m_isRunning && gameLoop.GetState() != GameLoopState::Stopped) {
            if (m_window) {
                ProcessWindowEvents();
            }
            
            input.BeginFrame();
            gameLoop.Tick();
            input.UpdateMouseDelta();
        }
    }

    void ProcessWindowEvents();

    Platform::Window* m_window = nullptr;
    Graphics::Renderer* m_renderer = nullptr;
    
    // callbacks 
    GameInitFunc m_gameInit = nullptr;
    GameShutdownFunc m_gameShutdown = nullptr;
    
    // status
    bool m_isRunning = false;
    bool m_isInitialized = false;
    int m_width = 0;
    int m_height = 0;

};



}// namespace Core
}// namespace Runtime