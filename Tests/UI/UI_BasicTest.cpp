#include <gtest/gtest.h>

#include "ECS/EventBus.h"
#include "UI/UIManager.h"
#include "UI/Widgets/Button.h"
#include "UI/Widgets/Label.h"
#include "UI/Widgets/Slider.h"

using namespace Runtime::UI;

TEST(UIManagerTest, CanCreateCanvasAndCollectDrawCommands) {
    Runtime::ECS::EventBus eventBus;

    UIManager manager;
    ASSERT_TRUE(manager.Initialize(&eventBus));

    auto canvas = manager.CreateCanvas("MainCanvas", CanvasSpace::Screen, 10);
    ASSERT_NE(canvas, nullptr);

    auto label = std::make_shared<Label>("Title");
    label->SetText("MyGame UI");
    label->SetSize({240.0f, 32.0f});

    auto button = std::make_shared<Button>("PlayButton");
    button->SetPosition({0.0f, 40.0f});
    button->SetSize({160.0f, 36.0f});

    canvas->AddChild(label);
    canvas->AddChild(button);

    manager.Update(1.0f / 60.0f, Rect{{0.0f, 0.0f}, {1280.0f, 720.0f}});
    manager.Render();

    ASSERT_NE(manager.GetRenderer(), nullptr);
    const auto& stats = manager.GetRenderer()->GetStats();
    EXPECT_GT(stats.commandCount, 0u);
    EXPECT_GT(stats.textCount, 0u);

    manager.Shutdown();
}

TEST(UIInteractionTest, SliderConsumesPointerEventsAndUpdatesValue) {
    UIManager manager;
    ASSERT_TRUE(manager.Initialize());

    auto canvas = manager.CreateCanvas("MainCanvas", CanvasSpace::Screen, 0);
    auto slider = std::make_shared<Slider>("Volume");
    slider->SetPosition({100.0f, 100.0f});
    slider->SetSize({200.0f, 20.0f});
    slider->SetRange(0.0f, 100.0f);
    canvas->AddChild(slider);

    manager.Update(1.0f / 60.0f, Rect{{0.0f, 0.0f}, {1280.0f, 720.0f}});

    UIEvent down;
    down.type = UIEventType::PointerDown;
    down.screenPosition = {250.0f, 110.0f};
    EXPECT_TRUE(manager.RouteEvent(down));

    UIEvent up;
    up.type = UIEventType::PointerUp;
    up.screenPosition = {250.0f, 110.0f};
    manager.RouteEvent(up);

    EXPECT_GT(slider->GetValue(), 50.0f);

    manager.Shutdown();
}
