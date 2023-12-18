#include <array>
#include <raylib-cpp.hpp>

namespace rl = raylib;

int main()
{
    constexpr int screenWidth = 1080 * 2;
    constexpr int screenHeight = 1080;

    rl::Window window { screenWidth, screenHeight, "VR Environment" };

    constexpr VrDeviceInfo vr_device {
        .hResolution = 1440 * 2, // pixels
        .vResolution = 1440, // pixels
        .hScreenSize = 0.054f * 2, // meters
        .vScreenSize = 0.054f, // meters
        .eyeToScreenDistance = 0.048f, // meters
        .lensSeparationDistance = 0.065f / 2, // meters
        .interpupillaryDistance = 0.065f, // meters
        .lensDistortionValues = { 1.0f, 0.01f, 0.01f, 0.0f },
        .chromaAbCorrection = { 1.0f, 0.0f, 1.0f, 0.0f }
    };

    const rl::VrStereoConfig vr_config { vr_device };

    rl::Shader vr_distortion_shader { nullptr, "res/distortion330.fs" };

    vr_distortion_shader.SetValue(
        vr_distortion_shader.GetLocation("leftLensCenter"), vr_config.leftLensCenter, SHADER_UNIFORM_VEC2);
    vr_distortion_shader.SetValue(
        vr_distortion_shader.GetLocation("rightLensCenter"), vr_config.rightLensCenter, SHADER_UNIFORM_VEC2);
    vr_distortion_shader.SetValue(
        vr_distortion_shader.GetLocation("leftScreenCenter"), vr_config.leftScreenCenter, SHADER_UNIFORM_VEC2);
    vr_distortion_shader.SetValue(
        vr_distortion_shader.GetLocation("rightScreenCenter"), vr_config.rightScreenCenter, SHADER_UNIFORM_VEC2);
    vr_distortion_shader.SetValue(vr_distortion_shader.GetLocation("scale"), vr_config.scale, SHADER_UNIFORM_VEC2);
    vr_distortion_shader.SetValue(vr_distortion_shader.GetLocation("scaleIn"), vr_config.scaleIn, SHADER_UNIFORM_VEC2);
    vr_distortion_shader.SetValue(
        vr_distortion_shader.GetLocation("deviceWarpParam"), vr_device.lensDistortionValues, SHADER_UNIFORM_VEC4);
    vr_distortion_shader.SetValue(
        vr_distortion_shader.GetLocation("chromaAbParam"), vr_device.chromaAbCorrection, SHADER_UNIFORM_VEC4);

    const rl::RenderTexture2D vr_render_tex { vr_device.hResolution, vr_device.vResolution };

    // The target's height is flipped (in the source Rectangle), due to OpenGL reasons
    const rl::Rectangle source_rect {
        0.0f, 0.0f, static_cast<float>(vr_render_tex.texture.width), -static_cast<float>(vr_render_tex.texture.height)
    };
    const rl::Rectangle dest_rect {
        0.0f, 0.0f, static_cast<float>(GetScreenWidth()), static_cast<float>(GetScreenHeight())
    };

    rl::Camera camera;
    camera.position = { 5.0f, 2.0f, 5.0f };
    camera.target = { 0.0f, 2.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    window.SetTargetFPS(90);

    constexpr int max_columns = 20;
    // Generates some random columns
    std::array<float, max_columns> heights { 0 };
    std::array<rl::Vector3, max_columns> positions { 0 };
    std::array<rl::Color, max_columns> colors { 0 };

    for (int i = 0; i < max_columns; ++i) {
        heights[i] = static_cast<float>(GetRandomValue(1, 12));
        positions[i] = rl::Vector3 {
            static_cast<float>(GetRandomValue(-15, 15)), heights[i] / 2.0f, static_cast<float>(GetRandomValue(-15, 15))
        };
        colors[i] = rl::Color { static_cast<unsigned char>(GetRandomValue(20, 255)),
                                static_cast<unsigned char>(GetRandomValue(10, 55)),
                                30,
                                255 };
    }

    while (!window.ShouldClose()) {
        BeginTextureMode(vr_render_tex);
        window.ClearBackground(rl::Color::RayWhite());
        BeginVrStereoMode(vr_config);
        camera.BeginMode();

        DrawPlane(rl::Vector3 { 0.0f, 0.0f, 0.0f }, rl::Vector2 { 32.0f, 32.0f }, rl::Color::LightGray());
        DrawCube(rl::Vector3 { -16.0f, 2.5f, 0.0f }, 1.0f, 5.0f, 32.0f, rl::Color::Blue());
        DrawCube(rl::Vector3 { 16.0f, 2.5f, 0.0f }, 1.0f, 5.0f, 32.0f, rl::Color::Lime());
        DrawCube(rl::Vector3 { 0.0f, 2.5f, 16.0f }, 32.0f, 5.0f, 1.0f, rl::Color::Gold());

        // Draw some cubes around
        for (int i = 0; i < max_columns; ++i) {
            DrawCube(positions[i], 2.0f, heights[i], 2.0f, colors[i]);
            DrawCubeWires(positions[i], 2.0f, heights[i], 2.0f, rl::Color::Maroon());
        }

        // DrawCube(cubePosition, 2.0f, 2.0f, 2.0f, RED);
        // DrawCubeWires(cubePosition, 2.0f, 2.0f, 2.0f, MAROON);
        // DrawGrid(40, 1.0f);

        camera.EndMode();
        EndVrStereoMode();
        EndTextureMode();

        window.BeginDrawing();
        window.ClearBackground(rl::Color::RayWhite());
        vr_distortion_shader.BeginMode();
        DrawTexturePro(
            vr_render_tex.texture, source_rect, dest_rect, rl::Vector2 { 0.0f, 0.0f }, 0.0f, rl::Color::White());
        vr_distortion_shader.EndMode();
        window.DrawFPS(10, 10);
        window.EndDrawing();
        //----------------------------------------------------------------------------------
    }

    return EXIT_SUCCESS;
}