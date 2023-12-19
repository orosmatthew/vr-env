#include <array>

#include <Eigen/Geometry>
#include <raylib-cpp.hpp>
#include <rlgl.h>

#include "networking.hpp"

#include <iostream>

namespace rl = raylib;

Matrix eig_matrix_to_rl_matrix(const Eigen::Matrix4f& eigenMatrix)
{
    Matrix raylibMatrix;

    // Convert Eigen::Matrix4f to raylib Matrix
    raylibMatrix.m0 = eigenMatrix(0, 0);
    raylibMatrix.m1 = eigenMatrix(0, 1);
    raylibMatrix.m2 = eigenMatrix(0, 2);
    raylibMatrix.m3 = eigenMatrix(0, 3);

    raylibMatrix.m4 = eigenMatrix(1, 0);
    raylibMatrix.m5 = eigenMatrix(1, 1);
    raylibMatrix.m6 = eigenMatrix(1, 2);
    raylibMatrix.m7 = eigenMatrix(1, 3);

    raylibMatrix.m8 = eigenMatrix(2, 0);
    raylibMatrix.m9 = eigenMatrix(2, 1);
    raylibMatrix.m10 = eigenMatrix(2, 2);
    raylibMatrix.m11 = eigenMatrix(2, 3);

    raylibMatrix.m12 = eigenMatrix(3, 0);
    raylibMatrix.m13 = eigenMatrix(3, 1);
    raylibMatrix.m14 = eigenMatrix(3, 2);
    raylibMatrix.m15 = eigenMatrix(3, 3);

    return raylibMatrix;
}

void pos_quat_to_cam_params(
    const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, Vector3& out_pos, Vector3& out_target, Vector3& out_up)
{
    constexpr float translate_scale = 10.0f;
    out_pos = { pos.x() * translate_scale, -pos.y() * translate_scale, -pos.z() * translate_scale };
    Eigen::Vector3f forward = quat * Eigen::Vector3f(0, 0, 1);
    const Eigen::AngleAxisf adj(PI, Eigen::Vector3f(1, 0, 0));
    forward = adj * forward;
    out_target = { out_pos.x + forward.x(), out_pos.y + forward.y(), out_pos.z + forward.z() };
    Eigen::Vector3f up = quat * Eigen::Vector3f(0, 1, 0);
    out_up = { up.x(), -up.y(), -up.z() };
}

// void pos_euler_to_cam_params(
//     const Eigen::Vector3f& pos, const Eigen::Vector3f& euler, Vector3& out_pos, Vector3& out_target, Vector3& out_up)
// {
//     constexpr float translate_scale = 10.0f;
//     out_pos = { pos.x() * translate_scale, -pos.y() * translate_scale, -pos.z() * translate_scale };
//
//     // Convert Euler angles (X, Y, Z) to Quaternion
//     const Eigen::Quaternionf quat = Eigen::AngleAxisf(euler[2], Eigen::Vector3f::UnitZ()) // Z
//         * Eigen::AngleAxisf(euler[1], Eigen::Vector3f::UnitY()) // Y
//         * Eigen::AngleAxisf(euler[0], Eigen::Vector3f::UnitX()); // X
//
//     // Eigen::Vector3f forward = quat * Eigen::Vector3f(0, 0, 1);
//     Eigen::Vector3f forward = Eigen::Vector3f(0, 0, 1);
//     // const Eigen::AngleAxisf adj(PI, Eigen::Vector3f(1, 0, 0));
//     // forward = adj * forward;
//     out_target = { out_pos.x + forward.x(), out_pos.y + forward.y(), out_pos.z + forward.z() };
//
//     Eigen::Vector3f up = quat * Eigen::Vector3f(0, 1, 0);
//     // Eigen::Vector3f up = Eigen::Vector3f(0, 1, 0);
//     out_up = { up.x(), up.y(), -up.z() };
// }
//
// Eigen::Matrix4f pos_quat_to_matrix(const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat)
// {
//     Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
//
//     // Construct rotation matrix from quaternion
//     Eigen::Matrix3f rotation_matrix = quat.toRotationMatrix();
//
//     // Set the rotation part of the transformation matrix with adjusted axes
//     transformation.block<3, 3>(0, 0) = rotation_matrix;
//
//     // Apply adjustments similar to the original function
//     Eigen::Vector3f adjusted_pos = Eigen::Vector3f(pos.x(), -pos.y(), -pos.z());
//     Eigen::Vector3f forward = rotation_matrix * Eigen::Vector3f(0, 0, -1);
//     Eigen::Vector3f adjusted_target = adjusted_pos + forward;
//     Eigen::Vector3f adjusted_up = rotation_matrix * Eigen::Vector3f(0, -1, 0);
//
//     // Set the translation part of the transformation matrix
//     transformation.block<3, 1>(0, 3) = -rotation_matrix * adjusted_pos;
//
//     // Update the transformation matrix with adjusted target and up vectors
//     transformation(0, 2) = adjusted_target.x();
//     transformation(1, 2) = adjusted_target.y();
//     transformation(2, 2) = adjusted_target.z();
//
//     transformation(0, 1) = adjusted_up.x();
//     transformation(1, 1) = adjusted_up.y();
//     transformation(2, 1) = adjusted_up.z();
//
//     return transformation;
// }
//
// void transform_to_cam_params(const Eigen::Matrix4f& transform, Vector3& out_pos, Vector3& out_target, Vector3&
// out_up)
// {
//     Eigen::Vector3f position = transform.block<3, 1>(0, 3);
//     out_pos = { position.x(), position.y(), position.z() };
//     Eigen::Vector3f direction = transform.block<3, 3>(0, 0) * Eigen::Vector3f(0, 0, -1);
//     out_target = { out_pos.x + direction.x(), out_pos.y + direction.y(), out_pos.z + direction.z() };
//     Eigen::Vector3f up = transform.block<3, 3>(0, 0) * Eigen::Vector3f(0, 1, 0);
//     out_up = { up.x(), up.y(), up.z() };
// }
//
Eigen::Matrix4f pos_quat_to_transform(const Eigen::Vector3f& position, const Eigen::Quaternionf& rotation)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rotation.normalized().toRotationMatrix();
    transform.block<3, 1>(0, 3) = position;
    return transform;
}

// void pos_quat_to_cam_params(
//     const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, Vector3& out_pos, Vector3& out_target, Vector3&
//     out_up)
// {
//     // Flip the z and y coordinates of the position
//     out_pos = { pos.x(), -pos.y(), -pos.z() };
//
//     // Calculate the direction of the camera from the quaternion
//     Eigen::Vector3f direction = quat * Eigen::Vector3f(0, 0, 1);
//
//     // Apply a 180 degree rotation around the x-axis
//     Eigen::AngleAxisf adj(PI, Eigen::Vector3f(1, 0, 0));
//     direction = adj * direction;
//
//     // Calculate the target
//     out_target = { out_pos.x + direction.x(), out_pos.y + direction.y(), out_pos.z + direction.z() };
//
//     // Calculate the right vector as the cross product of the direction and the up vector
//     Eigen::Vector3f right = direction.cross(Eigen::Vector3f(0, 1, 0));
//
//     // Calculate the up vector as the cross product of the direction and the right vector
//     Eigen::Vector3f up = direction.cross(right);
//     out_up = { up.x(), -up.y(), -up.z() }; // Flip the y and z coordinates
// }

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
    camera.position = { 0.0f, 1.0f, 0.0f };
    camera.target = { 0.0f, 0.0f, 1.0f };
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

    // Mesh cube_mesh = GenMeshCube(1.0f, 1.0f, 1.0f);
    // Model cube_model = LoadModelFromMesh(cube_mesh);

    const ReceiverUdp receiver_udp;

    while (!window.ShouldClose()) {
        auto [bytes, n_bytes] = receiver_udp.get_data();
        int offset = 0;
        Eigen::Vector3f pos;
        Eigen::Vector4f rot;
        std::string buffer;
        if (n_bytes != 0) {
            for (int i = 0; i < n_bytes; ++i) {
                if (bytes[i] == ' ') {
                    switch (offset) {
                    case 0:
                        pos[0] = std::stof(buffer);
                        break;
                    case 1:
                        pos[1] = std::stof(buffer);
                        break;
                    case 2:
                        pos[2] = std::stof(buffer);
                        break;
                    case 3:
                        rot[0] = std::stof(buffer);
                        break;
                    case 4:
                        rot[1] = std::stof(buffer);
                        break;
                    case 5:
                        rot[2] = std::stof(buffer);
                        break;
                    case 6:
                        rot[3] = std::stof(buffer);
                        break;
                    default:
                        throw std::runtime_error("Unreachable");
                    }
                    buffer.clear();
                    offset++;
                }
                else {
                    buffer.push_back(bytes[i]);
                }
            }
            Eigen::Quaternionf quat(rot);
            // Eigen::Matrix4f transform = pos_quat_to_transform(pos, quat);
            // cube_model.transform = eig_matrix_to_rl_matrix(transform);
            quat.normalize();
            // camera = QuaternionToCamera({ pos.x(), pos.y(), pos.z() }, { quat.x(), quat.y(), quat.z(), quat.w() });
            pos_quat_to_cam_params(pos, quat, camera.position, camera.target, camera.up);
            // Eigen::Matrix4f transform;
            // transform.setIdentity();
            // transform.block<3, 3>(0, 0) = rot.cast<float>();
            // transform.block<3, 1>(0, 3) = pos;
            // std::cout << rot << std::endl;
            // cube_model.transform = eig_matrix_to_rl_matrix(transform);
            camera.position = Vector3Add(camera.position, { 0, 5, 0 });
            camera.target = Vector3Add(camera.target, { 0, 5, 0 });
        }

        // camera.Update(CAMERA_FIRST_PERSON);
        BeginTextureMode(vr_render_tex);
        window.ClearBackground(rl::Color::RayWhite());
        BeginVrStereoMode(vr_config);
        BeginMode3D(camera);
        // {
        //     rlDrawRenderBatchActive(); // Update and draw internal render batch
        //     rlMatrixMode(RL_PROJECTION); // Switch to projection matrix
        //     rlPushMatrix(); // Save previous matrix, which contains the settings for the 2d ortho projection
        //     rlLoadIdentity(); // Reset current matrix (projection)
        //     rlMatrixMode(RL_MODELVIEW); // Switch back to modelview matrix
        //     rlLoadIdentity(); // Reset current matrix (modelview)
        //     rlMultMatrixf(MatrixToFloat(view_mat)); // Multiply modelview matrix by view matrix (camera)
        //     rlEnableDepthTest(); // Enable DEPTH_TEST for 3D
        // }

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
        // DrawModel(cube_model, { 0.0f, 2.0f, 0.0f }, 1.0f, RED);

        EndMode3D();
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