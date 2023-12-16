#include <chrono>
#include <fstream>
#include <optional>
#include <vector>

#include <json.hpp>
#include <opencv2/opencv.hpp>
#include <raylib-cpp.hpp>

#define RAYGUI_IMPLEMENTATION
#include <raygui.h>

namespace rl = raylib;

constexpr int checkers_size_width = 6;
constexpr int checkers_size_height = 8;

void write_camera_json(cv::Mat cam_mat, const std::vector<double>& dist_coef)
{
    assert(cam_mat.rows == 3 && cam_mat.cols == 3);
    assert(dist_coef.size() == 5);
    const nlohmann::json camera_json
        = { { "camera_matrix",
              { { cam_mat.at<double>(0, 0), cam_mat.at<double>(1, 0), cam_mat.at<double>(2, 0) },
                { cam_mat.at<double>(0, 1), cam_mat.at<double>(1, 1), cam_mat.at<double>(2, 1) },
                { cam_mat.at<double>(0, 2), cam_mat.at<double>(1, 2), cam_mat.at<double>(2, 2) } } },
            { "distortion_coefficients", { dist_coef[0], dist_coef[1], dist_coef[2], dist_coef[3], dist_coef[4] } } };
    std::ofstream json_out("camera.json");
    json_out << camera_json;
}

void process_calibration(
    const cv::Size size,
    const std::vector<std::vector<cv::Point3f>>& obj_points,
    const std::vector<std::vector<cv::Point2f>>& img_points)
{
    assert(!img_points.empty() && img_points.size() == obj_points.size());
    cv::Mat camera_matrix;
    std::vector<double> distortion_coefficients;
    std::vector<cv::Mat> rotation_vectors;
    std::vector<cv::Mat> translation_vectors;
    calibrateCamera(
        obj_points, img_points, size, camera_matrix, distortion_coefficients, rotation_vectors, translation_vectors);
    write_camera_json(camera_matrix, distortion_coefficients);
}

int main()
{
    const cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);

    std::vector<cv::Point3f> object_points_ref;
    for (int i = 0; i < checkers_size_height; i++)
        for (int j = 0; j < checkers_size_width; j++)
            object_points_ref.emplace_back(j, i, 0);

    std::vector<std::vector<cv::Point3f>> obj_points;
    std::vector<std::vector<cv::Point2f>> img_points;

    cv::VideoCapture video_capture(0);
    if (!video_capture.isOpened())
        return EXIT_FAILURE;

    rl::Window window(800, 600, "Camera Calibration");
    GuiSetStyle(DEFAULT, TEXT_SIZE, 20);

    std::optional<rl::Texture> rl_texture;
    std::optional<cv::Size> frame_size;

    int capture_count = 0;
    while (!window.ShouldClose()) {
        constexpr int ui_margin_top = 50;
        constexpr int ui_padding = 5;

        cv::Mat frame;
        video_capture >> frame;

        cv::Mat frame_gray;
        cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        if (!frame_size.has_value()) {
            frame_size = frame.size();
            window.SetSize(frame_size->width, frame_size->height + ui_margin_top);
        }

        bool found = false;
        std::vector<cv::Point2f> corners;
        if (findChessboardCorners(frame_gray, cv::Size(checkers_size_height, checkers_size_width), corners)) {
            found = true;
            cornerSubPix(frame_gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            drawChessboardCorners(frame, cv::Size(checkers_size_height, checkers_size_width), corners, true);
        }

        cv::Mat screen;
        cvtColor(frame, screen, cv::COLOR_BGR2RGB);
        if (rl_texture.has_value()) {
            rl_texture->Update(screen.data);
        }
        else {
            Image image;
            image.data = screen.ptr();
            image.width = screen.cols;
            image.height = screen.rows;
            image.mipmaps = 1;
            image.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8;
            rl_texture = rl::Texture(image);
        }

        window.BeginDrawing();
        window.ClearBackground(rl::Color::White());
        if (rl_texture.has_value()) {
            rl_texture->Draw(0, ui_margin_top);
        }
        int ui_offset = 0;
        if (GuiButton({ ui_padding, ui_padding, 100, ui_margin_top - ui_padding * 2 }, "Reset")) {
            img_points.clear();
            obj_points.clear();
            capture_count = 0;
        }
        ui_offset += 100 + ui_padding;
        if (GuiButton(
                { static_cast<float>(ui_offset + ui_padding), ui_padding, 100, ui_margin_top - ui_padding * 2 },
                "Capture")
            || IsKeyPressed(KEY_SPACE)) {
            if (found) {
                obj_points.push_back(object_points_ref);
                img_points.push_back(corners);
                capture_count++;
            }
        }
        ui_offset += 100 + ui_padding;
        GuiLabel(
            { static_cast<float>(ui_offset + ui_padding), ui_padding, 400, ui_margin_top - ui_padding * 2 },
            TextFormat("Captured: %d", capture_count));

        bool should_process = false;
        if (capture_count > 0) {
            if (GuiButton(
                    { static_cast<float>(GetScreenWidth() - (100 + ui_padding)),
                      ui_padding,
                      100,
                      ui_margin_top - ui_padding * 2 },
                    "Process")) {
                rl::DrawText("Processing...", ui_padding, ui_margin_top + ui_padding, 32, rl::Color::Green());
                should_process = true;
            }
        }
        window.EndDrawing();

        if (should_process) {
            process_calibration(frame_size.value(), obj_points, img_points);
        }
    }

    video_capture.release();
    cv::destroyAllWindows();

    return EXIT_SUCCESS;
}