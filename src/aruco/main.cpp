#include <filesystem>
#include <fstream>
#include <iostream>

#include <eigen3/Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <raylib-cpp.hpp>

using json = nlohmann::json;
namespace rl = raylib;

cv::Mat parse_camera_matrix(const json& camera_json)
{
    cv::Mat camera_matrix
        = (cv::Mat_<double>(3, 3) << camera_json["camera_matrix"][0][0],
           camera_json["camera_matrix"][1][0],
           camera_json["camera_matrix"][2][0],
           camera_json["camera_matrix"][0][1],
           camera_json["camera_matrix"][1][1],
           camera_json["camera_matrix"][2][1],
           camera_json["camera_matrix"][0][2],
           camera_json["camera_matrix"][1][2],
           camera_json["camera_matrix"][2][2]);
    return camera_matrix;
}

std::array<double, 5> parse_distortion_coefficients(const json& camera_json)
{
    return { camera_json["distortion_coefficients"][0],
             camera_json["distortion_coefficients"][1],
             camera_json["distortion_coefficients"][2],
             camera_json["distortion_coefficients"][3],
             camera_json["distortion_coefficients"][4] };
}

int main()
{
    if (!std::filesystem::exists("camera.json")) {
        std::cerr << "camera.json not found" << std::endl;
        return EXIT_FAILURE;
    }

    cv::Mat camera_matrix;
    std::array<double, 5> distortion_coefficients; // NOLINT(*-pro-type-member-init)
    {
        std::ifstream file("camera.json");
        const json camera_json = json::parse(file);
        camera_matrix = parse_camera_matrix(camera_json);
        distortion_coefficients = parse_distortion_coefficients(camera_json);
    }
    if (camera_matrix.cols != 3 || camera_matrix.rows != 3) {
        std::cerr << "Invalid camera.json" << std::endl;
        return EXIT_FAILURE;
    }

    cv::VideoCapture video_capture(0);
    if (!video_capture.isOpened()) {
        std::cerr << "Failed to open video capture" << std::endl;
        return EXIT_FAILURE;
    }

    rl::Window window(800, 600, "Aruco");

    std::optional<rl::Texture> rl_texture;
    std::optional<cv::Size> frame_size;

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::DetectorParameters aruco_parameters {};
    aruco_parameters.cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
    cv::aruco::Dictionary aruco_dict = getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::ArucoDetector aruco_detector { aruco_dict, aruco_parameters };

    // cv::Mat frame_undistorted;
    // ReSharper disable once CppTooWideScope
    cv::Mat camera_matrix_undistorted;
    cv::Mat frame;
    // cv::Mat frame_gray;
    cv::Mat screen;

    const std::vector<cv::Point3f> tracker_points {
        cv::Vec3f(-0.06435f, 0.02f, -0.0226f),  cv::Vec3f(-0.02971f, 0.02f, -0.0026f),
        cv::Vec3f(-0.02971f, -0.02f, -0.0026f), cv::Vec3f(-0.06435f, -0.02f, -0.0226f),

        cv::Vec3f(-0.02f, 0.02f, 0.0f),         cv::Vec3f(0.02f, 0.02f, 0.0f),
        cv::Vec3f(0.02f, -0.02f, 0.0f),         cv::Vec3f(-0.02f, -0.02f, 0.0f),

        cv::Vec3f(0.02971f, 0.02f, -0.0026f),   cv::Vec3f(0.06435f, 0.02f, -0.0226f),
        cv::Vec3f(0.06435f, -0.02f, -0.0226f),  cv::Vec3f(0.02971f, -0.02f, -0.0026f)
    };

    // std::vector<cv::Vec3d> marker_rvecs;
    // std::vector<cv::Vec3d> marker_tvecs;

    // std::optional<Eigen::Quaternionf> marker_quat;
    std::optional<Eigen::Vector3f> marker_trans;
    std::optional<Eigen::Matrix3f> marker_rot;
    rl::Camera camera;
    camera.position = { 5.0f, 2.0f, 5.0f };
    camera.target = { 0.0f, 2.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    // rl::Mesh cube_mesh = GenMeshCone(1.0f, 1.0f, 10);
    // rl::Model cube_model = LoadModelFromMesh(cube_mesh);

    std::optional<cv::Vec3d> rvec;
    std::optional<cv::Vec3d> tvec;

    SetTargetFPS(60);

    while (!window.ShouldClose()) {
        video_capture >> frame;

        if (!frame_size.has_value()) {
            frame_size = frame.size();
            window.SetSize(frame_size->width, frame_size->height);
        }

        // cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        marker_ids.clear();
        marker_corners.clear();
        aruco_detector.detectMarkers(frame, marker_corners, marker_ids);
        cv::aruco::drawDetectedMarkers(frame, marker_corners);

        // bool use_prev = true;
        // if (marker_rvecs.size() < marker_corners.size()) {
        //     use_prev = false;
        //     marker_rvecs.resize(marker_corners.size());
        //     marker_tvecs.resize(marker_corners.size());
        // }

        std::vector<cv::Point3f> tracker_subset;
        std::vector<cv::Point2f> corners_subset;
        for (int i = 0; i < marker_corners.size(); ++i) {
            int sub_begin;
            int sub_end;
            bool valid_marker = true;
            switch (marker_ids.at(i)) {
            case 6:
                sub_begin = 0;
                sub_end = 4;
                break;
            case 4:
                sub_begin = 4;
                sub_end = 8;
                break;
            case 7:
                sub_begin = 8;
                sub_end = 12;
                break;
            default:
                valid_marker = false;
            }
            if (!valid_marker) {
                continue;
            }
            for (int c = sub_begin; c < sub_end; ++c) {
                tracker_subset.push_back(tracker_points.at(c));
            }
            corners_subset.insert(corners_subset.cend(), marker_corners.at(i).begin(), marker_corners.at(i).end());
        }
        if (!tracker_subset.empty()) {
            solvePnP(
                tracker_subset,
                corners_subset,
                camera_matrix,
                distortion_coefficients,
                *rvec,
                *tvec,
                rvec.has_value() && tvec.has_value(),
                cv::SOLVEPNP_ITERATIVE);

            drawFrameAxes(frame, camera_matrix, distortion_coefficients, *rvec, *tvec, 0.05);
        }

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
        window.ClearBackground(rl::Color::Black());
        if (rl_texture.has_value()) {
            rl_texture->Draw(0, 0);
        }
        window.EndDrawing();
    }

    return EXIT_SUCCESS;
}