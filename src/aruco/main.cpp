#include <filesystem>
#include <fstream>
#include <iostream>

#include <eigen3/Eigen/Geometry>
#include <json.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <raylib-cpp.hpp>

using json = nlohmann::json;
namespace rl = raylib;

/*
    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

    cv::Mat R;
    cv::Rodrigues(rvec, R); // Convert rotation vector to rotation matrix

    R = R.t();  // Inverse rotation matrix (transpose of the rotation matrix)
    tvec = -R * tvec;  // Inverse translation vector

    cv::Mat T = cv::Mat::eye(4, 4, R.type()); // Create a 4x4 transformation matrix
    T(cv::Range(0,3), cv::Range(0,3)) = R * 1; // Copy R into T
    T(cv::Range(0,3), cv::Range(3,4)) = tvec * 1; // Copy tvec into T
*/

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
    // aruco_parameters.adaptiveThreshWinSizeMin = 3;
    // aruco_parameters.adaptiveThreshWinSizeMax = 23;
    // aruco_parameters.adaptiveThreshWinSizeStep = 10;
    aruco_parameters.cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
    // aruco_parameters.minMarkerPerimeterRate = 0.02;
    // aruco_parameters.maxMarkerPerimeterRate = 4.0;
    // aruco_parameters.polygonalApproxAccuracyRate = 0.4;
    cv::aruco::Dictionary aruco_dict = getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    cv::aruco::ArucoDetector aruco_detector { aruco_dict, aruco_parameters };

    // cv::Mat frame_undistorted;
    // ReSharper disable once CppTooWideScope
    cv::Mat camera_matrix_undistorted;
    cv::Mat frame;
    cv::Mat frame_gray;
    cv::Mat screen;

    float marker_length = 0.07f;
    cv::Mat marker_points(4, 1, CV_32FC3);
    marker_points.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_length / 2.f, marker_length / 2.f, 0);
    marker_points.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(marker_length / 2.f, marker_length / 2.f, 0);
    marker_points.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(marker_length / 2.f, -marker_length / 2.f, 0);
    marker_points.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-marker_length / 2.f, -marker_length / 2.f, 0);
    // marker_points.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(0, marker_length, 0);
    // marker_points.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(marker_length, marker_length, 0);
    // marker_points.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(marker_length, 0, 0);
    // marker_points.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(0, 0, 0);

    std::vector<cv::Vec3d> marker_rvecs;
    std::vector<cv::Vec3d> marker_tvecs;

    // std::optional<Eigen::Quaternionf> marker_quat;
    std::optional<Eigen::Vector3f> marker_trans;
    std::optional<Eigen::Matrix3f> marker_rot;
    rl::Camera camera;
    camera.position = { 5.0f, 2.0f, 5.0f };
    camera.target = { 0.0f, 2.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    rl::Mesh cube_mesh = GenMeshCone(1.0f, 1.0f, 10);
    rl::Model cube_model = LoadModelFromMesh(cube_mesh);

    while (!window.ShouldClose()) {

        video_capture >> frame;

        // cv::Mat new_camera_matrix = getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, frame.size(),
        // 1); undistort(frame, frame_undistorted, camera_matrix, distortion_coefficients, new_camera_matrix);

        if (!frame_size.has_value()) {
            frame_size = frame.size();
            window.SetSize(frame_size->width, frame_size->height);
        }

        cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        marker_ids.clear();
        marker_corners.clear();
        aruco_detector.detectMarkers(frame_gray, marker_corners, marker_ids);
        cv::aruco::drawDetectedMarkers(frame, marker_corners);

        bool use_prev = true;
        if (marker_rvecs.empty()) {
            use_prev = false;
            marker_rvecs.resize(marker_corners.size());
            marker_tvecs.resize(marker_corners.size());
        }

        for (int i = 0; i < marker_corners.size(); ++i) {
            solvePnP(
                marker_points,
                marker_corners.at(i),
                camera_matrix,
                distortion_coefficients,
                marker_rvecs.at(i),
                marker_tvecs.at(i),
                use_prev,
                cv::SOLVEPNP_ITERATIVE);
            if (marker_ids.at(i) == 0) {
                const cv::Vec3d& rvec = marker_rvecs.at(i);
                cv::Mat rot_mat;
                Rodrigues(rvec, rot_mat);
                rot_mat = rot_mat.t();
                cv2eigen(rot_mat, *marker_rot);
                // marker_quat = Eigen::Quaternionf(mat);
                const cv::Vec3d& tvec = marker_tvecs.at(i);
                marker_trans = Eigen::Vector3f(
                    static_cast<float>(tvec[0]), static_cast<float>(tvec[1]), static_cast<float>(tvec[2]));
                drawFrameAxes(frame, camera_matrix, distortion_coefficients, rvec, tvec, 0.05);
            }
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
        // BeginMode3D(camera);
        // if (marker_trans.has_value()) {
        //     // std::cout << (*marker_trans)[0] << ", " << (*marker_trans)[1] << ", " << (*marker_trans)[2] <<
        //     std::endl;
        //     // std::cout << *marker_rot << std::endl;
        //     Eigen::Matrix3f r = marker_rot->transpose();
        //     cube_model.transform.m0 = r(0, 0);
        //     cube_model.transform.m1 = r(1, 0);
        //     cube_model.transform.m2 = r(2, 0);
        //     cube_model.transform.m4 = r(0, 1);
        //     cube_model.transform.m5 = r(1, 1);
        //     cube_model.transform.m6 = r(2, 1);
        //     cube_model.transform.m8 = r(0, 2);
        //     cube_model.transform.m9 = r(1, 2);
        //     cube_model.transform.m10 = r(2, 2);
        //
        //     rl::Vector3 pos { (*marker_trans)[0] * 2, (*marker_trans)[1] * 2, (*marker_trans)[2] * 2 };
        //     cube_model.Draw(pos, 1, rl::Color::Red());
        // }
        // EndMode3D();
        window.EndDrawing();
    }

    return EXIT_SUCCESS;
}