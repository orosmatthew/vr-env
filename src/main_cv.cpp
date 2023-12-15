#include <chrono>
#include <optional>
#include <vector>

#include <fstream>
#include <json.hpp>
#include <opencv2/opencv.hpp>

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

int main()
{
    const cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);

    std::vector<cv::Point3f> object_points_ref;
    for (int i = 0; i < checkers_size_height; i++)
        for (int j = 0; j < checkers_size_width; j++)
            object_points_ref.emplace_back(j, i, 0);

    // 2d points in image plane.
    std::vector<std::vector<cv::Point3f>> obj_points;
    std::vector<std::vector<cv::Point2f>> img_points;

    cv::VideoCapture video_capture(0);
    if (!video_capture.isOpened())
        return EXIT_FAILURE;

    namedWindow("Webcam", cv::WINDOW_NORMAL);

    std::optional<cv::Size> frame_size;

    bool exit = false;
    while (!exit) {
        cv::Mat frame;
        video_capture >> frame;

        cv::Mat frame_gray;
        cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        if (!frame_size.has_value()) {
            frame_size = frame.size();
        }

        if (std::vector<cv::Point2f> corners;
            findChessboardCorners(frame_gray, cv::Size(checkers_size_height, checkers_size_width), corners)) {
            cornerSubPix(frame_gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            obj_points.push_back(object_points_ref);
            img_points.push_back(corners);
            drawChessboardCorners(frame, cv::Size(checkers_size_height, checkers_size_width), corners, true);
        }
        imshow("Webcam", frame);
        if (cv::waitKey(1) == 'q') {
            exit = true;
        }
    }

    if (frame_size.has_value() && !img_points.empty()) {
        cv::Mat camera_matrix;
        std::vector<double> distortion_coefficients;
        std::vector<cv::Mat> rotation_vectors;
        std::vector<cv::Mat> translation_vectors;
        calibrateCamera(
            obj_points,
            img_points,
            frame_size.value(),
            camera_matrix,
            distortion_coefficients,
            rotation_vectors,
            translation_vectors);

        write_camera_json(camera_matrix, distortion_coefficients);
        std::cout << camera_matrix << std::endl;
        for (double d : distortion_coefficients) {
            std::cout << d << ", ";
        }
        std::cout << std::endl;
    }

    video_capture.release();
    cv::destroyAllWindows();

    return EXIT_SUCCESS;
}