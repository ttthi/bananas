#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/videoio.hpp>

#include <bananas_aruco/box_board.h>
#include <bananas_aruco/visualization/visualizer.h>
#include <bananas_aruco/world.h>

namespace {

const char *const about{"Find camera and box locations from a video"};
const char *const keys{
    "{@infile | <none> | Input video }"
    "{env     | <none> | JSON file describing the static environment }"
    "{boxes   | <none> | JSON file containing the box descriptions }"
    "{camera  | <none> | JSON file containing the camera information }"
    "{vo      |        | Video output file }"};

struct CameraCalibration {
    float focal_length_x{};
    float focal_length_y{};
    float optical_center_x{};
    float optical_center_y{};
    std::vector<float> distortion_coefficients{};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CameraCalibration, focal_length_x,
                                   focal_length_y, optical_center_x,
                                   optical_center_y, distortion_coefficients);

/// Returns true if the user wants to exit the application. Handles pausing and
/// blocks until the user unpauses.
auto handle_keys() -> bool {
    bool paused{false};
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-do-while)
    do {
        const int key{cv::waitKey(paused ? 0 : 40)};
        // \033 ESC
        if (key == '\033' || key == 'q') {
            return true;
        }
        if (key == ' ') {
            paused = !paused;
        }
    } while (paused);
    return false;
}

} // namespace

auto main(int argc, char *argv[]) -> int {
    cv::CommandLineParser parser{argc, argv, keys};
    parser.about(about);

    const auto camera_file{parser.get<std::string>("camera")};
    const auto static_environment_file{parser.get<std::string>("env")};
    const auto box_file{parser.get<std::string>("boxes")};
    const auto video_file{parser.get<std::string>(0)};
    std::optional<std::string> video_output_file{};
    if (parser.has("vo")) {
        video_output_file = parser.get<std::string>("vo");
    }

    if (!parser.check()) {
        parser.printErrors();
        parser.printMessage();
        return EXIT_FAILURE;
    }

    cv::Mat camera_matrix{};
    cv::Mat distortion_coefficients{};
    {
        std::ifstream camera_info_stream{camera_file};
        if (!camera_info_stream) {
            std::cerr << "Failed to open camera information file\n";
            return EXIT_FAILURE;
        }

        CameraCalibration camera_info;
        try {
            const auto json = nlohmann::json::parse(camera_info_stream);
            json.get_to(camera_info);
        } catch (const std::exception &e) {
            std::cerr << "Failed to parse camera information file: " << e.what()
                      << '\n';
            return EXIT_FAILURE;
        }
        cv::Mat calibration_matrix{{3, 3},
                                   std::initializer_list<float>{
                                       camera_info.focal_length_x, 0,
                                       camera_info.optical_center_x, 0,
                                       camera_info.focal_length_y,
                                       camera_info.optical_center_y, 0, 0, 1}};
        cv::Mat distortion_mat{camera_info.distortion_coefficients, true};

        camera_matrix = std::move(calibration_matrix);
        distortion_coefficients = std::move(distortion_mat);
    }

    world::StaticEnvironment static_environment{
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100), {}};
    {
        std::ifstream static_env_stream{static_environment_file};
        if (!static_env_stream) {
            std::cerr << "Failed to open static environment file\n";
            return EXIT_FAILURE;
        }

        try {
            const auto json = nlohmann::json::parse(static_env_stream);
            json.get_to(static_environment);
        } catch (const std::exception &e) {
            std::cerr << "Failed to parse static environment file: " << e.what()
                      << '\n';
            return EXIT_FAILURE;
        }
    }
    std::vector<board::BoxSettings> boxes{};
    {
        std::ifstream box_stream{box_file};
        if (!box_stream) {
            std::cerr << "Failed to open box file\n";
            return EXIT_FAILURE;
        }

        try {
            const auto json = nlohmann::json::parse(box_stream);
            json.get_to(boxes);
        } catch (const std::exception &e) {
            std::cerr << "Failed to parse box file: " << e.what() << '\n';
            return EXIT_FAILURE;
        }
    }

    world::World world{camera_matrix, distortion_coefficients,
                       static_environment.getBoard().getDictionary(),
                       static_environment};
    visualizer::Visualizer visualizer{};

    visualizer.updateStaticEnvironment(static_environment.getObjects());
    for (const auto &box : boxes) {
        const auto box_id{world.addBox(board::example_box)};
        visualizer.addBox(box_id, box.size);
    }

    cv::VideoCapture capture{video_file};
    cv::VideoWriter output{};
    if (video_output_file) {
        output.open(*video_output_file,
                    cv::VideoWriter::fourcc('a', 'v', 'c', '1'),
                    capture.get(cv::CAP_PROP_FPS),
                    {static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH)),
                     static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT))});
    }

    cv::Mat image{};
    cv::Mat render_image{};
    while (capture.isOpened()) {
        const bool got_frame{capture.read(image)};
        if (!got_frame) {
            break;
        }

        const auto fit_result{world.fit(image)};
        visualizer.update(fit_result);
        visualizer.refresh();

        image.copyTo(render_image);
        cv::aruco::drawDetectedMarkers(render_image, fit_result.corners,
                                       fit_result.ids);
        if (video_output_file) {
            output.write(render_image);
        } else {
            cv::imshow("out", render_image);
            if (handle_keys()) {
                break;
            }
        }
    }
}
