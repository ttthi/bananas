#include <cstdlib>
#include <initializer_list>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/videoio.hpp>

namespace {

const char *const about{"TODO"};
const char *const keys{
    "{@infile  |<none> | Input video }"
    "{w        |       | Number of markers in X direction }"
    "{h        |       | Number of markers in Y direction }"
    "{l        |       | Marker side length }"
    "{s        |       | Separation between two consecutive "
    "markers in the grid }"
    "{cd       |       | Input file with custom dictionary }"};

const float focal_length_x{3555.555556F};
const float focal_length_y{3555.555556F};
const float optical_center_x{1280.0F};
const float optical_center_y{720.0F};
const cv::Mat camera_matrix({3, 3},
                            std::initializer_list<float>{
                                focal_length_x, 0, optical_center_x, 0,
                                focal_length_y, optical_center_y, 0, 0, 1});

struct GroundPlaneSettings {
    cv::Size size{1, 1};
    float marker_side{1.0F};
    float marker_separation{1.0F};
    int from_id{};
};

void read_dictionary(cv::aruco::Dictionary &dictionary,
                     const std::string &file_name) {
    const cv::FileStorage fs{file_name, cv::FileStorage::READ};
    const bool readOk{dictionary.readDictionary(fs.root())};
    if (!readOk) {
        throw std::runtime_error{"Invalid dictionary file"};
    }
}

auto form_board(const cv::aruco::Dictionary &dictionary,
                const GroundPlaneSettings &ground_settings)
    -> cv::aruco::Board {
    std::vector<int> ids(ground_settings.size.area());
    std::iota(ids.begin(), ids.end(), ground_settings.from_id);
    cv::aruco::GridBoard board{
        ground_settings.size, ground_settings.marker_side,
        ground_settings.marker_separation, dictionary, ids};
    return board;
}

auto detect_markers(const cv::aruco::Dictionary &dictionary,
                    const cv::aruco::Board &board, const cv::Mat &image)
    -> cv::Mat {
    // TODO(vainiovano): configurable detector parameters
    const cv::aruco::ArucoDetector detector{dictionary, {}};

    std::vector<std::vector<cv::Point2f>> corners{};
    std::vector<std::vector<cv::Point2f>> rejected{};
    std::vector<int> ids{};
    detector.detectMarkers(image, corners, ids, rejected);
    detector.refineDetectedMarkers(image, board, corners, ids, rejected,
                                   camera_matrix, {});

    cv::Vec3d rvec;
    cv::Vec3d tvec;
    bool board_detected{false};
    if (!ids.empty()) {
        cv::Mat object_points;
        cv::Mat image_points;
        board.matchImagePoints(corners, ids, object_points, image_points);
        cv::solvePnP(object_points, image_points, camera_matrix, {}, rvec,
                     tvec);
        board_detected = !object_points.empty();
    }

    cv::Mat render_image{};
    image.copyTo(render_image);
    if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(render_image, corners, ids);
    }
    if (board_detected) {
        cv::drawFrameAxes(render_image, camera_matrix, {}, rvec, tvec, 0.001F);
    }
    return render_image;
}

/// Returns true if the user wants to exit the application. Handles pausing and
/// blocks until the user unpauses.
auto handle_keys() -> bool {
    bool paused{false};
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-do-while)
    do {
        const int key{cv::waitKey(paused ? 0 : 60)};
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

    const auto ground_plane_width{parser.get<int>("w")};
    const auto ground_plane_height{parser.get<int>("h")};
    const auto ground_plane_marker_side{parser.get<float>("l")};
    const auto ground_plane_marker_separation{parser.get<float>("s")};
    const auto dictionary_file{parser.get<std::string>("cd")};
    const auto video_file{parser.get<std::string>(0)};

    if (!parser.check()) {
        parser.printErrors();
        parser.printMessage();
        return EXIT_FAILURE;
    }

    cv::aruco::Dictionary dictionary{};
    try {
        read_dictionary(dictionary, dictionary_file);
    } catch (const std::runtime_error &e) {
        std::cerr << "Failed to read dictionary: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    const struct GroundPlaneSettings ground_settings{
        {ground_plane_width, ground_plane_height},
        ground_plane_marker_side,
        ground_plane_marker_separation,
        0};
    const auto board{form_board(dictionary, ground_settings)};

    cv::VideoCapture capture{video_file};
    cv::Mat image{};
    while (capture.isOpened()) {
        const bool got_frame{capture.read(image)};
        if (!got_frame) {
            break;
        }

        const auto marker_image{detect_markers(dictionary, board, image)};
        cv::imshow("out", marker_image);
        if (handle_keys()) {
            break;
        };
    }
}
