#include <cstdlib>
#include <initializer_list>
#include <iostream>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/videoio.hpp>

#include "visualization/visualizer.h"
#include "world.h"

namespace {

const char *const about{"TODO"};
const char *const keys{"{@infile  |<none> | Input video }"
                       "{w        |       | Number of markers in X direction }"
                       "{h        |       | Number of markers in Y direction }"
                       "{l        |       | Marker side length }"
                       "{s        |       | Separation between two consecutive "
                       "markers in the grid }"
                       "{c        |       | Cube side length }"
                       "{m        |       | Cube marker margin }"
                       "{cd       |       | Input file with custom dictionary }"
                       "{vo       |       | Video output file }"};

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

auto form_ground_board(const cv::aruco::Dictionary &dictionary,
                       const GroundPlaneSettings &ground_settings)
    -> cv::aruco::Board {
    std::vector<int> ids(ground_settings.size.area());
    std::iota(ids.begin(), ids.end(), ground_settings.from_id);
    cv::aruco::GridBoard board{
        ground_settings.size, ground_settings.marker_side,
        ground_settings.marker_separation, dictionary, ids};
    return board;
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

    visualizer::Visualizer visualizer{};
    const auto ground_plane_width{parser.get<int>("w")};
    const auto ground_plane_height{parser.get<int>("h")};
    const auto ground_plane_marker_side{parser.get<float>("l")};
    const auto ground_plane_marker_separation{parser.get<float>("s")};
    const auto cube_side{parser.get<float>("c")};
    const auto cube_margin{parser.get<float>("m")};
    const auto dictionary_file{parser.get<std::string>("cd")};
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
    auto ground_board{form_ground_board(dictionary, ground_settings)};
    world::World world{
        camera_matrix, {}, std::move(dictionary), std::move(ground_board)};

    const auto cube_id{world.addCube(cube_side, cube_margin, 25)};
    visualizer.addBox(cube_id, cube_side, cube_side, cube_side);

    const auto cube2_id{world.addCube(cube_side, cube_margin, 31)};
    visualizer.addBox(cube2_id, cube_side, cube_side, cube_side);

    const auto cube3_id{world.addCube(cube_side, cube_margin, 37)};
    visualizer.addBox(cube3_id, cube_side, cube_side, cube_side);

    const float plane_width{
        static_cast<float>(ground_plane_width) *
        (ground_plane_marker_side + ground_plane_marker_separation)};
    const float plane_height{
        static_cast<float>(ground_plane_height) *
        (ground_plane_marker_side + ground_plane_marker_separation)};
    visualizer.setStaticEnvironmentSize(plane_width, plane_height);

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
