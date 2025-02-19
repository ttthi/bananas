#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

#include <gsl/span>

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
#include <bananas_aruco/grid_board.h>
#include <bananas_aruco/visualization/visualizer.h>
#include <bananas_aruco/world.h>

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

// TODO(vainiovano): Take the camera matrix as input
// Phone camera:
// const float focal_length_x{3208.864324F};
// const float focal_length_y{3207.79768F};
// const float optical_center_x{2028.096239F};
// const float optical_center_y{1506.648877F};

// OAK-D Pro center camera, from the camera's default configuration:
const float focal_length_x{3080.274658203125F};
const float focal_length_y{3079.234130859375F};
const float optical_center_x{1907.0689697265625F};
const float optical_center_y{1073.337158203125F};
const cv::Mat distortion_coeffs{14.909083366394043,
                                -91.85440826416016,
                                3.919406299246475e-05,
                                0.0004296370898373425,
                                352.7801513671875,
                                14.676776885986328,
                                -90.88777923583984,
                                347.32025146484375,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                -0.0033481859136372805,
                                0.0007485055830329657};

const cv::Mat camera_matrix({3, 3},
                            std::initializer_list<float>{
                                focal_length_x, 0, optical_center_x, 0,
                                focal_length_y, optical_center_y, 0, 0, 1});

void read_dictionary(cv::aruco::Dictionary &dictionary,
                     const std::string &file_name) {
    const cv::FileStorage fs{file_name, cv::FileStorage::READ};
    const bool readOk{dictionary.readDictionary(fs.root())};
    if (!readOk) {
        throw std::runtime_error{"Invalid dictionary file"};
    }
}

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

    visualizer::Visualizer visualizer{};
    const auto ground_plane_width{parser.get<std::uint32_t>("w")};
    const auto ground_plane_height{parser.get<std::uint32_t>("h")};
    const auto ground_plane_marker_side{parser.get<float>("l")};
    const auto ground_plane_marker_separation{parser.get<float>("s")};
    // const auto cube_side{parser.get<float>("c")};
    // const auto cube_margin{parser.get<float>("m")};
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

    const board::GridSettings ground_settings{
        {ground_plane_width, ground_plane_height},
        ground_plane_marker_side,
        ground_plane_marker_separation,
        0};
    const world::StaticEnvironment::PlacedObject ground_object{ground_settings,
                                                               {}};
    const world::StaticEnvironment environment{
        dictionary, gsl::make_span(&ground_object, 1)};
    world::World world{camera_matrix, distortion_coeffs, dictionary,
                       environment};

    const auto box_id{world.addBox(board::example_box)};
    visualizer.addBox(box_id, board::example_box.size);

    // const auto cube_id{world.addCube(cube_side, cube_margin, 25)};
    // visualizer.addBox(cube_id, cube_side, cube_side, cube_side);

    // const auto cube2_id{world.addCube(cube_side, cube_margin, 31)};
    // visualizer.addBox(cube2_id, cube_side, cube_side, cube_side);

    // const auto cube3_id{world.addCube(cube_side, cube_margin, 37)};
    // visualizer.addBox(cube3_id, cube_side, cube_side, cube_side);

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
