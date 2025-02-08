#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gsl/span>

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

const std::array<const int, 6> cube_id_offsets{3, 4, 2, 1, 0, 5};
const float quarter_circle{std::atanf(1.0F) * 2.0F};
const std::array<const float, 6> cube_face_angles{
    quarter_circle, -quarter_circle, 0, 0, 2 * quarter_circle, 0};

struct GroundPlaneSettings {
    cv::Size size{1, 1};
    float marker_side{1.0F};
    float marker_separation{1.0F};
    int from_id{};
};

struct BoxFaceSettings {
    float left_margin{};
    float top_margin{};
    float rotation{};
    float side{1.0F};
    int id{};
};

struct BoxSettings {
    cv::Vec3f size{};
    // TODO(vainiovano): define the order
    std::array<BoxFaceSettings, 6> faces{};
};

auto cube_ids_from(int from) -> std::array<int, 6> {
    std::array<int, 6> result{};
    std::transform(cube_id_offsets.cbegin(), cube_id_offsets.cend(),
                   result.begin(), [from](int id) { return from + id; });
    return result;
}

auto make_cube_settings(float size, float margin, gsl::span<const int, 6> ids)
    -> BoxSettings {
    BoxSettings box_settings{};
    box_settings.size = {size, size, size};
    for (std::size_t i{0}; i < box_settings.faces.size(); ++i) {
        box_settings.faces[i].left_margin = margin;
        box_settings.faces[i].top_margin = margin;
        box_settings.faces[i].side = size - 2 * margin;
        box_settings.faces[i].id = ids[i];
    }
    for (std::size_t i{0}; i < box_settings.faces.size(); ++i) {
        box_settings.faces[i].left_margin = margin;
        box_settings.faces[i].top_margin = margin;
        box_settings.faces[i].side = size - 2 * margin;
        box_settings.faces[i].rotation = cube_face_angles[i];
    }
    return box_settings;
}

auto form_box_face(const BoxFaceSettings &face_settings)
    -> std::array<cv::Point2f, 4> {
    const float half_side{face_settings.side / 2.0F};
    std::array<cv::Point2f, 4> face{cv::Point2f{-half_side, -half_side},
                                    {half_side, -half_side},
                                    {half_side, half_side},
                                    {-half_side, half_side}};

    std::transform(
        face.cbegin(), face.cend(), face.begin(),
        [&face_settings](cv::Point2f point) -> cv::Point2f {
            const std::complex<float> point_complex{point.x, point.y};
            const std::complex<float> rotated{
                std::polar(1.0F, face_settings.rotation) * point_complex};
            // TODO(vainiovano): margins
            // const std::complex<float> translated{
            //     rotated.real() + face_settings.left_margin,
            //     rotated.imag() + face_settings.top_margin};
            return {rotated.real(), rotated.imag()};
        });

    return face;
}

auto form_box_board(const cv::aruco::Dictionary &dictionary,
                    const BoxSettings &box_settings) -> cv::aruco::Board {
    std::vector<std::vector<cv::Point3f>> object_points(
        6, std::vector<cv::Point3f>(4));

    const auto set_face{
        [&box_settings, &object_points](
            int face_index,
            std::function<cv::Point3f(cv::Point2f)> to_space) -> void {
            const auto face{form_box_face(box_settings.faces[face_index])};
            std::transform(face.cbegin(), face.cend(),
                           object_points[face_index].begin(),
                           std::move(to_space));
        }};

    // TODO(vainiovano): Select a coordinate system that makes sense.
    set_face(0, [&box_settings](cv::Point2f point) -> cv::Point3f {
        return {point.x, point.y, -box_settings.size[0] / 2.0F};
    });
    set_face(1, [&box_settings](cv::Point2f point) -> cv::Point3f {
        return {-point.x, point.y, box_settings.size[0] / 2.0F};
    });
    set_face(2, [&box_settings](cv::Point2f point) -> cv::Point3f {
        return {-box_settings.size[1] / 2.0F, -point.x, -point.y};
    });
    set_face(3, [&box_settings](cv::Point2f point) -> cv::Point3f {
        return {box_settings.size[1] / 2.0F, -point.x, point.y};
    });
    set_face(4, [&box_settings](cv::Point2f point) -> cv::Point3f {
        return {point.x, -box_settings.size[2] / 2.0F, -point.y};
    });
    set_face(5, [&box_settings](cv::Point2f point) -> cv::Point3f {
        return {point.x, box_settings.size[2] / 2.0F, point.y};
    });

    std::array<int, 6> ids{};
    std::transform(box_settings.faces.cbegin(), box_settings.faces.cend(),
                   ids.begin(),
                   [](const BoxFaceSettings &face) -> int { return face.id; });
    return {object_points, dictionary, ids};
}

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
    world::World world{camera_matrix, {}, dictionary, std::move(ground_board)};

    const auto cube_settings{
        make_cube_settings(cube_side, cube_margin, cube_ids_from(25))};
    auto cube_board{form_box_board(dictionary, cube_settings)};
    const auto cube_id{world.addBoard(std::move(cube_board))};
    visualizer.addBox(cube_id, cube_side, cube_side, cube_side);

    const auto cube2_settings{
        make_cube_settings(cube_side, cube_margin, cube_ids_from(31))};
    auto cube2_board{form_box_board(dictionary, cube2_settings)};
    const auto cube2_id{world.addBoard(std::move(cube2_board))};
    visualizer.addBox(cube2_id, cube_side, cube_side, cube_side);

    const auto cube3_settings{
        make_cube_settings(cube_side, cube_margin, cube_ids_from(37))};
    auto cube3_board{form_box_board(dictionary, cube3_settings)};
    const auto cube3_id{world.addBoard(std::move(cube3_board))};
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
