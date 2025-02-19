#include <bananas_aruco/box_board.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <functional>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <opencv2/core/types.hpp>

#include <bananas_aruco/board.h>

namespace {

auto form_box_face(const board::BoxMarkerSettings &face_settings)
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
            // Apply the rotation backwards: we want it to go counterclockwise,
            // but since OpenCV markers are "upside down", the rotation needs to
            // be the other way around here.
            const std::complex<float> rotated{
                std::polar(1.0F, -face_settings.rotation) * point_complex};
            // Apply the Y offset backwards: we want it to go "upwards", not
            // "downwards".
            const std::complex<float> translated{
                rotated.real() + face_settings.x_offset,
                rotated.imag() - face_settings.y_offset};
            return {translated.real(), translated.imag()};
        });

    return face;
}

const float quarter_circle{std::atanf(1.0F) * 2.0F};

} // namespace

namespace board {

void from_json(const nlohmann::json &j, BoxSettings &box_settings) {
    j.at("size").get_to(box_settings.size);

    const auto &markers{j.at("markers")};
    auto forward_markers{
        markers.value<std::vector<BoxMarkerSettings>>("forward", {})};
    auto backward_markers{
        markers.value<std::vector<BoxMarkerSettings>>("backward", {})};
    auto left_markers{
        markers.value<std::vector<BoxMarkerSettings>>("left", {})};
    auto right_markers{
        markers.value<std::vector<BoxMarkerSettings>>("right", {})};
    auto up_markers{markers.value<std::vector<BoxMarkerSettings>>("up", {})};
    auto down_markers{
        markers.value<std::vector<BoxMarkerSettings>>("down", {})};
    box_settings.markers = {
        std::move(forward_markers), std::move(backward_markers),
        std::move(left_markers),    std::move(right_markers),
        std::move(up_markers),      std::move(down_markers)};
}

auto make_board(const BoxSettings &settings) -> board::Board {
    std::vector<std::vector<cv::Point3f>> object_points{};
    std::vector<int> ids{};

    // NOTE: For some reason, clang-tidy starts seeing null pointer issues if
    // this is written with direct {} initialization.
    const auto set_face =
        [&settings, &object_points, &ids](
            int face_index,
            const std::function<cv::Point3f(cv::Point2f)> &to_space) -> void {
        for (const auto &face_aruco : settings.markers[face_index]) {
            const auto face{form_box_face(face_aruco)};
            std::vector<cv::Point3f> face_3d(4);
            std::transform(face.cbegin(), face.cend(), face_3d.begin(),
                           to_space);

            object_points.emplace_back(std::move(face_3d));
            ids.push_back(face_aruco.id);
        }
    };

    set_face(BoxSettings::forward_face_index,
             [&settings](cv::Point2f point) -> cv::Point3f {
                 return {point.x, -point.y, settings.size.depth / 2.0F};
             });
    set_face(BoxSettings::backward_face_index,
             [&settings](cv::Point2f point) -> cv::Point3f {
                 return {-point.x, -point.y, -settings.size.depth / 2.0F};
             });
    set_face(BoxSettings::left_face_index,
             [&settings](cv::Point2f point) -> cv::Point3f {
                 return {settings.size.width / 2.0F, -point.y, -point.x};
             });
    set_face(BoxSettings::right_face_index,
             [&settings](cv::Point2f point) -> cv::Point3f {
                 return {-settings.size.width / 2.0F, -point.y, point.x};
             });
    set_face(BoxSettings::up_face_index,
             [&settings](cv::Point2f point) -> cv::Point3f {
                 return {point.y, settings.size.height / 2.0F, -point.x};
             });
    set_face(BoxSettings::down_face_index,
             [&settings](cv::Point2f point) -> cv::Point3f {
                 return {point.y, -settings.size.height / 2.0F, point.x};
             });

    return {std::move(object_points), std::move(ids)};
}

const BoxSettings example_box{
    {0.125F, 0.134F, 0.18F},
    {std::vector<BoxMarkerSettings>{{43, -0.0195F, 0.0515F, 0.0F, 0.024F},
                                    {44, 0.018F, 0.0515F, 0.0F, 0.024F},
                                    {48, -0.0195F, 0.0125F, 0.0F, 0.024F},
                                    {49, 0.018F, 0.0125F, 0.0F, 0.024F}},
     {},
     {{27, -0.028F, -0.0035F, quarter_circle, 0.038F},
      {30, 0.0315F, -0.0035F, quarter_circle, 0.038F}},
     {},
     {},
     {}}};

} // namespace board
