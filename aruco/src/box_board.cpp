#include <bananas_aruco/box_board.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <functional>
#include <utility>
#include <vector>

#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/objdetect/aruco_board.hpp>

namespace {

auto form_box_face(const box_board::BoxFaceSettings &face_settings)
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
            const std::complex<float> translated{
                rotated.real() + face_settings.x,
                rotated.imag() + face_settings.y};
            return {translated.real(), translated.imag()};
        });

    return face;
}

const float quarter_circle{std::atanf(1.0F) * 2.0F};

} // namespace

namespace box_board {

auto make_board(const cv::aruco::Dictionary &dictionary,
                const box_board::BoxSettings &settings) -> cv::aruco::Board {
    std::vector<std::vector<cv::Point3f>> object_points{};
    std::vector<int> ids{};

    // NOTE: For some reason, clang-tidy starts seeing null pointer issues if
    // this is written with direct {} initialization
    const auto set_face =
        [&settings, &object_points, &ids](
            int face_index,
            const std::function<cv::Point3f(cv::Point2f)> &to_space) -> void {
        for (const auto &face_aruco : settings.faces[face_index]) {
            const auto face{form_box_face(face_aruco)};
            std::vector<cv::Point3f> face_3d(4);
            std::transform(face.cbegin(), face.cend(), face_3d.begin(),
                           to_space);

            object_points.emplace_back(std::move(face_3d));
            ids.push_back(face_aruco.id);
        }
    };

    // TODO(vainiovano): Select a coordinate system that makes sense.
    set_face(0, [&settings](cv::Point2f point) -> cv::Point3f {
        return {point.x, point.y, -settings.size[2] / 2.0F};
    });
    set_face(1, [&settings](cv::Point2f point) -> cv::Point3f {
        return {-point.x, point.y, settings.size[2] / 2.0F};
    });
    set_face(2, [&settings](cv::Point2f point) -> cv::Point3f {
        return {-settings.size[0] / 2.0F, -point.x, -point.y};
    });
    set_face(3, [&settings](cv::Point2f point) -> cv::Point3f {
        return {settings.size[0] / 2.0F, -point.x, point.y};
    });
    set_face(4, [&settings](cv::Point2f point) -> cv::Point3f {
        return {point.x, -settings.size[1] / 2.0F, -point.y};
    });
    set_face(5, [&settings](cv::Point2f point) -> cv::Point3f {
        return {point.x, settings.size[1] / 2.0F, point.y};
    });

    return {object_points, dictionary, ids};
}

const box_board::BoxSettings example_box{
    {0.18F, 0.134F, 0.125F},
    {std::vector<box_board::BoxFaceSettings>{
         // X left, Y up, Z outwards
         {27, 0.028F, -0.0035F, quarter_circle, 0.038F},
         {30, -0.0315F, -0.0035F, quarter_circle, 0.038F}},
     {},
     {},
     {
         {43, -0.0515F, 0.0195F, -quarter_circle, 0.024F},
         {44, -0.0515F, -0.018F, -quarter_circle, 0.024F},
         {48, -0.0125F, 0.0195F, -quarter_circle, 0.024F},
         {49, -0.0125F, -0.018F, -quarter_circle, 0.024F},
     },
     {},
     {}}};

} // namespace box_board
