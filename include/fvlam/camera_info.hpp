#ifndef FVLAM_CAMERA_INFO_HPP
#define FVLAM_CAMERA_INFO_HPP
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

#include <Eigen/Geometry>

namespace fvlam
{
// ==============================================================================
// CameraInfo class
// ==============================================================================

  class CameraInfo
  {
  public:
    using CameraMatrix = Eigen::Vector3d;
    using DistCoeffs = Eigen::Matrix<double, 5, 1>;

  private:
    std::uint32_t width_;
    std::uint32_t height_;
    CameraMatrix camera_matrix_;
    DistCoeffs dist_coeffs_;

  public:
    CameraInfo(std::uint32_t width, std::uint32_t height, CameraMatrix camera_matrix, DistCoeffs dist_coeffs) :
      width_{width}, height_{height}, camera_matrix_{std::move(camera_matrix)}, dist_coeffs_{std::move(dist_coeffs)}
    {}

    auto &width() const
    { return width_; }

    auto &height() const
    { return height_; }

    const auto &camera_matrix() const
    { return camera_matrix_; }

    const auto &dist_coeffs() const
    { return dist_coeffs_; }

    template<typename T>
    static CameraInfo from(const T &other);

    template<typename T>
    T to() const;

    std::string to_string() const;
  };
}

#endif //FVLAM_CAMERA_INFO_HPP
