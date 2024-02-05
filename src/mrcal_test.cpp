/*
 * Copyright (C) Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <malloc.h>
#include <stdint.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <span>
#include <stdexcept>
#include <vector>

#include "mrcal_jni.h"
#include "mrcal_wrapper.h"

using namespace cv;

extern "C" {
#include <vnlog-parser.h>
} // extern "C"

struct cmpByFilename {
  bool operator()(const std::string &a, const std::string &b) const {
    auto a2 = std::stoi(a.substr(3, a.length() - 7));
    auto b2 = std::stoi(b.substr(3, b.length() - 7));
    // std::cout << a2 << " _ " << b2 << std::endl;
    return a2 < b2;
  }
};

int homography_test() {
  Size boardSize = {7, 7};
  Size imagerSize = {640, 480};
  // std::FILE *fp =
  //     std::fopen("/home/matt/github/photon_640_480/corners.vnl", "r");
  // Size boardSize = {10, 10};
  // Size imagerSize = {1600, 896};
  std::FILE *fp =
      std::fopen("/home/matt/Documents/GitHub/photonvision/test-resources/"
                 "calibrationSquaresImg/piCam/640_480_1/corners.vnl",
                 "r");

  if (fp == NULL)
    return -1;

  vnlog_parser_t ctx;
  if (VNL_OK != vnlog_parser_init(&ctx, fp))
    return -1;

  std::map<std::string, std::vector<mrcal_point3_t>> points;
  vnlog_parser_result_t result;
  while (VNL_OK == (result = vnlog_parser_read_record(&ctx, fp))) {
    const char *const *name = vnlog_parser_record_from_key(&ctx, "filename");
    const char *const *x = vnlog_parser_record_from_key(&ctx, "x");
    const char *const *y = vnlog_parser_record_from_key(&ctx, "y");
    const char *const *level_ = vnlog_parser_record_from_key(&ctx, "level");

    // From calibration.py:
    // if weight_column_kind == 'level': the 4th column is a decimation level of
    // the
    //   detected corner. If we needed to cut down the image resolution to
    //   detect a corner, its coordinates are known less precisely, and we use
    //   that information to weight the errors appropriately later. We set the
    //   output weight = 1/2^level. If the 4th column is '-' or <0, the given
    //   point was not detected, and should be ignored: we set weight = -1

    try {
      using std::stod;
      double level = stod(*level_);
      double weight;
      if (level < 0) {
        weight = -1;
      } else {
        weight = std::pow(0.5, level);
      }
      points[*name].push_back(mrcal_point3_t{stod(*x), stod(*y), weight});
      // std::printf("put %s\n", *name);
    } catch (std::exception const &e) {
    }
  }

  vnlog_parser_free(&ctx);
  std::fclose(fp);

  auto start = std::chrono::steady_clock::now();

  std::vector<mrcal_point3_t> observations_board;
  std::vector<mrcal_pose_t> frames_rt_toref;
  // Pre-allocate worst case
  size_t total_points = boardSize.width * boardSize.height * points.size();
  observations_board.reserve(total_points);
  frames_rt_toref.reserve(points.size());

  for (const auto &[key, value] : points) {
    if (value.size()) {
      auto ret = getSeedPose(value.data(), boardSize, imagerSize, 0.0254, 800);

      if (0)
        std::printf("Seed pose %s: r %f %f %f t %f %f %f\n", key.c_str(),
                    ret.r.x, ret.r.y, ret.r.z, ret.t.x, ret.t.y, ret.t.z);

      // Append to the Big List of board corners/levels
      observations_board.insert(observations_board.end(), value.begin(),
                                value.end());
      // And list of pose seeds
      frames_rt_toref.push_back(ret);
    } else {
      std::printf("No points for %s\n", key.c_str());
    }
  }

  auto cal_result = mrcal_main(observations_board, frames_rt_toref, boardSize,
                               0.0254, imagerSize);

  auto dt = std::chrono::steady_clock::now() - start;
  int dt_ms = dt.count();

  auto &stats = *cal_result;

  double max_error =
      *std::max_element(stats.residuals.begin(), stats.residuals.end());

  if (0) {
    std::printf("\n===============================\n\n");
    std::printf("RMS Reprojection Error: %.2f pixels\n", stats.rms_error);
    std::printf("Worst residual (by measurement): %.1f pixels\n", max_error);
    std::printf("Noutliers: %i of %lu (%.1f percent of the data)\n",
                stats.Noutliers_board, total_points,
                100.0 * stats.Noutliers_board / total_points);
    std::printf("calobject_warp: [%f, %f]\n", stats.calobject_warp.x2,
                stats.calobject_warp.y2);
    std::printf("dt, seeding + solve: %f ms\n", dt_ms / 1e6);
    std::printf("Intrinsics [%lu]:\n", stats.intrinsics.size());
    for (auto i : stats.intrinsics)
      std::printf("%f ", i);
    std::printf("\n");
  }

  double fx = 100, fy = 100, cx = 50, cy = 20;
  cv::Mat1d camMat = (Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  cv::Mat1d distCoeffs = (Mat_<double>(1,5) << 0.17802570252202954,-1.461379065131586,0.001019661566461145,0.0003215220840230439,2.7249642067580533);

  undistort_mrcal(&inputs, &outputs, &camMat, &distCoeffs, CameraLensModel::LENSMODEL_OPENCV5, 0, 0, 0, 0);

  cv::Mat2d outputs_opencv = cv::Mat2d::zeros(inputs.size());
  cv::undistortImagePoints(inputs, outputs_opencv, camMat, distCoeffs,
                          TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 50,
                                                      1e-4));

  std::cout << "cam mat\n" << camMat << std::endl;
  std::cout << "dist\n" << distCoeffs << std::endl;
  std::cout << "Inputs\n" << inputs << std::endl;
  std::cout << "Outputs (mrcal)\n" << outputs << std::endl;
  std::cout << "Outputs (opencv)\n" << outputs_opencv << std::endl;
}

int main() {
  // for (int i = 0; i < 1e6; i++) {
    homography_test();
  // }

}
