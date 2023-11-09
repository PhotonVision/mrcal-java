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

#pragma once

extern "C" {
// Seems to be missing C++ guards
#include <mrcal/mrcal.h>

} // extern "C"

#include <span>
#include <vector>

class mrcal_result {
public:
  bool success;
  std::vector<double> intrinsics;
  double rms_error;
  std::vector<double> residuals;
  mrcal_calobject_warp_t calobject_warp;
  int Noutliers_board;
  // TODO standard devs
};

#include <opencv2/opencv.hpp>

mrcal_pose_t getSeedPose(const mrcal_point3_t *c_observations_board_pool,
                         cv::Size boardSize, cv::Size imagerSize,
                         double squareSize, double focal_len_guess);

mrcal_result mrcal_main(
    // List, depth is ordered array observation[N frames, object_height,
    // object_width] = [x,y, weight] weight<0 means ignored)
    std::span<mrcal_point3_t> observations_board,
    // RT transform from camera to object
    std::span<mrcal_pose_t> frames_rt_toref,
    // Chessboard size, in corners (not squares)
    cv::Size calobjectSize, double boardSpacing,
    // res, pixels
    cv::Size cameraRes);
