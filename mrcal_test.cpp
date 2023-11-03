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

#include <stdint.h>

#include <vector>

extern "C" {
    // SOMEONE forgot C++ guards
    #include "mrcal/mrcal.h"
}

#include "mrcal_jni.h"

typedef enum {
  OPTIMIZEMODE_OPTIMIZE,
  OPTIMIZEMODE_CALLBACK,
  OPTIMIZEMODE_DRTRRP_DB
} optimizemode_t;

#include <cstdio>
#define BARF(args...) std::fprintf(stderr, ##args)

static mrcal_problem_selections_t construct_problem_selections(
    bool do_optimize_intrinsics_core, bool do_optimize_intrinsics_distortions,
    bool do_optimize_extrinsics, bool do_optimize_frames,
    bool do_optimize_calobject_warp, bool do_apply_regularization,
    bool do_apply_regularization_unity_cam01, bool do_apply_outlier_rejection,

    bool Ncameras_intrinsics, bool Ncameras_extrinsics, bool Nframes,
    bool Nobservations_board);

bool lensmodel_one_validate_args( // out
    mrcal_lensmodel_t *mrcal_lensmodel,

    // in
    std::vector<double> intrinsics, bool do_check_layout);

int main() {

  int Ncameras_intrinsics = 8; // This is valid for OPENCV8 (8 distortion
                               // coefficients + pinhole params)
  int Ncameras_extrinsics = 0; // Seems to always be zero for single camera

  // number of pictures we're optimizing. TODO. Specifically length of
  // frames_rt_toref ?
  int Nframes = 8;
  // Number of board observations we've got. List of boards. in python, it's
  // (number of chessboard pictures) x (rows) x (cos) x (3)
  int Nobservations_board = 8;
  // Also looks to never be >0 in python for calibrate-camera?
//   int n_observations_point_triangulated = 0;
//   mrcal_observation_point_triangulated_t
//       c_observations_point_triangulated[n_observations_point_triangulated];

  // Looks like this is hard-coded to 0
  int Nobservations_point = 0;

  int calibration_object_width_n = 7;  // Width, in # of corners
  int calibration_object_height_n = 7; // Height, in # of corners

  // TODO set sizes and populate
  int imagersize[] = {960, 720};
  // mrcal_pose_t intrinsics[0];
  mrcal_calobject_warp_t * calobject_warp = NULL;  // this is non-NULL for the final solve! TODO

  int Nobservations_point_triangulated = 0; // no clue what this is

  int Npoints = 0;       // seems like this is also unused? whack
  int Npoints_fixed = 0; // seems like this is also unused? whack

  bool do_optimize_intrinsics_core =
      true; // basically just non-splined should always be true
  bool do_optimize_intrinsics_distortions =
      true;                           // can skip intrinsics if we want
  bool do_optimize_extrinsics = true; // can skip extrinsics if we want
  bool do_optimize_frames = true;
  bool do_optimize_calobject_warp = false;
  bool do_apply_regularization = true;
  bool do_apply_regularization_unity_cam01 = 0;
  bool do_apply_outlier_rejection = true; // can also skip

  std::vector<double> intrinsics = {
      700, 700, 320, 240, 0, 0,
      0,   0,   0,   0,   0, 0}; // best guess at intrinsics from prior solve

  std::vector<mrcal_pose_t> extrinsics_rt_fromref = {}; // Always zero for single camera, it seems?
  std::vector<mrcal_point3_t> points = {}; // Seems to always to be None for single camera?
  std::vector<mrcal_pose_t> frames_rt_toref = {}; // TODO!

  mrcal_lensmodel_t mrcal_lensmodel;
  mrcal_lensmodel.type = MRCAL_LENSMODEL_OPENCV8; // TODO expose other models
  if (!lensmodel_one_validate_args(&mrcal_lensmodel, intrinsics,
                                   false))
    return false;

  mrcal_problem_selections_t problem_selections = construct_problem_selections(
      do_optimize_intrinsics_core, do_optimize_intrinsics_distortions,
      do_optimize_extrinsics, do_optimize_frames, do_optimize_calobject_warp,
      do_apply_regularization, do_apply_regularization_unity_cam01,
      do_apply_outlier_rejection,

      Ncameras_intrinsics, Ncameras_extrinsics, Nframes, Nobservations_board);

  int Nstate = mrcal_num_states(
      Ncameras_intrinsics, Ncameras_extrinsics, Nframes, Npoints, Npoints_fixed,
      Nobservations_board, problem_selections, &mrcal_lensmodel);

  int Nmeasurements = mrcal_num_measurements(
      Nobservations_board, Nobservations_point,
    //   c_observations_point_triangulated, Nobservations_point_triangulated,
      calibration_object_width_n, calibration_object_height_n,
      Ncameras_intrinsics, Ncameras_extrinsics, Nframes, Npoints, Npoints_fixed,
      problem_selections, &mrcal_lensmodel);


  // Outputs
  double b_packed[Nstate];
  double x[Nmeasurements];

  // Seeds (TODO)
  double *c_intrinsics = intrinsics.data();
  mrcal_pose_t *c_extrinsics = extrinsics_rt_fromref.data();
  mrcal_pose_t *c_frames = frames_rt_toref.data();
  mrcal_point3_t *c_points = points.data();
  mrcal_calobject_warp_t *c_calobject_warp = calobject_warp;

  // in (TODO)
  mrcal_observation_board_t c_observations_board[Nobservations_board];
  mrcal_observation_point_t c_observations_point[Nobservations_point];
  mrcal_point3_t *c_observations_board_pool;
  mrcal_point3_t *c_observations_point_pool;
  int *c_imagersizes = imagersize;

  auto point_min_range = -1.0, point_max_range = -1.0;
  mrcal_problem_constants_t problem_constants = {
      .point_min_range = point_min_range, .point_max_range = point_max_range};
  double calibration_object_spacing;
  int verbose = 1;

  mrcal_optimize(
      b_packed, sizeof(b_packed), x, sizeof(x),
      c_intrinsics, c_extrinsics, c_frames, c_points, c_calobject_warp,
      Ncameras_intrinsics, Ncameras_extrinsics, Nframes, Npoints, Npoints_fixed,
      c_observations_board, c_observations_point, Nobservations_board,
      Nobservations_point,
    //   c_observations_point_triangulated, Nobservations_point_triangulated,
      c_observations_board_pool, 
    //   c_observations_point_pool,
      &mrcal_lensmodel, c_imagersizes, problem_selections, &problem_constants,
      calibration_object_spacing, calibration_object_width_n,
      calibration_object_height_n, verbose,

      false);

  return 0;
}

// lifted from mrcal-pywrap.c
static mrcal_problem_selections_t construct_problem_selections(
    bool do_optimize_intrinsics_core, bool do_optimize_intrinsics_distortions,
    bool do_optimize_extrinsics, bool do_optimize_frames,
    bool do_optimize_calobject_warp, bool do_apply_regularization,
    bool do_apply_regularization_unity_cam01, bool do_apply_outlier_rejection,

    bool Ncameras_intrinsics, bool Ncameras_extrinsics, bool Nframes,
    bool Nobservations_board) {
  // By default we optimize everything we can
  if (do_optimize_intrinsics_core < 0)
    do_optimize_intrinsics_core = Ncameras_intrinsics > 0;
  if (do_optimize_intrinsics_distortions < 0)
    do_optimize_intrinsics_core = Ncameras_intrinsics > 0;
  if (do_optimize_extrinsics < 0)
    do_optimize_extrinsics = Ncameras_extrinsics > 0;
  if (do_optimize_frames < 0)
    do_optimize_frames = Nframes > 0;
  if (do_optimize_calobject_warp < 0)
    do_optimize_calobject_warp = Nobservations_board > 0;
  return {.do_optimize_intrinsics_core = do_optimize_intrinsics_core,
          .do_optimize_intrinsics_distortions =
              do_optimize_intrinsics_distortions,
          .do_optimize_extrinsics = do_optimize_extrinsics,
          .do_optimize_frames = do_optimize_frames,
          .do_optimize_calobject_warp = do_optimize_calobject_warp,
          .do_apply_regularization = do_apply_regularization,
          .do_apply_outlier_rejection = do_apply_outlier_rejection};
        //   .do_apply_regularization_unity_cam01 =
            //   do_apply_regularization_unity_cam01};
}

bool lensmodel_one_validate_args( // out
    mrcal_lensmodel_t *mrcal_lensmodel,

    // in
    std::vector<double> intrinsics, bool do_check_layout) {

  int NlensParams = mrcal_lensmodel_num_params(mrcal_lensmodel);
  int NlensParams_have = intrinsics.size();
  if (NlensParams != NlensParams_have) {
    BARF("intrinsics.shape[-1] MUST be %d. Instead got %d", NlensParams,
         NlensParams_have);
    return false;
  }

  return true;
}
