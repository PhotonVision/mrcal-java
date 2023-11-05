#include "mrcal_wrapper.h"

#include <malloc.h>
#include <stdint.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <span>
#include <stdexcept>
#include <vector>

using namespace cv;

#define BARF(args...) std::fprintf(stderr, ##args)

// forward declarations for functions borrowed from mrcal-pywrap
static mrcal_problem_selections_t construct_problem_selections(
    int do_optimize_intrinsics_core, int do_optimize_intrinsics_distortions,
    int do_optimize_extrinsics, int do_optimize_frames,
    int do_optimize_calobject_warp, int do_apply_regularization,
    int do_apply_outlier_rejection, int Ncameras_intrinsics,
    int Ncameras_extrinsics, int Nframes, int Nobservations_board);

bool lensmodel_one_validate_args(mrcal_lensmodel_t *mrcal_lensmodel,
                                 std::vector<double> intrinsics,
                                 bool do_check_layout);


// Empty vector just to pass in so it's not NULL?
mrcal_point3_t observations_point[0];
mrcal_pose_t extrinsics_rt_fromref[0]; // Always zero for single camera, it seems?
mrcal_point3_t points[0];     // Seems to always to be None for single camera?

mrcal_result mrcal_main(
    // List, depth is ordered array observation[N frames, object_height,
    // object_width] = [x,y, weight] weight<0 means ignored)
    std::span<mrcal_point3_t> observations_board,
    // RT transform from camera to object
    std::span<mrcal_pose_t> frames_rt_toref,
    // Chessboard size, in corners (not squares)
    Size calobjectSize, double calibration_object_spacing,
    // res, pixels
    Size cameraRes) {
  // Number of board observations we've got. List of boards. in python, it's
  // (number of chessboard pictures) x (rows) x (cos) x (3)
  // hard-coded to 8, since that's what I've got below
  int Nobservations_board = frames_rt_toref.size();

  // Looks like this is hard-coded to 0 in Python for initial single-camera
  // solve?
  int Nobservations_point = 0;

  int calibration_object_width_n =
      calobjectSize.width; // Object width, in # of corners
  int calibration_object_height_n =
      calobjectSize.height; // Object height, in # of corners

  // TODO set sizes and populate
  int imagersize[] = {cameraRes.width, cameraRes.height};

  mrcal_calobject_warp_t calobject_warp = {0, 0};

  // int Nobservations_point_triangulated = 0; // no clue what this is

  int Npoints = 0;       // seems like this is also unused? whack
  int Npoints_fixed = 0; // seems like this is also unused? whack

  int do_optimize_intrinsics_core =
      1; // basically just non-splined should always be true
  int do_optimize_intrinsics_distortions = 1; // can skip intrinsics if we want
  int do_optimize_extrinsics = 1;             // can skip extrinsics if we want
  int do_optimize_frames = 1;
  int do_optimize_calobject_warp = 1;
  int do_apply_regularization = 1;
  int do_apply_outlier_rejection = 1; // can also skip

  mrcal_lensmodel_t mrcal_lensmodel;
  mrcal_lensmodel.type = MRCAL_LENSMODEL_OPENCV8; // TODO expose other models

  // pure pinhole guess for initial solve
  double cx = (cameraRes.width / 2.0) - 0.5;
  double cy = (cameraRes.height / 2.0) - 0.5;
  std::vector<double> intrinsics = {1200, 1200, cx, cy, 0, 0, 0, 0, 0, 0, 0, 0};

  // Number of cameras to solve for intrinsics
  int Ncameras_intrinsics = 1;

  // Hard-coded to match out 8 frames from above (borrowed from python)
  std::vector<mrcal_point3_t> indices_frame_camintrinsics_camextrinsics;
  // Frame index, camera number, (camera number)-1???
  for (int i = 0; i < Nobservations_board; i++) {
    indices_frame_camintrinsics_camextrinsics.push_back(
        {static_cast<double>(i), 0, -1});
  }

  // Pool is the raw observation backing array
  mrcal_point3_t *c_observations_board_pool = (observations_board.data());
  mrcal_point3_t *c_observations_point_pool = observations_point;

  // Copy from board/point pool above, using some code borrowed from
  // mrcal-pywrap
  mrcal_observation_board_t c_observations_board[Nobservations_board];
  // Try to make sure we don't accidentally make a zero-length array or something stupid
  mrcal_observation_point_t c_observations_point[std::min(Nobservations_point, 1)];

  for (int i_observation = 0; i_observation < Nobservations_board;
       i_observation++) {
    int32_t iframe =
        indices_frame_camintrinsics_camextrinsics.at(i_observation).x;
    int32_t icam_intrinsics =
        indices_frame_camintrinsics_camextrinsics.at(i_observation).y;
    int32_t icam_extrinsics =
        indices_frame_camintrinsics_camextrinsics.at(i_observation).z;

    c_observations_board[i_observation].icam.intrinsics = icam_intrinsics;
    c_observations_board[i_observation].icam.extrinsics = icam_extrinsics;
    c_observations_board[i_observation].iframe = iframe;
  }
  for (int i_observation = 0; i_observation < Nobservations_point;
       i_observation++) {
    int32_t i_point =
        indices_frame_camintrinsics_camextrinsics.at(i_observation).x;
    int32_t icam_intrinsics =
        indices_frame_camintrinsics_camextrinsics.at(i_observation).y;
    int32_t icam_extrinsics =
        indices_frame_camintrinsics_camextrinsics.at(i_observation).z;

    c_observations_point[i_observation].icam.intrinsics = icam_intrinsics;
    c_observations_point[i_observation].icam.extrinsics = icam_extrinsics;
    c_observations_point[i_observation].i_point = i_point;
  }

  int Ncameras_extrinsics = 0;  // Seems to always be zero for single camera
  int Nframes =
      frames_rt_toref.size(); // Number of pictures of the object we've got
  mrcal_observation_point_triangulated_t *observations_point_triangulated =
      NULL;

  if (!lensmodel_one_validate_args(&mrcal_lensmodel, intrinsics, false))
    return {.success = false};

  mrcal_problem_selections_t problem_selections = construct_problem_selections(
      do_optimize_intrinsics_core, do_optimize_intrinsics_distortions,
      do_optimize_extrinsics, do_optimize_frames, do_optimize_calobject_warp,
      do_apply_regularization, do_apply_outlier_rejection, Ncameras_intrinsics,
      Ncameras_extrinsics, Nframes, Nobservations_board);

  int Nstate = mrcal_num_states(
      Ncameras_intrinsics, Ncameras_extrinsics, Nframes, Npoints, Npoints_fixed,
      Nobservations_board, problem_selections, &mrcal_lensmodel);

  int Nmeasurements = mrcal_num_measurements(
      Nobservations_board, Nobservations_point, observations_point_triangulated,
      0, // hard-coded to 0
      calibration_object_width_n, calibration_object_height_n,
      Ncameras_intrinsics, Ncameras_extrinsics, Nframes, Npoints, Npoints_fixed,
      problem_selections, &mrcal_lensmodel);

  // OK, now we should have everything ready! Just some final setup and then
  // call optimize

  // Outputs
  double c_b_packed_final[Nstate];
  double c_x_final[Nmeasurements];

  // Seeds
  double *c_intrinsics = intrinsics.data();
  mrcal_pose_t *c_extrinsics = extrinsics_rt_fromref;
  mrcal_pose_t *c_frames = frames_rt_toref.data();
  mrcal_point3_t *c_points = points;
  mrcal_calobject_warp_t *c_calobject_warp = &calobject_warp;

  // in
  int *c_imagersizes = imagersize;
  auto point_min_range = -1.0, point_max_range = -1.0;
  mrcal_problem_constants_t problem_constants = {
      .point_min_range = point_min_range, .point_max_range = point_max_range};
  int verbose = 0;

  auto stats = mrcal_optimize(
      c_b_packed_final, Nstate * sizeof(double), c_x_final,
      Nmeasurements * sizeof(double), c_intrinsics, c_extrinsics, c_frames,
      c_points, c_calobject_warp, Ncameras_intrinsics, Ncameras_extrinsics,
      Nframes, Npoints, Npoints_fixed, c_observations_board,
      c_observations_point, Nobservations_board, Nobservations_point,
      observations_point_triangulated, 0, c_observations_board_pool,
      c_observations_point_pool, &mrcal_lensmodel, c_imagersizes,
      problem_selections, &problem_constants, calibration_object_spacing,
      calibration_object_width_n, calibration_object_height_n, verbose, false);

  // Stat prints I copied from Python
  int total_points = calibration_object_width_n * calibration_object_height_n *
                     Nobservations_board;
  // Measurements=corner locations, in pixels. Recall the shape is (num
  // pictures) * (rows x cols in chessboard) * (x, y)

  double max_error = *std::max_element(c_x_final, c_x_final + Nmeasurements);

  // for (int i = 0; i < sizeof(c_x_final); i+= 2) {
  //   mrcal_point2_t &error_pixels =
  //   *reinterpret_cast<mrcal_point2_t*>(c_x_final + i); max_error =
  //   std::max(max_error, std::max(error_pixels.x, error_pixels.y));
  // }



  mrcal_result ret {
    .success = true,
    .intrinsics = intrinsics,
    .rms_error = stats.rms_reproj_error__pixels,
    .residuals = {c_x_final, c_x_final + Nmeasurements},
    .calobject_warp = calobject_warp,
    .Noutliers_board = stats.Noutliers_board,
  };
  return ret;
}

// lifted from mrcal-pywrap.c
static mrcal_problem_selections_t construct_problem_selections(
    int do_optimize_intrinsics_core, int do_optimize_intrinsics_distortions,
    int do_optimize_extrinsics, int do_optimize_frames,
    int do_optimize_calobject_warp, int do_apply_regularization,
    int do_apply_outlier_rejection,

    int Ncameras_intrinsics, int Ncameras_extrinsics, int Nframes,
    int Nobservations_board) {
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
  return {.do_optimize_intrinsics_core = (bool)do_optimize_intrinsics_core,
          .do_optimize_intrinsics_distortions =
              (bool)do_optimize_intrinsics_distortions,
          .do_optimize_extrinsics = (bool)do_optimize_extrinsics,
          .do_optimize_frames = (bool)do_optimize_frames,
          .do_optimize_calobject_warp = (bool)do_optimize_calobject_warp,
          .do_apply_regularization = (bool)do_apply_regularization,
          .do_apply_outlier_rejection = (bool)do_apply_outlier_rejection,
          .do_apply_regularization_unity_cam01 = false};
}

bool lensmodel_one_validate_args(mrcal_lensmodel_t *mrcal_lensmodel,
                                 std::vector<double> intrinsics,
                                 bool do_check_layout) {
  int NlensParams = mrcal_lensmodel_num_params(mrcal_lensmodel);
  int NlensParams_have = intrinsics.size();
  if (NlensParams != NlensParams_have) {
    BARF("intrinsics.shape[-1] MUST be %d. Instead got %d", NlensParams,
         NlensParams_have);
    return false;
  }

  return true;
}

mrcal_pose_t getSeedPose(const mrcal_point3_t *c_observations_board_pool, Size boardSize,
            Size imagerSize, double squareSize, double focal_len_guess) {
  using namespace std;

  if (!c_observations_board_pool) {
    throw runtime_error("board was null");
  }

  double fx = focal_len_guess;
  double fy = fx;
  double cx = (imagerSize.width / 2.0) - 0.5;
  double cy = (imagerSize.height / 2.0) - 0.5;

  vector<Point3f> objectPoints;
  vector<Point2f> imagePoints;

  // Fill in object/image points
  for (int i = 0; i < boardSize.height; i++) {
    for (int j = 0; j < boardSize.width; j++) {
      auto &corner = c_observations_board_pool[i * boardSize.height + j];
      // weight<0 means ignored -- filter these out
      if (corner.z >= 0) {
        imagePoints.emplace_back(corner.x, corner.y);
        objectPoints.push_back(Point3f(j * squareSize, i * squareSize, 0));
      } else {
        std::printf("Ignoring %i,%i!\n", i, j);
      }
    }
  }

  // Initial guess at intrinsics
  Mat cameraMatrix = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  Mat distCoeffs = Mat(4, 1, CV_64FC1, Scalar(0));

  Mat_<double> rvec, tvec;
  vector<Point3f> objectPoints3;
  for (auto a : objectPoints)
    objectPoints3.push_back(Point3f(a.x, a.y, 0));
  solvePnP(objectPoints3, imagePoints, cameraMatrix, distCoeffs, rvec, tvec,
           false, SOLVEPNP_ITERATIVE);

  return mrcal_pose_t{.r = {.x = rvec(0), .y = rvec(1), .z = rvec(2)},
                       .t = {.x = tvec(0), .y = tvec(1), .z = tvec(2)}};
}
