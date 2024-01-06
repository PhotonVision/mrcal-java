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

class CholmodCtx {
public:
  cholmod_common Common, *cc;
  CholmodCtx() {
    cc = &Common;
    cholmod_l_start(cc);
  }

  ~CholmodCtx() { cholmod_l_finish(cc); }
};
static CholmodCtx cctx;

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
mrcal_pose_t
    extrinsics_rt_fromref[0]; // Always zero for single camera, it seems?
mrcal_point3_t points[0];     // Seems to always to be None for single camera?

std::unique_ptr<mrcal_result> mrcal_main(
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
  // mrcal_point3_t *c_observations_point_pool = observations_point;

  // Copy from board/point pool above, using some code borrowed from
  // mrcal-pywrap
  mrcal_observation_board_t c_observations_board[Nobservations_board];
  // Try to make sure we don't accidentally make a zero-length array or
  // something stupid
  mrcal_observation_point_t
      c_observations_point[std::max(Nobservations_point, 1)];

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

  int Ncameras_extrinsics = 0; // Seems to always be zero for single camera
  int Nframes =
      frames_rt_toref.size(); // Number of pictures of the object we've got
  // mrcal_observation_point_triangulated_t *observations_point_triangulated =
  //     NULL;

  if (!lensmodel_one_validate_args(&mrcal_lensmodel, intrinsics, false)) {
    auto ret = std::make_unique<mrcal_result>();
    ret->success = false;
    return ret;
  }

  mrcal_problem_selections_t problem_selections = construct_problem_selections(
      do_optimize_intrinsics_core, do_optimize_intrinsics_distortions,
      do_optimize_extrinsics, do_optimize_frames, do_optimize_calobject_warp,
      do_apply_regularization, do_apply_outlier_rejection, Ncameras_intrinsics,
      Ncameras_extrinsics, Nframes, Nobservations_board);

  int Nstate = mrcal_num_states(
      Ncameras_intrinsics, Ncameras_extrinsics, Nframes, Npoints, Npoints_fixed,
      Nobservations_board, problem_selections, &mrcal_lensmodel);

  int Nmeasurements = mrcal_num_measurements(
      Nobservations_board, Nobservations_point,
      // observations_point_triangulated,
      // 0, // hard-coded to 0
      calibration_object_width_n, calibration_object_height_n,
      Ncameras_intrinsics, Ncameras_extrinsics, Nframes, Npoints, Npoints_fixed,
      problem_selections, &mrcal_lensmodel);

  // OK, now we should have everything ready! Just some final setup and then
  // call optimize

  // Residuals
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
      NULL, -1, c_x_final, Nmeasurements * sizeof(double), c_intrinsics,
      c_extrinsics, c_frames, c_points, c_calobject_warp, Ncameras_intrinsics,
      Ncameras_extrinsics, Nframes, Npoints, Npoints_fixed,
      c_observations_board, c_observations_point, Nobservations_board,
      Nobservations_point, c_observations_board_pool, &mrcal_lensmodel,
      c_imagersizes, problem_selections, &problem_constants,
      calibration_object_spacing, calibration_object_width_n,
      calibration_object_height_n, verbose, false);

  // and for fun, evaluate the jacobian
  // cholmod_sparse* Jt = NULL;
  int N_j_nonzero = _mrcal_num_j_nonzero(
      Nobservations_board, Nobservations_point, calibration_object_width_n,
      calibration_object_height_n, Ncameras_intrinsics, Ncameras_extrinsics,
      Nframes, Npoints, Npoints_fixed, c_observations_board,
      c_observations_point, problem_selections, &mrcal_lensmodel);

  cholmod_sparse* Jt = cholmod_l_allocate_sparse(
      static_cast<size_t>(Nstate), static_cast<size_t>(Nmeasurements),
      static_cast<size_t>(N_j_nonzero), 1, 1, 0, CHOLMOD_REAL, cctx.cc);

  // std::printf("Getting jacobian\n");
  if (!mrcal_optimizer_callback(
          c_b_packed_final, Nstate * sizeof(double), c_x_final,
          Nmeasurements * sizeof(double), Jt, c_intrinsics, c_extrinsics,
          c_frames, c_points, c_calobject_warp, Ncameras_intrinsics,
          Ncameras_extrinsics, Nframes, Npoints, Npoints_fixed,
          c_observations_board, c_observations_point, Nobservations_board,
          Nobservations_point, c_observations_board_pool, &mrcal_lensmodel,
          c_imagersizes, problem_selections, &problem_constants,
          calibration_object_spacing, calibration_object_width_n,
          calibration_object_height_n, verbose)) {
    std::cerr << "callback failed!\n";
  }
  // std::cout << "Jacobian! " << std::endl;

  std::vector<double> residuals = {c_x_final, c_x_final + Nmeasurements};
  return std::make_unique<mrcal_result>(
      true, intrinsics, stats.rms_reproj_error__pixels, residuals, Jt,
      calobject_warp, stats.Noutliers);
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
  return {
      .do_optimize_intrinsics_core =
          static_cast<bool>(do_optimize_intrinsics_core),
      .do_optimize_intrinsics_distortions =
          static_cast<bool>(do_optimize_intrinsics_distortions),
      .do_optimize_extrinsics = static_cast<bool>(do_optimize_extrinsics),
      .do_optimize_frames = static_cast<bool>(do_optimize_frames),
      .do_optimize_calobject_warp =
          static_cast<bool>(do_optimize_calobject_warp),
      .do_apply_regularization = static_cast<bool>(do_apply_regularization),
      .do_apply_outlier_rejection =
          static_cast<bool>(do_apply_outlier_rejection),
      // .do_apply_regularization_unity_cam01 = false
  };
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

mrcal_pose_t getSeedPose(const mrcal_point3_t *c_observations_board_pool,
                         Size boardSize, Size imagerSize, double squareSize,
                         double focal_len_guess) {
  using std::vector, std::runtime_error;

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
           false, SOLVEPNP_EPNP);

  return mrcal_pose_t{.r = {.x = rvec(0), .y = rvec(1), .z = rvec(2)},
                      .t = {.x = tvec(0), .y = tvec(1), .z = tvec(2)}};
}

mrcal_result::~mrcal_result() {
  cholmod_l_free_sparse(&Jt, cctx.cc);
  // free(Jt.p);
  // free(Jt.i);
  // free(Jt.x);
  return;
}

bool unproject_mrcal(cv::Mat& src, cv::Mat& dst, cv::Mat& cameraMat, cv::Mat& distCoeffs, CameraLensModel lensModel,
  // Extra stuff for splined stereographic models
  uint16_t order,
  uint16_t Nx,
  uint16_t Ny,
  uint16_t fov_x_deg
) {
  
  mrcal_lensmodel_t mrcal_lensmodel;
  switch (lensModel) {
    case CameraLensModel::LENSMODEL_OPENCV5:
      mrcal_lensmodel.type = MRCAL_LENSMODEL_OPENCV5;
      break;
    case CameraLensModel::LENSMODEL_OPENCV8:
      mrcal_lensmodel.type = MRCAL_LENSMODEL_OPENCV8;
      break;
    case CameraLensModel::LENSMODEL_STEREOGRAPHIC:
      mrcal_lensmodel.type = MRCAL_LENSMODEL_STEREOGRAPHIC;
      break;
    case CameraLensModel::LENSMODEL_SPLINED_STEREOGRAPHIC:
      mrcal_lensmodel.type = MRCAL_LENSMODEL_SPLINED_STEREOGRAPHIC;

      /* Maximum degree of each 1D polynomial. This is almost certainly 2 */          
      mrcal_lensmodel.LENSMODEL_SPLINED_STEREOGRAPHIC__config.fov_x_deg = fov_x_deg;
      /* The horizontal field of view. Not including fov_y. It's proportional with */ 
      /* Ny and Nx */                                                                 
      mrcal_lensmodel.LENSMODEL_SPLINED_STEREOGRAPHIC__config.order = order;
      /* We have a Nx by Ny grid of control points */                                 
      mrcal_lensmodel.LENSMODEL_SPLINED_STEREOGRAPHIC__config.Nx = Nx;
      mrcal_lensmodel.LENSMODEL_SPLINED_STEREOGRAPHIC__config.Ny = Ny;
      break;
    default:
      std::cerr << "Unknown lensmodel\n";
      return false;
  }

  if (!(dst.cols == 2 && src.cols == 2 && dst.rows == src.rows)) {
      std::cerr << "Bad input array size\n";
      return false;
  }
  if (!(dst.type() == CV_64FC2 && src.type() == CV_64FC2)) {
      std::cerr << "Bad input type -- need CV_64F\n";
      return false;
  }
  if (!(dst.isContinuous() && src.isContinuous())) {
      std::cerr << "Bad input array -- need continuous\n";
      return false;
  }

  // extract intrinsics core from opencv camera matrix
  double fx = cameraMat.at<double>(0, 0);
  double fy = cameraMat.at<double>(1, 1);
  double cx = cameraMat.at<double>(0, 2);
  double cy = cameraMat.at<double>(1, 2);

  // Core, distortion coefficients concatenated
  int NlensParams = mrcal_lensmodel_num_params(&mrcal_lensmodel);
  std::vector<double> intrinsics(NlensParams);
  intrinsics[0] = fx;
  intrinsics[1] = fy;
  intrinsics[2] = cx;
  intrinsics[3] = cy;
  for (size_t i = 0; i < distCoeffs.cols; i++) {
    intrinsics[i + 4] = distCoeffs.at<double>(i);
  }

  // input points in the distorted image pixel coordinates
  mrcal_point2_t* in = reinterpret_cast<mrcal_point2_t*>(src.data);
  // vec3 observation vectors defined up-to-length
  std::vector<mrcal_point3_t> out(src.rows);



  mrcal_unproject(
    out.data(), in,
    src.rows,
    &mrcal_lensmodel,
    intrinsics.data()
  );

  // The output is defined "up-to-length"
  // Let's project through pinhole again

  // Output points in pinhole pixel coordinates
  mrcal_point2_t* pinhole_pts = reinterpret_cast<mrcal_point2_t*>(dst.data);


  size_t bound = src.rows;
  for (size_t i = 0; i < bound; i++) {
    // from mrcal-project-internal/pinhole model
    mrcal_point3_t& p = out[i];

    double z_recip = 1./p.z;
    double x = p.x * z_recip;
    double y = p.y * z_recip;

    pinhole_pts[i].x = x * fx + cx;
    pinhole_pts[i].y = y * fy + cy;
  }

  return true;
}
