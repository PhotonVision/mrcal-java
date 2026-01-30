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

#include "mrcal-uncertainty.hpp"
#include <chrono>
#include <memory>
#include <utility>
#include <vector>

using namespace cv;

template <> struct fmt::formatter<mrcal_point2_t> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const mrcal_point2_t &p, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "({}, {})", p.x, p.y);
  }
};

template <> struct fmt::formatter<mrcal_point3_t> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const mrcal_point3_t &p, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "({}, {}, {})", p.x, p.y, p.z);
  }
};

using EigenPoint2 = Eigen::Matrix<double, 2, 1>;
using EigenPoint3 = Eigen::Matrix<double, 3, 1>;

// 10.5 seconds. definitely still CPU bound
// using SolverType = Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double,
// Eigen::ColMajor>>;

// 10.5 seconds
using SolverType =
    Eigen::SparseLU<Eigen::SparseMatrix<double, Eigen::ColMajor>>;

// 11 seconds
// using SolverType = Eigen::SimplicialLLT<Eigen::SparseMatrix<double,
// Eigen::ColMajor>>;

struct CalibrationUncertaintyContext {
  std::unique_ptr<SolverType> solver;
  Eigen::SparseMatrix<double, Eigen::ColMajor>
      J_observations; // J matrix for board observations only
  double observed_pixel_uncertainty;
  int Nstate;
  int Nmeasurements_boards;
};

static inline double worst_direction_stdev(const Eigen::Matrix2d &cov) {
  // Direct formula for 2x2: sqrt((a+c)/2 + sqrt((a-c)^2/4 + b^2))
  double a = cov(0, 0);
  double b = cov(1, 0);
  double c = cov(1, 1);

  return std::sqrt((a + c) / 2.0 + std::sqrt((a - c) * (a - c) / 4.0 + b * b));
}

static std::vector<mrcal_point2_t> sample_imager(Size numSamples,
                                                 Size imagerSize) {
  std::vector<mrcal_point2_t> samples;
  samples.reserve(numSamples.width * numSamples.height);

  for (int j = 0; j < numSamples.height; j++) {
    for (int i = 0; i < numSamples.width; i++) {
      double x, y;

      // linspace formula: start + (stop - start) * i / (n - 1)
      if (numSamples.width == 1) {
        x = 0;
      } else {
        x = (imagerSize.width - 1) * i / (numSamples.width - 1.0);
      }

      if (numSamples.height == 1) {
        y = 0;
      } else {
        y = (imagerSize.height - 1) * j / (numSamples.height - 1.0);
      }

      samples.push_back({.x = x, .y = y});
    }
  }
  return samples;
}

// Compute dq_db for multiple points at once
// Returns matrix of shape (2*Npoints, Nstate) where each pair of rows
// corresponds to one point
static Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
_dq_db_projection_uncertainty_batched(std::span<mrcal_point3_t> pcam_points,
                                      mrcal_lensmodel_t lensmodel,
                                      std::span<mrcal_pose_t> rt_ref_frame,
                                      int Nstate, int istate_frames0,
                                      std::span<double> intrinsics) {
  const size_t Npoints = pcam_points.size();
  const size_t Nboards = rt_ref_frame.size();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      dq_db_all(2 * Npoints, Nstate);
  dq_db_all.setZero();

  for (size_t pt = 0; pt < Npoints; pt++) {
    const mrcal_point3_t &pcam = pcam_points[pt];
    // Project with gradients
    mrcal_point2_t q{};
    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> dq_dpcam;
    std::vector<double> dq_dintrinsics(2 * intrinsics.size());

    bool ret = mrcal_project(
        &q, reinterpret_cast<mrcal_point3_t *>(dq_dpcam.data()),
        dq_dintrinsics.data(), &pcam, 1, &lensmodel, intrinsics.data());
    if (!ret) {
      throw std::runtime_error("mrcal_project failed");
    }

    // p_ref = pcam (identity transform from cam to ref)
    Eigen::Matrix<double, 1, 3> p_ref{pcam.x, pcam.y, pcam.z};

    // Set intrinsics portion
    Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor>>
        dq_dintrinsics_eigen(dq_dintrinsics.data(), 2, intrinsics.size());
    dq_db_all.block(2 * pt, 0, 2, intrinsics.size()) = dq_dintrinsics_eigen;

    // Compute dpcam_dpref (identity rotation case)
    Eigen::Matrix<double, 3, 3> dpcam_dpref;
    Eigen::Matrix<double, 3, 3> dpcam_dr;
    {
      Eigen::Matrix<double, 1, 6> rt_cam_ref;
      rt_cam_ref.setZero();
      mrcal_point3_t rotated_p_ref;
      mrcal_rotate_point_r(rotated_p_ref.xyz, dpcam_dr.data(),
                           dpcam_dpref.data(), rt_cam_ref.data(), p_ref.data());
    }

    Eigen::Matrix<double, 2, 3> dq_dpref = dq_dpcam * dpcam_dpref;

    // Compute p_frames and dpref_dframes for each board
    for (size_t pose = 0; pose < Nboards; pose++) {
      Eigen::Matrix<double, 1, 3> p_frame;
      mrcal_rotate_point_r_inverted(p_frame.data(), NULL, NULL,
                                    rt_ref_frame[pose].r.xyz, p_ref.data());

      Eigen::Matrix<double, 3, 3, Eigen::RowMajor> dpref_dframe;
      mrcal_point3_t dummy;
      mrcal_rotate_point_r(dummy.xyz, dpref_dframe.data(), NULL,
                           rt_ref_frame[pose].r.xyz, p_frame.data());

      Eigen::Matrix<double, 2, 3> dq_dframe = dq_dpref * dpref_dframe;

      int frame_start = istate_frames0 + pose * 6;
      dq_db_all.block(2 * pt, frame_start, 2, 3) =
          dq_dframe / static_cast<double>(Nboards);
    }
  }

  return dq_db_all;
}

static double
_observed_pixel_uncertainty_from_inputs(std::vector<double> &x,
                                        int num_measurements_board,
                                        int measurement_index_board) {
  // Compute variance from residuals
  double sum = 0.0, sum_sq = 0.0;
  for (size_t i = measurement_index_board;
       i < measurement_index_board + num_measurements_board; i++) {
    double val = x[i];
    sum += val;
    sum_sq += val * val;
  }
  double mean = sum / x.size();
  double variance = (sum_sq / x.size()) - (mean * mean);

  return std::sqrt(variance);
}

static CalibrationUncertaintyContext create_calibration_uncertainty_context(
    mrcal_problem_selections_t &problem_selections,
    mrcal_lensmodel_t &lensmodel, const std::span<double> intrinsics,
    const std::span<mrcal_pose_t> rt_ref_frame,
    const mrcal_observation_board_t *observations_board,
    const mrcal_point3_t *observations_board_pool, int Nobservations_board,
    int calibration_object_width_n, int calibration_object_height_n,
    double calibration_object_spacing, cv::Size imagerSize,
    mrcal_calobject_warp_t warp) {
  int Nstate =
      mrcal_num_states(1, 0, rt_ref_frame.size(), 0, 0, Nobservations_board,
                       problem_selections, &lensmodel);

  int Nmeasurements = mrcal_num_measurements(
      Nobservations_board, 0, NULL, 0, calibration_object_width_n,
      calibration_object_height_n, 1, 0, 6, 0, 0, problem_selections,
      &lensmodel);
  int Nmeasurements_boards = mrcal_num_measurements_boards(
      Nobservations_board, calibration_object_width_n,
      calibration_object_height_n);
  int imeas0 = mrcal_measurement_index_boards(0, Nobservations_board, 0,
                                              calibration_object_width_n,
                                              calibration_object_height_n);

  // Allocate buffers for Jt sparse matrix
  int N_j_nonzero = Nstate * Nmeasurements; // Upper bound, actual will be less
  std::vector<int32_t> Jt_p(Nmeasurements + 1);
  std::vector<int32_t> Jt_i(N_j_nonzero);
  std::vector<double_t> Jt_x(N_j_nonzero);

  cholmod_sparse Jt = {.nrow = static_cast<size_t>(Nstate),
                       .ncol = static_cast<size_t>(Nmeasurements),
                       .nzmax = static_cast<size_t>(N_j_nonzero),
                       .p = Jt_p.data(),
                       .i = Jt_i.data(),
                       .nz = nullptr,
                       .x = Jt_x.data(),
                       .z = nullptr,
                       .stype = 0,
                       .itype = CHOLMOD_INT,
                       .xtype = CHOLMOD_REAL,
                       .dtype = CHOLMOD_DOUBLE,
                       .sorted = 1,
                       .packed = 1};

  std::vector<double> b_packed(Nstate);
  std::vector<double> x(Nmeasurements);

  double point_min_range = -1.0, point_max_range = -1.0;
  mrcal_problem_constants_t problem_constants = {
      .point_min_range = point_min_range, .point_max_range = point_max_range};

  int imagersize[2]{imagerSize.width, imagerSize.height};

  bool ret = mrcal_optimizer_callback(
      b_packed.data(), b_packed.size() * sizeof(double), // out
      x.data(), x.size() * sizeof(double),               // out
      &Jt,                                               // Get the Jacobian
      // IN parameters
      intrinsics.data(),
      nullptr, // no extrinsics
      rt_ref_frame.data(),
      nullptr, // no points
      &warp, 1, 0, static_cast<int>(rt_ref_frame.size()), 0, 0,
      observations_board, nullptr, Nobservations_board, 0, nullptr, 0,
      observations_board_pool, nullptr, &lensmodel, imagersize,
      problem_selections, &problem_constants, calibration_object_spacing,
      calibration_object_width_n, calibration_object_height_n, false);

  if (!ret) {
    throw std::runtime_error("mrcal_optimizer_callback failed");
  }

  double observed_pixel_uncertainty =
      _observed_pixel_uncertainty_from_inputs(x, Nmeasurements_boards, imeas0);

  // Convert CHOLMOD sparse Jt to Eigen sparse matrix
  using SpMat = Eigen::SparseMatrix<double, Eigen::ColMajor>;
  SpMat Jt_eigen(Nstate, Nmeasurements);

  {
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(N_j_nonzero);

    for (int col = 0; col < Nmeasurements; col++) {
      for (int p = Jt_p[col]; p < Jt_p[col + 1]; p++) {
        int row = Jt_i[p];
        double val = Jt_x[p];
        triplets.emplace_back(row, col, val);
      }
    }

    Jt_eigen.setFromTriplets(triplets.begin(), triplets.end());
  }

  // J = Jt^T has shape (Nmeasurements, Nstate)
  SpMat J = Jt_eigen.transpose();

  // Compute JtJ = Jt * J (shape Nstate x Nstate)
  SpMat JtJ = Jt_eigen * J;

  // check positive definite happens for free during compute/info

  // Pre-compute the factorization
  auto solver = std::make_unique<SolverType>();
  solver->compute(JtJ);

  if (solver->info() != Eigen::Success) {
    throw std::runtime_error("Eigen factorization failed");
  }

  // Store the observation rows of J for fast uncertainty propagation
  SpMat J_observations = J.topRows(Nmeasurements_boards);

  return CalibrationUncertaintyContext{
      .solver = std::move(solver),
      .J_observations = std::move(J_observations),
      .observed_pixel_uncertainty = observed_pixel_uncertainty,
      .Nstate = Nstate,
      .Nmeasurements_boards = Nmeasurements_boards};
}

// Propagate uncertainty for multiple points at once
// dF_dbunpacked has shape (2*Npoints, Nstate)
// Returns vector of worst-direction stdevs for each point
static std::vector<double> _propagate_calibration_uncertainty_batched(
    const CalibrationUncertaintyContext &context,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        &dF_dbunpacked,
    mrcal_problem_selections_t &problem_selections,
    mrcal_lensmodel_t &lensmodel, const std::span<double> intrinsics,
    const std::span<mrcal_pose_t> rt_ref_frame, int Nobservations_board) {

  const int Npoints = dF_dbunpacked.rows() / 2;
  const int Nstate = dF_dbunpacked.cols();

  // Pack the gradient (in-place modification)
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      dF_dbpacked = dF_dbunpacked;
  for (int i = 0; i < dF_dbpacked.rows(); i++) {
    mrcal_unpack_solver_state_vector(
        dF_dbpacked.row(i).data(), 1, 0, rt_ref_frame.size(), 0, 0,
        Nobservations_board, problem_selections, &lensmodel);
  }

  // Solve JtJ * A = dF_dbpacked^T for ALL points at once
  // rhs shape: (Nstate, 2*Npoints)
  Eigen::MatrixXd rhs = dF_dbpacked.transpose();
  Eigen::MatrixXd A = context.solver->solve(rhs); // (Nstate, 2*Npoints)

  if (context.solver->info() != Eigen::Success) {
    throw std::runtime_error("Eigen solve failed");
  }

  // Compute J_obs * A for all points: (Nmeas_obs, 2*Npoints)
  Eigen::MatrixXd JA = context.J_observations * A;

  // Extract per-point uncertainties
  std::vector<double> uncertainties(Npoints);
  for (int pt = 0; pt < Npoints; pt++) {
    // Extract the 2 columns for this point
    Eigen::MatrixXd JA_pt = JA.middleCols(2 * pt, 2); // (Nmeas_obs, 2)

    // Compute Var(F) = JA_pt^T * JA_pt
    Eigen::Matrix2d Var_dF = JA_pt.transpose() * JA_pt;

    uncertainties[pt] =
        worst_direction_stdev(Var_dF) * context.observed_pixel_uncertainty;
  }

  return uncertainties;
}

// Compute projection uncertainty for multiple points at once
static std::vector<double> projection_uncertainty_batched(
    const CalibrationUncertaintyContext &context,
    std::span<mrcal_point3_t> pcam_points, mrcal_lensmodel_t lensmodel,
    std::span<mrcal_pose_t> rt_ref_frames, std::span<double> intrinsics) {

  mrcal_problem_selections_t problem_selections{0};
  problem_selections.do_optimize_intrinsics_core = true;
  problem_selections.do_optimize_intrinsics_distortions = true;
  problem_selections.do_optimize_extrinsics = true;
  problem_selections.do_optimize_frames = true;
  problem_selections.do_optimize_calobject_warp = true;
  problem_selections.do_apply_regularization = true;
  problem_selections.do_apply_outlier_rejection = true;
  problem_selections.do_apply_regularization_unity_cam01 = false;

  int istate_frames0 = mrcal_state_index_frames(0, 1, 0, 6, 0, 0, 6,
                                                problem_selections, &lensmodel);
  int Nobservations_board = rt_ref_frames.size();

  auto dq_db = _dq_db_projection_uncertainty_batched(
      pcam_points, lensmodel, rt_ref_frames, context.Nstate, istate_frames0,
      intrinsics);

  return _propagate_calibration_uncertainty_batched(
      context, dq_db, problem_selections, lensmodel, intrinsics, rt_ref_frames,
      Nobservations_board);
}

double projection_uncertainty_fast(const CalibrationUncertaintyContext &context,
                                   mrcal_point3_t pcam,
                                   mrcal_lensmodel_t lensmodel,
                                   std::span<mrcal_pose_t> rt_ref_frames,
                                   std::span<double> intrinsics) {
  // Single-point version - consider using batched version for better
  // performance
  std::vector<mrcal_point3_t> pcam_vec = {pcam};
  auto results = projection_uncertainty_batched(context, pcam_vec, lensmodel,
                                                rt_ref_frames, intrinsics);
  return results[0];
}

std::vector<mrcal_point3_t> compute_uncertainty(
    std::span<mrcal_point3_t> observations_board, std::span<double> intrinsics,
    std::span<mrcal_pose_t> rt_ref_frames, mrcal_calobject_warp_t warp,
    cv::Size imagerSize, cv::Size calobjectSize, double calobjectSpacing,
    cv::Size sampleResolution) {

  mrcal_lensmodel_t lensmodel;
  lensmodel.type = MRCAL_LENSMODEL_OPENCV8;

  // Create calibration uncertainty context once
  mrcal_problem_selections_t problem_selections{0};
  problem_selections.do_optimize_intrinsics_core = true;
  problem_selections.do_optimize_intrinsics_distortions = true;
  problem_selections.do_optimize_extrinsics = true;
  problem_selections.do_optimize_frames = true;
  problem_selections.do_optimize_calobject_warp = true;
  problem_selections.do_apply_regularization = true;
  problem_selections.do_apply_outlier_rejection = true;
  problem_selections.do_apply_regularization_unity_cam01 = false;

  std::vector<mrcal_point3_t> indices_frame_camintrinsics_camextrinsics;
  for (int i = 0; i < rt_ref_frames.size(); i++) {
    indices_frame_camintrinsics_camextrinsics.push_back(
        {.x = static_cast<double>(i), .y = 0, .z = -1});
  }

  std::vector<mrcal_observation_board_t> observations_board_data(
      rt_ref_frames.size());
  auto c_observations_board = observations_board_data.data();

  for (int i_observation = 0; i_observation < rt_ref_frames.size();
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

  auto context = create_calibration_uncertainty_context(
      problem_selections, lensmodel, intrinsics, rt_ref_frames,
      c_observations_board, observations_board.data(), rt_ref_frames.size(),
      calobjectSize.width, calobjectSize.height, calobjectSpacing, imagerSize,
      warp);

  auto q = sample_imager(sampleResolution, imagerSize);

  // and unproject
  std::vector<mrcal_point3_t> pcam;
  pcam.resize(q.size());
  mrcal_unproject(pcam.data(), q.data(), q.size(), &lensmodel,
                  intrinsics.data());

  // Always normalize, setting rows with any non-finite elements to zero
  for (auto &p : pcam) {
    double nrm = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (std::isfinite(nrm) && nrm > 0) {
      p.x /= nrm;
      p.y /= nrm;
      p.z /= nrm;
    } else {
      p.x = 0;
      p.y = 0;
      p.z = 0;
    }
  }

  std::vector<double> uncertainties = projection_uncertainty_batched(
      context, pcam, lensmodel, rt_ref_frames, intrinsics);

  // Build result
  std::vector<mrcal_point3_t> ret;
  ret.reserve(pcam.size());
  for (size_t i = 0; i < pcam.size(); i++) {
    ret.push_back({q[i].x, q[i].y, uncertainties[i]});
  }

  return ret;
}
