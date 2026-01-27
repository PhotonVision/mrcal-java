#pragma once

#include <mrcal.h>
#include <poseutils.h>
#include <opencv2/core/types.hpp>
#include <ranges>
#include <fmt/core.h>
#include <fmt/ranges.h>
// #include "EigenFormat.hpp"
#include <Eigen/Dense>

using namespace cv;

template <>
struct fmt::formatter<mrcal_point2_t>
{
    constexpr auto parse(format_parse_context &ctx)
    {
        return ctx.begin();
    }

    template <typename FormatContext>
    auto format(const mrcal_point2_t &p, FormatContext &ctx) const
    {
        return fmt::format_to(ctx.out(), "({}, {})", p.x, p.y);
    }
};

template <>
struct fmt::formatter<mrcal_point3_t>
{
    constexpr auto parse(format_parse_context &ctx)
    {
        return ctx.begin();
    }

    template <typename FormatContext>
    auto format(const mrcal_point3_t &p, FormatContext &ctx) const
    {
        return fmt::format_to(ctx.out(), "({}, {}, {})", p.x, p.y, p.z);
    }
};

using EigenPoint2 = Eigen::Matrix<double, 2, 1>;
using EigenPoint3 = Eigen::Matrix<double, 3, 1>;

std::vector<mrcal_point2_t> sample_imager(Size numSamples, Size imagerSize)
{
    std::vector<mrcal_point2_t> samples;
    samples.reserve(numSamples.width * numSamples.height);

    for (int j = 0; j < numSamples.height; j++)
    {
        for (int i = 0; i < numSamples.width; i++)
        {
            double x, y;

            // linspace formula: start + (stop - start) * i / (n - 1)
            if (numSamples.width == 1)
            {
                x = 0;
            }
            else
            {
                x = (imagerSize.width - 1) * i / (numSamples.width - 1.0);
            }

            if (numSamples.height == 1)
            {
                y = 0;
            }
            else
            {
                y = (imagerSize.height - 1) * j / (numSamples.height - 1.0);
            }

            samples.push_back({.x = x, .y = y});
        }
    }
    return samples;
}

// The derivative of q (pixel space location/s) wrt b (state vector)
// at some point this should be a matrix
std::vector<double> _dq_db_projection_uncertainty(
    mrcal_point3_t pcam,
    mrcal_lensmodel_t lensmodel,
    std::vector<mrcal_pose_t> rt_ref_frame,
    int Nstate,
    std::vector<double>& intrinsics)
{
    // project with gradients
    // model_analysis.py:1067
    mrcal_point2_t q{};
    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> dq_dpcam;
    std::vector<double> dq_dintrinsics(2 * 1ull * intrinsics.size());
    bool ret = mrcal_project(
        // out
        &q,
        reinterpret_cast<mrcal_point3_t*>(dq_dpcam.data()),
        dq_dintrinsics.data(),
        // in
        &pcam, 1,
        &lensmodel, intrinsics.data());

    if (!ret)
    {
        throw std::runtime_error("mrcal_project_with_gradients failed");
    }

    // number of boards
    const size_t Nboards {rt_ref_frame.size()};

    // p_ref = pcam rotated by r (always zero1)
    Eigen::Matrix<double, 1, 3> p_ref {pcam.x, pcam.y, pcam.z};

    fmt::print("_dq_db_projection_uncertainty: ==========\n");
    fmt::print("q={}\n", q);
    std::cout << "dq_dpcam:\n" << dq_dpcam << "\n";
    fmt::print("dq_dintrinsics={}\n", dq_dintrinsics);

    // prepare dq_db. Mrcal does this as a 40x60x2xNstate tensor, but we
    // are only projecting one point
    Eigen::Matrix<double, 2, Eigen::Dynamic> dq_db(2, Nstate);
    dq_db.setZero();

    // set dq_db[0:num intrinsics] = [dq_dintrinsics]
    Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor>> dq_dintrinsics_eigen(
        dq_dintrinsics.data(), 2, intrinsics.size());
    dq_db.leftCols(intrinsics.size()) = dq_dintrinsics_eigen;

    // determine dpcam_dr and dpcamp_dpref
    Eigen::Matrix<double, 3, 3> dpcam_dpref; //dxout/dxin
    Eigen::Matrix<double, 3, 3> dpcam_dr; // dx_out/dr
    {
        // HACK -- hard-coded to origin
        Eigen::Matrix<double, 1, 6> rt_cam_ref;
        rt_cam_ref.setZero();


        // output arrays
        mrcal_point3_t rotated_p_ref;

        mrcal_rotate_point_r(
            // out
            rotated_p_ref.xyz,
            dpcam_dr.data(),
            dpcam_dpref.data(),
            // in
            rt_cam_ref.data(),
            p_ref.data()
        );
    }

    // method is always mean-pcam
    auto dq_dpref = dq_dpcam * dpcam_dpref; 

    // calculate p_frames
    {
        // output arrays
        Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> p_frames(Nboards, 3);

        for (size_t pose = 0; pose < Nboards; pose++)
        {
            auto p_frame_i = p_frames.row(pose);

            mrcal_rotate_point_r(
                // out
                p_frame_i.data(), 
                NULL, NULL,
                // in
                rt_ref_frame[pose].data(),
                p_ref.data()
            );
        }

    }


    std::cout << "dq_db:\n" << dq_db << "\n";
    std::cout << "dq_dpref:\n" << dq_dpref << "\n";

    return {};
}

bool _propagate_calibration_uncertainty()
{
    return true;
}

bool projection_uncertainty(mrcal_point3_t pcam,
                            /* method always mean_pcam, at_infinity always true */
                            /* what always worstdirection-stdev */
                            mrcal_lensmodel_t lensmodel,
                            std::vector<double> intrinsics)
{

    // Prepare inputs
    mrcal_problem_selections_t problem_selections{0};
    problem_selections.do_optimize_intrinsics_core = true;
    problem_selections.do_optimize_intrinsics_distortions = true;
    problem_selections.do_optimize_extrinsics = true;
    problem_selections.do_optimize_frames = true;
    problem_selections.do_optimize_calobject_warp = true;
    problem_selections.do_apply_regularization = true;
    problem_selections.do_apply_outlier_rejection = true;
    problem_selections.do_apply_regularization_unity_cam01 = false;

    int Nstate = mrcal_num_states(
        1, // Ncameras_intrinsics
        0, // Ncameras_extrinsics
        6, // Nframes
        0, // Npoints
        0, // Npoints_fixed
        6, // Nobservations_board
        problem_selections,
        &lensmodel);

    // hard code reference frame transformations
    std::vector<mrcal_pose_t> rt_ref_frames {
        {-0.13982929, -0.37331785, -0.01785786, -0.15373499, -0.13686309, , 0.59757725},
        {-0.18951098, -0.50825451, , 0.02212706, -0.26111978, -0.10816078, , 0.58305005}, 
        {-0.09580704, -0.4029582, , -0.01730795, -0.17221784, -0.17518785, , 0.58390618}, 
        {-0.08038678, -0.21667501, -0.00719197, -0.05143006, -0.17069075, , 0.59817879}, 
        {-0.11562071, -0.19945448, -0.02150643, -0.02264116, -0.15749109, 0.59493056}, 
        {-0.14950876, -0.05920069,  0.0375357,   0.08856689, -0.10811448,  0.59142776}
    };

    auto dq_db{_dq_db_projection_uncertainty(pcam, lensmodel, rt_ref_frames, Nstate, intrinsics)};
    return _propagate_calibration_uncertainty();
}

void compute_uncertainty()
{
    mrcal_lensmodel_t lensmodel;
    lensmodel.type = MRCAL_LENSMODEL_OPENCV8;
    std::vector<double> intrinsics{
        1129.780043, 1134.155189, 602.9912814, 371.6042914, 0.4825503559, -1.184116885, 0.006642882849, -0.003419705443, 2.387477579, 0.3025651431, 0.1612900202, -0.06008374044};

    // hard code some stuff
    auto q = sample_imager({1, 1}, {1280, 720});
    fmt::print("Samples: {}, len={}\n", q[q.size() - 1], q.size());

    // and unproject
    std::vector<mrcal_point3_t> pcam;
    pcam.resize(q.size());
    mrcal_unproject(
        pcam.data(), q.data(), q.size(),
        &lensmodel, intrinsics.data());

    // Always normalize, setting rows with any non-finite elements to zero
    for (auto &p : pcam)
    {
        double nrm = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        if (std::isfinite(nrm) && nrm > 0)
        {
            p.x /= nrm;
            p.y /= nrm;
            p.z /= nrm;
        }
        else
        {
            p.x = 0;
            p.y = 0;
            p.z = 0;
        }
    }

    fmt::print("Unprojected: {}, len={}\n", pcam[pcam.size() - 1], pcam.size());

    for (const auto &p : pcam)
    {
        fmt::print("pcam: {}\n", p);

        projection_uncertainty(
            p,
            lensmodel,
            intrinsics);
    }
}
