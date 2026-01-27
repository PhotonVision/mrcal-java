#pragma once

#include <mrcal.h>
#include <opencv2/core/types.hpp>
#include <ranges>
#include <fmt/core.h>
#include <fmt/ranges.h> 

using namespace cv;

template <>
struct fmt::formatter<mrcal_point2_t> {
    constexpr auto parse(format_parse_context& ctx) {
        return ctx.begin();
    }

    template <typename FormatContext>
    auto format(const mrcal_point2_t& p, FormatContext& ctx) const {
        return fmt::format_to(ctx.out(), "({}, {})", p.x, p.y);
    }
};

template <>
struct fmt::formatter<mrcal_point3_t> {
    constexpr auto parse(format_parse_context& ctx) {
        return ctx.begin();
    }

    template <typename FormatContext>
    auto format(const mrcal_point3_t& p, FormatContext& ctx) const {
        return fmt::format_to(ctx.out(), "({}, {}, {})", p.x, p.y, p.z);
    }
};

std::vector<mrcal_point2_t> sample_imager(Size numSamples, Size imagerSize) {
    std::vector<mrcal_point2_t> samples;
    samples.reserve(numSamples.width * numSamples.height);
    
    for (int j = 0; j < numSamples.height; j++) {
        for (int i = 0; i < numSamples.width; i++) {
            double x, y;
            
            // linspace formula: start + (stop - start) * i / (n - 1)
            if (numSamples.width == 1) {
                x=0;
            } else {
                x = (imagerSize.width - 1) * i / (numSamples.width - 1.0);
            }
            
            if (numSamples.height == 1) {
                y=0;
            } else {
                y = (imagerSize.height - 1) * j / (numSamples.height - 1.0);
            }
            
            samples.push_back({.x = x, .y = y});
        }
    }
    return samples;
}

// The derivative of q (pixel space location/s) wrt b (state vector)
// at some point this should be a matrix
std::vector<std::vector<double>> _dq_db_projection_uncertainty(
    std::vector<mrcal_point3_t> pcam,
    mrcal_lensmodel_t lensmodel,
    std::vector<double> intrinsics
) {
    // project with gradients
    // model_analysis.py:1067
    std::vector<mrcal_point2_t> q(pcam.size());
    std::vector<mrcal_point3_t> dq_dp(pcam.size()*2);
    std::vector<double> dq_dintrinsics(2*pcam.size()*intrinsics.size()); 
    bool ret = mrcal_project(
        // out
        q.data(),
        dq_dp.data(),
        dq_dintrinsics.data(),
        // in
        pcam.data(), pcam.size(),
        &lensmodel, intrinsics.data()
    );

    if (!ret) {
        throw std::runtime_error("mrcal_project_with_gradients failed");
    }

    fmt::print("q={}\n", q);
    fmt::print("dq_dp={}\n", dq_dp);
    fmt::print("dq_dintrinsics={}\n", dq_dintrinsics);

    // prepare dq_db. Mrcal does this as a 40x60x2xNstate tensor, but we
    // flatten to a 2D array of shape (2*40*60, Nstate)


    return {};
}

bool _propagate_calibration_uncertainty() {
    return true;
}

bool projection_uncertainty(std::vector<mrcal_point3_t> pcam, 
    /* method always mean_pcam, at_infinity always true */
    /* what always worstdirection-stdev */
    mrcal_lensmodel_t lensmodel,
    std::vector<double> intrinsics
    ) {
    auto dq_db {_dq_db_projection_uncertainty(pcam, lensmodel, intrinsics)};
    return _propagate_calibration_uncertainty();
}

void compute_uncertainty() {
    mrcal_lensmodel_t lensmodel;
    lensmodel.type = MRCAL_LENSMODEL_OPENCV8;
    std::vector<double> intrinsics {
        1129.780043, 1134.155189, 602.9912814, 371.6042914, 0.4825503559, -1.184116885, 0.006642882849, -0.003419705443, 2.387477579, 0.3025651431, 0.1612900202, -0.06008374044};

    // hard code some stuff
    auto q = sample_imager({1, 1}, {1280, 720});
    fmt::print("Samples: {}, len={}\n", q[q.size()-1], q.size());

    // and unproject
    std::vector<mrcal_point3_t> pcam;
    pcam.resize(q.size());
    mrcal_unproject(
        pcam.data(), q.data(), q.size(),
        &lensmodel, intrinsics.data()
    );

    // Always normalize, setting rows with any non-finite elements to zero
    for (auto& p : pcam) {
        double nrm = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        if (std::isfinite(nrm) && nrm > 0) {
            p.x /= nrm;
            p.y /= nrm;
            p.z /= nrm;
        }
        else {
            p.x = 0;
            p.y = 0;
            p.z = 0;
        }
    }

    fmt::print("Unprojected: {}, len={}\n", pcam[pcam.size()-1], pcam.size());

    projection_uncertainty(
        pcam,
        lensmodel,
        intrinsics
    );
}
    
