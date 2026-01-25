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
                x = (imagerSize.width - 1) / 2.0;
            } else {
                x = (imagerSize.width - 1) * i / (numSamples.width - 1.0);
            }
            
            if (numSamples.height == 1) {
                y = (imagerSize.height - 1) / 2.0;
            } else {
                y = (imagerSize.height - 1) * j / (numSamples.height - 1.0);
            }
            
            samples.push_back({.x = x, .y = y});
        }
    }
    return samples;
}

void compute_uncertainty() {
    mrcal_lensmodel_t lensmodel;
    lensmodel.type = MRCAL_LENSMODEL_OPENCV8;
    std::vector<double> intrinsics {
        1088.363893, 1095.830102, 608.3108394, 355.4729472, 0.2922172061, -1.429731317, -0.001481463652, 0.004184715836, 2.570603373, 0.0639153706, 0.07765775163, -0.09689803946,};

    // hard code some stuff
    auto q = sample_imager({60, 40}, {1280, 720});
    fmt::print("Samples: {}, len={}\n", q, q.size());

    // and unproject
    std::vector<mrcal_point3_t> out;
    out.reserve(q.size());
    mrcal_unproject(
        out.data(), q.data(), q.size(),
        &lensmodel, intrinsics.data()
    );
    fmt::print("Unprojected: {}, len={}\n", out, out.size());
}
    
