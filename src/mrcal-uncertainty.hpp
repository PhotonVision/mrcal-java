#pragma once

#include <mrcal.h>
#include <opencv2/core/types.hpp>
#include <ranges>
#include <fmt/core.h>
#include <fmt/ranges.h> 

using namespace cv;

std::vector<mrcal_point3_t> sample_imager(Size numSamples, Size imagerSize) {
    // using std::ranges, sample the imager with numSamples points per axis
    std::vector<mrcal_point3_t> samples;
    for (int i = 0; i < numSamples.width; i++) {
        for (int j = 0; j < numSamples.height; j++) {
            double x = (i + 0.5) * imagerSize.width / numSamples.width;
            double y = (j + 0.5) * imagerSize.height / numSamples.height;
            samples.push_back({.x = x, .y = y, .z = 1.0});
        }
    }
    return samples;
}

void compute_uncertainty() {
    // hard code some stuff
    auto q = sample_imager({60, 40}, {1280, 720});
    fmt::print("Samples: {}\n", q);
}
    
