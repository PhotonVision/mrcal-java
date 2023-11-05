#include <malloc.h>
#include <stdint.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <span>
#include <stdexcept>
#include <vector>

#include "mrcal_wrapper.h"

using namespace cv;

extern "C" {
#include "../vnlog/vnlog-parser.h"
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
  // Size boardSize = {7, 7};
  // Size imagerSize = {640, 480};
  // std::FILE *fp =
  //     std::fopen("/home/matt/github/photon_640_480/corners.vnl", "r");
  Size boardSize = {10, 10};
  Size imagerSize = {1600, 896};
  std::FILE *fp = std::fopen("/home/matt/github/c920_cal/corners.vnl", "r");

  if (fp == NULL)
    return -1;

  vnlog_parser_t ctx;
  if (VNL_OK != vnlog_parser_init(&ctx, fp))
    return -1;

  std::map<std::string, std::vector<mrcal_point3_t>> points;
  int i_record = 0;
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
      using namespace std;
      double level = stod(*level_);
      double weight;
      if (level < 0) {
        weight = -1;
      } else {
        weight = std::pow(0.5, level);
      }
      points[*name].push_back(mrcal_point3_t{stod(*x), stod(*y), weight});
      // std::printf("put %s\n", *name);
    } catch (std::exception e) {
    }
  }

  vnlog_parser_free(&ctx);

  std::vector<mrcal_point3_t> observations_board;
  std::vector<mrcal_pose_t> frames_rt_toref;

  for (const auto &[key, value] : points) {
    if (value.size()) {
      auto [ret, imagePoints] =
          getSeedPose(value.data(), boardSize, imagerSize);
      std::printf("Seed pose %s: r %f %f %f t %f %f %f\n", key.c_str(), ret.r.x,
                  ret.r.y, ret.r.z, ret.t.x, ret.t.y, ret.t.z);

      // Append to the Big List of board corners/levels
      observations_board.insert(observations_board.end(), value.begin(),
                                value.end());
      // And list of pose seeds
      frames_rt_toref.push_back(ret);
    } else {
      std::printf("No points for %s\n", key.c_str());
    }
  }

  printf("hi 1!\n");
  auto cal_result = mrcal_main(observations_board, frames_rt_toref, boardSize, 0.0254,
                    imagerSize);
  printf("hi 898!\n");
  return true;
}

int main() { homography_test(); }
