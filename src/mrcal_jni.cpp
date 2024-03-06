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

#include "mrcal_jni.h"

#include <algorithm>
#include <cstdio>
#include <span>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "mrcal_wrapper.h"

// JClass helper from wpilib
#define WPI_JNI_MAKEJARRAY(T, F)                                               \
  inline T##Array MakeJ##F##Array(JNIEnv *env, T *data, size_t size) {         \
    T##Array jarr = env->New##F##Array(size);                                  \
    if (!jarr) {                                                               \
      return nullptr;                                                          \
    }                                                                          \
    env->Set##F##ArrayRegion(jarr, 0, size, data);                             \
    return jarr;                                                               \
  }

WPI_JNI_MAKEJARRAY(jboolean, Boolean)
WPI_JNI_MAKEJARRAY(jbyte, Byte)
WPI_JNI_MAKEJARRAY(jshort, Short)
WPI_JNI_MAKEJARRAY(jlong, Long)
WPI_JNI_MAKEJARRAY(jfloat, Float)
WPI_JNI_MAKEJARRAY(jdouble, Double)

#undef WPI_JNI_MAKEJARRAY

#define JNI_BOOL "Z"
#define JNI_VOID "V"
#define JNI_INT "I"
#define JNI_DOUBLE "D"
#define JNI_DOUBLEARR "[D"
#define JNI_BOOLARR "[Z"

#define JNI_STRINGIFY(x) #x

template <typename A, typename... Ts>
std::string jni_make_method_sig(A retval, Ts &&...args) {
  std::ostringstream oss;
  (oss << "(" << ... << std::forward<Ts>(args)) << ")" << retval;
  return oss.str();
}

/**
 * Finds a class and keeps it as a global reference.
 *
 * Use with caution, as the destructor does NOT call DeleteGlobalRef due to
 * potential shutdown issues with doing so.
 */
class JClass {
public:
  JClass() = default;

  JClass(JNIEnv *env, const char *name) {
    jclass local = env->FindClass(name);
    if (!local) {
      return;
    }
    m_cls = static_cast<jclass>(env->NewGlobalRef(local));
    env->DeleteLocalRef(local);
  }

  void free(JNIEnv *env) {
    if (m_cls) {
      env->DeleteGlobalRef(m_cls);
    }
    m_cls = nullptr;
  }

  explicit operator bool() const { return m_cls; }

  operator jclass() const { return m_cls; }

protected:
  jclass m_cls = nullptr;
};

// Cache MrCalResult class
static JClass detectionClass;
static jmethodID constructor;

extern "C" {
JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved) {
  JNIEnv *env;
  if (vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6) != JNI_OK) {
    return JNI_ERR;
  }

  detectionClass = JClass(env, "org/photonvision/mrcal/MrCalJNI$MrCalResult");

  if (!detectionClass) {
    std::printf("Couldn't find class!");
    return JNI_ERR;
  }

  // Find the constructor. Reference:
  // https://www.microfocus.com/documentation/extend-acucobol/925/BKITITJAVAS027.html
  constructor = env->GetMethodID(
      detectionClass, "<init>",
      jni_make_method_sig(JNI_VOID, JNI_BOOL, JNI_INT, JNI_INT, JNI_DOUBLEARR,
                          JNI_DOUBLEARR, JNI_DOUBLE, JNI_DOUBLEARR, JNI_DOUBLE,
                          JNI_DOUBLE, JNI_INT, JNI_BOOLARR)
          .c_str());

  return JNI_VERSION_1_6;
}

} // extern "C"

/*
 * Class:     org_photonvision_mrcal_MrCalJNI_mrcal_1calibrate
 * Method:    1camera
 * Signature: ([DIIDIID)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_org_photonvision_mrcal_MrCalJNI_mrcal_1calibrate_1camera
  (JNIEnv *env, jclass, jdoubleArray observations_board, jint boardWidth,
   jint boardHeight, jdouble boardSpacing, jint imageWidth, jint imageHeight,
   jdouble focalLenGuessMM)
{
  // Pull out arrays. We rely on data being packed and aligned to make this
  // work! Observations should be [x, y, level]
  std::span<mrcal_point3_t> observations{
      reinterpret_cast<mrcal_point3_t *>(
          env->GetDoubleArrayElements(observations_board, 0)),
      env->GetArrayLength(observations_board) / 3lu};

  size_t points_in_board = boardWidth * boardHeight;
  if (observations.size() % points_in_board != 0) {
    jclass exception_class = env->FindClass("java/lang/Exception");
    if (exception_class && env) {
      (env)->ExceptionClear();
      env->ThrowNew(exception_class,
                    "Observation list length does not match board size!");
      return {};
    } else {
      // ????
      std::cerr << "Observation list length does not match board size!\n";
    }
  }

  size_t boards_observed = observations.size() / points_in_board;

  const auto boardSize = cv::Size{boardWidth, boardHeight};
  const auto imagerSize = cv::Size{imageWidth, imageHeight};

  // down big list of observations/extrinsic guesses (one per board object)
  std::vector<mrcal_pose_t> total_frames_rt_toref;

  for (size_t i = 0; i < boards_observed; i++) {
    auto seed_pose =
        getSeedPose(&(*observations.begin()) + (i * points_in_board), boardSize,
                    imagerSize, boardSpacing, focalLenGuessMM);
    // std::printf("Seed pose %lu: r %f %f %f t %f %f %f\n", i, seed_pose.r.x,
    //             seed_pose.r.y, seed_pose.r.z, seed_pose.t.x, seed_pose.t.y,
    //             seed_pose.t.z);

    // Add to seed poses
    total_frames_rt_toref.push_back(seed_pose);
  }

  // Convert detection level to weights
  for (auto &o : observations) {
    double &level = o.z;
    if (level < 0) {
      o.z = -1;
    } else {
      o.z = std::pow(0.5, level);
    }
  }

  auto statsptr = mrcal_main(observations, total_frames_rt_toref, boardSize,
                             static_cast<double>(boardSpacing), imagerSize,
                             focalLenGuessMM);
  if (!statsptr) {
    return nullptr;
  }
  mrcal_result &stats = *statsptr;

  if (!constructor) {
    return nullptr;
  }

  size_t Nintrinsics = stats.intrinsics.size();
  size_t Nresid = stats.residuals.size();

  jdoubleArray intrinsics =
      MakeJDoubleArray(env, stats.intrinsics.data(), Nintrinsics);
  jdoubleArray residuals =
      MakeJDoubleArray(env, stats.residuals.data(), Nresid);
  jboolean success = stats.success;
  jdouble rms_err = stats.rms_error;
  jdouble warp_x = stats.calobject_warp.x2;
  jdouble warp_y = stats.calobject_warp.y2;
  jint Noutliers = stats.Noutliers_board;

  jdoubleArray optimized_rt_toref = MakeJDoubleArray(
      env, reinterpret_cast<double *>(total_frames_rt_toref.data()),
      total_frames_rt_toref.size() * sizeof(mrcal_pose_t) / sizeof(double));

  std::vector<jboolean> cornersUsedMask(observations.size());
  std::transform(observations.begin(), observations.end(),
                 cornersUsedMask.begin(),
                 [](const auto &pt) { return (jboolean)(pt.z > 0); });
  auto cornersUsedJarr = MakeJBooleanArray(env, cornersUsedMask.data(), cornersUsedMask.size());

  // Actually call the constructor
  auto ret =
      env->NewObject(detectionClass, constructor, success, boardWidth,
                     boardHeight, intrinsics, optimized_rt_toref, rms_err,
                     residuals, warp_x, warp_y, Noutliers, cornersUsedJarr);

  return ret;
}

/*
 * Class:     org_photonvision_mrcal_MrCalJNI_undistort
 * Method:    1mrcal
 * Signature: (JJJJIIIII)Z
 */
JNIEXPORT jboolean JNICALL
Java_org_photonvision_mrcal_MrCalJNI_undistort_1mrcal
  (JNIEnv *, jclass, jlong srcMat, jlong dstMat, jlong camMat, jlong distCoeffs,
   jint lensModelOrdinal, jint order, jint Nx, jint Ny, jint fov_x_deg)
{
  return undistort_mrcal(
      reinterpret_cast<cv::Mat *>(srcMat), reinterpret_cast<cv::Mat *>(dstMat),
      reinterpret_cast<cv::Mat *>(camMat),
      reinterpret_cast<cv::Mat *>(distCoeffs),
      static_cast<CameraLensModel>(lensModelOrdinal),
      static_cast<uint16_t>(order), static_cast<uint16_t>(Nx),
      static_cast<uint16_t>(Ny), static_cast<uint16_t>(fov_x_deg));
}
