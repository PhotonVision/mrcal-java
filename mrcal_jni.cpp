

#include "mrcal_jni.h"

#include <cstdio>
#include <span>
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
JClass detectionClass;

extern "C" {
JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved) {
  JNIEnv *env;
  if (vm->GetEnv((void **)(&env), JNI_VERSION_1_6) != JNI_OK) {
    return JNI_ERR;
  }

  detectionClass = JClass(env, "org/photonvision/mrcal/MrCalResult");

  if (!detectionClass) {
    std::printf("Couldn't find class!");
    return JNI_ERR;
  }

  return JNI_VERSION_1_6;
}
} // extern "C"

/*
 * Class:     MrCalJNI_mrcal_1calibrate
 * Method:    1camera
 * Signature: ([D[DIIDII)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_MrCalJNI_mrcal_1calibrate_1camera
  (JNIEnv *env, jclass, jdoubleArray observations_board,
   jint boardWidth, jint boardHeight,
   jdouble boardSpacing, jint imageWidth, jint imageHeight)
{
  // Pull out arrays. We rely on data being packed and aligned to make this
  // work! Observations should be [x, y, level]
  std::span<mrcal_point3_t> observations{
      reinterpret_cast<mrcal_point3_t *>(
          env->GetDoubleArrayElements(observations_board, 0)),
      env->GetArrayLength(observations_board) / sizeof(mrcal_point3_t)};

  size_t points_in_board = boardWidth * boardHeight;
  assert(observations.size() % points_in_board == 0);
  size_t boards_observed = observations.size() / points_in_board;

  const auto boardSize = cv::Size{boardWidth, boardHeight};
  const auto imagerSize = cv::Size{imageWidth, imageHeight};

  // down big list of observations/extrinsic guesses (one per board object)
  std::vector<mrcal_pose_t> total_frames_rt_toref;

  for (size_t i = 0; i < boards_observed; i++) {
      auto seed_pose = getSeedPose(&(*observations.begin()) + (i * points_in_board), boardSize, imagerSize);
      std::printf("Seed pose %lu: r %f %f %f t %f %f %f\n", i, seed_pose.r.x,
                  seed_pose.r.y, seed_pose.r.z, seed_pose.t.x, seed_pose.t.y, seed_pose.t.z);

      // Add to seed poses
      total_frames_rt_toref.push_back(seed_pose);
  }

  // Convert detection level to weights (we do a little memory manipulation)
  for (auto& o : observations) {
      double& level = o.z;
      if (level < 0) {
        o.z = -1;
      } else {
        o.z = std::pow(0.5, level);
      }
  }

  auto calibration_result =
      mrcal_main(observations, total_frames_rt_toref, boardSize,
                 static_cast<double>(boardSpacing), imagerSize);

  // Find the constructor
  static jmethodID constructor =
      env->GetMethodID(detectionClass, "<init>", "(IIF[DDD[D[D[DD[D[DD)V");
  if (!constructor) {
    return nullptr;
  }

  // Actually call the constructor (TODO)
  auto ret = env->NewObject(detectionClass, constructor, (jint)1234);

  return ret;
}
