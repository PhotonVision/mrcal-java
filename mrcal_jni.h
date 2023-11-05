

/* DO NOT EDIT THIS std::FILE - it is machine generated */
#include <jni.h>

/* Header for class MrCalJNI */

#ifndef MRCAL_JAVA_MRCAL_JNI_H_
#define MRCAL_JAVA_MRCAL_JNI_H_
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     MrCalJNI
 * Method:    mrcal_calibrate_camera
 * Signature: ([D[DDDDDD)LMrCalJNI/CalResult;
 */
JNIEXPORT jobject JNICALL Java_MrCalJNI_mrcal_1calibrate_1camera(
    JNIEnv *, jclass, jdoubleArray, jdoubleArray, jint, jint, jdouble, jint,
    jint);

#ifdef __cplusplus
} // extern "C"
#endif
#endif // MRCAL_JAVA_MRCAL_JNI_H_
