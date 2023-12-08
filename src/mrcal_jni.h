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
JNIEXPORT jobject JNICALL
Java_org_photonvision_mrcal_MrCalJNI_mrcal_1calibrate_1camera(
    JNIEnv *, jclass, jdoubleArray, jint, jint, jdouble, jint, jint, jdouble);

#ifdef __cplusplus
} // extern "C"
#endif
#endif // MRCAL_JAVA_MRCAL_JNI_H_
