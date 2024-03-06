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

package org.photonvision.mrcal;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.opencv.core.MatOfPoint2f;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class MrCalJNI {
    public static class MrCalResult {
        public boolean success;
        public double[] intrinsics;
        public double rms_error;
        public double[] residuals;
        public double warp_x;
        public double warp_y;
        public int Noutliers;
        public List<Pose3d> optimizedPoses;
        public List<boolean[]> cornersUsed;

        public MrCalResult(boolean success) {
            this.success = success;
        }

        public MrCalResult(
                boolean success, int width, int height, double[] intrinsics, double[] optimized_rt_rtoref,
                double rms_error, double[] residuals, double warp_x,
                double warp_y, int Noutliers, boolean[] cornerUseMask) {
            this.success = success;
            this.intrinsics = intrinsics;
            this.rms_error = rms_error;
            this.residuals = residuals;
            this.warp_x = warp_x;
            this.warp_y = warp_y;
            this.Noutliers = Noutliers;

            optimizedPoses = new ArrayList<>();
            for (int i = 0; i < optimized_rt_rtoref.length; i += 6) {
                var rot = new Rotation3d(VecBuilder.fill(
                        optimized_rt_rtoref[i + 0],
                        optimized_rt_rtoref[i + 1],
                        optimized_rt_rtoref[i + 2]));
                var t = new Translation3d(
                        optimized_rt_rtoref[i + 3],
                        optimized_rt_rtoref[i + 4],
                        optimized_rt_rtoref[i + 5]);

                optimizedPoses.add(new Pose3d(t, rot));
            }

            var cornersPerBoard = width * height;
            cornersUsed = new ArrayList<>();
            for (int cornerIdx = 0; cornerIdx < cornerUseMask.length; cornerIdx += cornersPerBoard) {
                cornersUsed.add(Arrays.copyOfRange(cornerUseMask, cornerIdx, cornerIdx + cornersPerBoard));
            }
        }

        @Override
        public String toString() {
            return "MrCalResult [success=" + success + ", intrinsics=" + Arrays.toString(intrinsics) + ", rms_error="
                    + rms_error + ", warp_x=" + warp_x + ", warp_y="
                    + warp_y + ", Noutliers=" + Noutliers + "]";
        }
    }

    public static native MrCalResult mrcal_calibrate_camera(
            double[] observations_board,
            int boardWidth, int boardHeight, double boardSpacing,
            int imageWidth, int imageHeight, double focalLen);

    public static native boolean undistort_mrcal(long srcMat, long dstMat, long cameraMat, long distCoeffsMat,
            int lensModelOrdinal, int order, int Nx, int Ny, int fov_x_deg);

    public static MrCalResult calibrateCamera(
            List<MatOfPoint2f> board_corners,
            int boardWidth, int boardHeight, double boardSpacing,
            int imageWidth, int imageHeight, double focalLen) {
        double[] observations = new double[boardWidth * boardHeight * 3 * board_corners.size()];

        int i = 0;
        for (var board : board_corners) {
            var corners = board.toArray();
            // Assume that we're correct in terms of row/column major-ness (lol)
            for (var c : corners) {
                float level = 1.0f; // if we have mrgingham, use level from that. Otherwise, hard-coded to 1

                observations[i * 3 + 0] = c.x;
                observations[i * 3 + 1] = c.y;
                observations[i * 3 + 2] = level;

                i += 1;
            }
        }

        if (i * 3 != observations.length) {
            return new MrCalResult(false);
        }

        return mrcal_calibrate_camera(observations, boardWidth, boardHeight, boardSpacing, imageWidth, imageHeight,
                focalLen);
    }
}
