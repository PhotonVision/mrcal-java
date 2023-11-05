

class MrCalJNI {
    public static class CalResult {
        public CalResult(){}
    }

    public static native CalResult mrcal_calibrate_camera(
        double[] observations_board,
        double[] frames_rt_toref,
        int boardWidth, int boardHeight, double boardSpacing,
        int imageWidth, int imageHeight
    );
}
