package frc.trigon.robot.poseestimation.poseestimator;

public class PoseEstimatorConstants {
    public static final double ODOMETRY_FREQUENCY_HERTZ = 250;

    static final double POSE_BUFFER_SIZE_SECONDS = 2;

    /**
     * Each number represents how ambiguous a value of the odometry is.
     * The first number represents how ambiguous the x is,
     * the second one is for the y, and the third one is for the theta (rotation).
     * The greater these numbers are, the less we trust the estimation.
     */
    static final double[] AMBIGUITY = {0.003, 0.003, 0.0002};
}

