package frc.trigon.robot.poseestimation.poseestimator;

public class PoseEstimatorConstants {
    public static final double ODOMETRY_FREQUENCY_HERTZ = 250;

    static final double POSE_BUFFER_SIZE_SECONDS = 2;

    static final StandardDeviations ODOMETRY_STANDARD_DEVIATIONS = new StandardDeviations(0.003, 0.0002);


    /**
     * A record that stores how ambiguous the odometry is.
     * The greater the number, the less trustworthy the estimated pose is.
     *
     * @param translation the ambiguity of the translation
     * @param theta       the ambiguity of the rotation
     */
    public record StandardDeviations(double translation, double theta) {
    }
}

