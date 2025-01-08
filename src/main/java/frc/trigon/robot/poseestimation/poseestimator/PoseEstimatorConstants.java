package frc.trigon.robot.poseestimation.poseestimator;

public class PoseEstimatorConstants {
    public static final double ODOMETRY_FREQUENCY_HERTZ = 250;

    static final double POSE_BUFFER_SIZE_SECONDS = 2;
    static final StandardDeviations ODOMETRY_STANDARD_DEVIATIONS = new StandardDeviations(Math.pow(0.003, 2), Math.pow(0.0002, 2));
    static final double
            MAXIMUM_TRANSLATION_VELOCITY_FOR_RELATIVE_ROBOT_POSE_SOURCE_OFFSET_RESETTING_METERS_PER_SECOND = 0,
            MAXIMUM_THETA_VELOCITY_FOR_RELATIVE_ROBOT_POSE_SOURCE_OFFSET_RESETTING_RADIANS_PER_SECOND = 0;
}

