package frc.trigon.robot.poseestimation.poseestimator;

public class PoseEstimatorConstants {
    public static final double ODOMETRY_FREQUENCY_HERTZ = 250;

    static final StandardDeviations ODOMETRY_STANDARD_DEVIATIONS = new StandardDeviations(0.003, 0.0002);
    static final double
            MAXIMUM_TRANSLATION_VELOCITY_FOR_RELATIVE_ROBOT_POSE_SOURCE_OFFSET_RESETTING_METERS_PER_SECOND = 0,
            MAXIMUM_THETA_VELOCITY_FOR_RELATIVE_ROBOT_POSE_SOURCE_OFFSET_RESETTING_RADIANS_PER_SECOND = 0;
}

