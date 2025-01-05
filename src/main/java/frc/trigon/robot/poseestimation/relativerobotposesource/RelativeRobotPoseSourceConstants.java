package frc.trigon.robot.poseestimation.relativerobotposesource;

import frc.trigon.robot.poseestimation.poseestimator.StandardDeviations;

public class RelativeRobotPoseSourceConstants {
    public static final StandardDeviations STANDARD_DEVIATIONS = new StandardDeviations(0.0000001, 0.0000001);

    public static final double
            MAXIMUM_TRANSLATION_VELOCITY_FOR_OFFSET_RESETTING_METERS_PER_SECOND = 0,
            MAXIMUM_THETA_VELOCITY_FOR_OFFSET_RESETTING_RADIANS_PER_SECOND = 0;
}
