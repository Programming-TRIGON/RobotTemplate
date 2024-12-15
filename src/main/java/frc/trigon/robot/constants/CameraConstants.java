package frc.trigon.robot.constants;

import frc.trigon.robot.poseestimation.poseestimator.PoseEstimatorConstants;
import frc.trigon.robot.poseestimation.t265.T265;

public class CameraConstants {
    public static final T265 T265 = new T265();

    private static final PoseEstimatorConstants.StandardDeviations STANDARD_DEVIATIONS = new PoseEstimatorConstants.StandardDeviations(0.02, 0.02);
    //TODO: implement CameraConstants
}
