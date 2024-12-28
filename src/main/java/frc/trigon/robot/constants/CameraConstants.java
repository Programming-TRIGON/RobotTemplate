package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSource;

public class CameraConstants {
    private static final Transform2d ROBOT_CENTER_TO_T265 = new Transform2d(
            new Translation2d(0, 0),
            Rotation2d.fromDegrees(0)
    );

    public static final RelativeRobotPoseSource T265 = new RelativeRobotPoseSource(ROBOT_CENTER_TO_T265, "T265");
    //TODO: implement the rest of CameraConstants
}
