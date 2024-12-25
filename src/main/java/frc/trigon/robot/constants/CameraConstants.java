package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSource;

public class CameraConstants {
    private static final Transform3d ROBOT_CENTER_TO_T265 = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    );

    public static final RelativeRobotPoseSource T265 = new RelativeRobotPoseSource(ROBOT_CENTER_TO_T265, "T265");
    //TODO: implement the rest of CameraConstants
}
