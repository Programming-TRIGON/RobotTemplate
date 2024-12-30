package frc.trigon.robot.poseestimation.relativerobotposesource.io;

import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceIO;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceInputsAutoLogged;

public class RelativeRobotPoseSourceSimulationIO extends RelativeRobotPoseSourceIO {
    @Override
    protected void updateInputs(RelativeRobotPoseSourceInputsAutoLogged inputs) {
        inputs.framesPerSecond = 60;
        inputs.batteryPercentage = 100;
        inputs.pose = RobotContainer.POSE_ESTIMATOR.getCurrentOdometryPose();
        inputs.lastResultTimestamp = Timer.getTimestamp();
    }
}
