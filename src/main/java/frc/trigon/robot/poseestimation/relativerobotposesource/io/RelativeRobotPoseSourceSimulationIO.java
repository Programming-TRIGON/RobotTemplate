package frc.trigon.robot.poseestimation.relativerobotposesource.io;

import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceIO;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceInputsAutoLogged;

public class RelativeRobotPoseSourceSimulationIO extends RelativeRobotPoseSourceIO {
    @Override
    protected void updateInputs(RelativeRobotPoseSourceInputsAutoLogged inputs) {
        inputs.pose = RobotContainer.POSE_ESTIMATOR.getCurrentOdometryPose();
        inputs.hasNewResult = true;
        inputs.latestResultTimestampSeconds = Timer.getFPGATimestamp();
    }
}
