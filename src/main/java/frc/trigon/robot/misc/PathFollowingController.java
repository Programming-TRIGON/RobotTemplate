package frc.trigon.robot.misc;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.trigon.robot.RobotContainer;

public class PathFollowingController implements com.pathplanner.lib.controllers.PathFollowingController {
    public PathFollowingController() {
    }

    @Override
    public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d pose2d, PathPlannerTrajectoryState pathPlannerTrajectoryState) {
        return new ChassisSpeeds(
                pathPlannerTrajectoryState.fieldSpeeds.vxMetersPerSecond,
                pathPlannerTrajectoryState.fieldSpeeds.vyMetersPerSecond,
                pathPlannerTrajectoryState.fieldSpeeds.omegaRadiansPerSecond
        );
    }

    @Override
    public void reset(Pose2d pose2d, ChassisSpeeds chassisSpeeds) {
        RobotContainer.POSE_ESTIMATOR.resetPose(pose2d);
    }

    @Override
    public boolean isHolonomic() {
        return false;
    }
}