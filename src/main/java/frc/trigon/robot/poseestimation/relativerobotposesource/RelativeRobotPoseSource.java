package frc.trigon.robot.poseestimation.relativerobotposesource;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.trigon.robot.RobotContainer;
import org.littletonrobotics.junction.AutoLogOutput;

public class RelativeRobotPoseSource {
    private final RelativeRobotPoseSourceInputsAutoLogged inputs = new RelativeRobotPoseSourceInputsAutoLogged();
    private final RelativeRobotPoseSourceIO relativeRobotPoseSourceIO;

    private Pose2d lastResettedRobotPose = new Pose2d(0, 0, new Rotation2d(0));
    private Pose2d lastResettedRelativePoseSourcePose = new Pose2d(0, 0, new Rotation2d(0));

    public RelativeRobotPoseSource(RelativeRobotPoseSourceIO relativeRobotPoseSourceIO) {
        this.relativeRobotPoseSourceIO = relativeRobotPoseSourceIO;
    }

    public void updatePeriodically() {
        relativeRobotPoseSourceIO.updateInputs(inputs);
    }

    /**
     * Resets the offset from the relative robot pose source to the robot pose.
     *
     * @param robotPose the current pose of the robot
     */
    public void resetOffset(Pose2d robotPose) {
        if (isUnderMaximumSpeedForOffsetResetting()) {
            this.lastResettedRobotPose = robotPose;
            this.lastResettedRelativePoseSourcePose = inputs.pose;
        }
    }

    @AutoLogOutput
    public Pose2d getEstimatedRobotPose() {
        final Transform2d movementFromLastResettedPose = new Transform2d(lastResettedRelativePoseSourcePose, inputs.pose);
        return lastResettedRobotPose.transformBy(movementFromLastResettedPose);
    }

    public double getLatestResultTimestampSeconds() {
        return inputs.latestResultTimestampSeconds;
    }

    private boolean isUnderMaximumSpeedForOffsetResetting() {
        final ChassisSpeeds chassisSpeeds = RobotContainer.SWERVE.getSelfRelativeVelocity();
        final double currentTranslationVelocityMetersPerSecond = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        final double currentThetaVelocityRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond;
        return currentTranslationVelocityMetersPerSecond <= RelativeRobotPoseSourceConstants.MAXIMUM_TRANSLATION_VELOCITY_FOR_OFFSET_RESETTING_METERS_PER_SECOND &&
                currentThetaVelocityRadiansPerSecond <= RelativeRobotPoseSourceConstants.MAXIMUM_THETA_VELOCITY_FOR_OFFSET_RESETTING_RADIANS_PER_SECOND;
    }
}
