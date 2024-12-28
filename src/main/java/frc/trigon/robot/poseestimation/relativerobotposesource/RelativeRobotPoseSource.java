package frc.trigon.robot.poseestimation.relativerobotposesource;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.poseestimation.relativerobotposesource.io.RelativeRobotPoseSourceSimulationIO;
import frc.trigon.robot.poseestimation.relativerobotposesource.io.RelativeRobotPoseSourceT265IO;
import org.littletonrobotics.junction.AutoLogOutput;
import org.trigon.hardware.RobotHardwareStats;

/**
 * A relative robot pose source is a robot pose source that calculates its position externally.
 * The origin point of the relative pose source doesn't necessarily match the origin point of the robot's estimated pose, so the estimated pose needs to be transformed by that difference.
 */
public class RelativeRobotPoseSource {
    private final RelativeRobotPoseSourceInputsAutoLogged inputs = new RelativeRobotPoseSourceInputsAutoLogged();
    private final Transform2d robotCenterToCamera;
    private final RelativeRobotPoseSourceIO relativeRobotPoseSourceIO;

    private Pose2d cameraPoseAtSyncTime = new Pose2d(0, 0, new Rotation2d(0));
    private Pose2d relativePoseSourcePoseAtSyncTime = new Pose2d(0, 0, new Rotation2d(0));

    /**
     * Constructs a new RelativeRobotPoseSource.
     *
     * @param robotCenterToCamera the transform of the robot's origin point to the camera
     * @param hostname            the name of the camera in the NetworkTables
     */
    public RelativeRobotPoseSource(Transform2d robotCenterToCamera, String hostname) {
        this.robotCenterToCamera = robotCenterToCamera;
        if (RobotHardwareStats.isReplay()) {
            this.relativeRobotPoseSourceIO = new RelativeRobotPoseSourceSimulationIO();
            return;
        } else if (RobotHardwareStats.isSimulation()) {
            this.relativeRobotPoseSourceIO = new RelativeRobotPoseSourceSimulationIO();
            return;
        }
        this.relativeRobotPoseSourceIO = new RelativeRobotPoseSourceT265IO(hostname);
    }

    public void updatePeriodically() {
        relativeRobotPoseSourceIO.updateInputs(inputs);
    }

    /**
     * Resets the offset from the relative robot pose source to the robot pose if the robot isn't moving too fast.
     * The offset is stored as two {@link Pose2d}s, one for the robot's pose and the other for the relative pose source's pose both at the same timestamp.
     * With these two poses, we can calculate the total distance moved from the relative pose source's perspective and transform the robot's pose by the same amount.
     *
     * @param robotPose the current pose of the robot
     */
    public void resetOffset(Pose2d robotPose) {
        if (isUnderMaximumSpeedForOffsetResetting()) {
            this.cameraPoseAtSyncTime = robotPose.transformBy(robotCenterToCamera);
            this.relativePoseSourcePoseAtSyncTime = inputs.pose;
        }
    }

    @AutoLogOutput
    public Pose2d getEstimatedRobotPose() {
        final Transform2d movementFromSyncedPose = new Transform2d(relativePoseSourcePoseAtSyncTime, inputs.pose);
        final Transform2d cameraToRobot = robotCenterToCamera.inverse();
        return cameraPoseAtSyncTime.transformBy(cameraToRobot).transformBy(movementFromSyncedPose);
    }

    public double getLatestResultTimestampSeconds() {
        return inputs.lastResultTimestamp;
    }

    /**
     * Checks if the current velocity of the slow enough to get an accurate result for when we call {@link RelativeRobotPoseSource#resetOffset(Pose2d)}.
     *
     * @return if the robot is moving slow enough to calculate an accurate offset result.
     */
    private boolean isUnderMaximumSpeedForOffsetResetting() {
        final ChassisSpeeds chassisSpeeds = RobotContainer.SWERVE.getSelfRelativeVelocity();
        final double currentTranslationVelocityMetersPerSecond = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        final double currentThetaVelocityRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond;
        return currentTranslationVelocityMetersPerSecond <= RelativeRobotPoseSourceConstants.MAXIMUM_TRANSLATION_VELOCITY_FOR_OFFSET_RESETTING_METERS_PER_SECOND &&
                currentThetaVelocityRadiansPerSecond <= RelativeRobotPoseSourceConstants.MAXIMUM_THETA_VELOCITY_FOR_OFFSET_RESETTING_RADIANS_PER_SECOND;
    }
}
