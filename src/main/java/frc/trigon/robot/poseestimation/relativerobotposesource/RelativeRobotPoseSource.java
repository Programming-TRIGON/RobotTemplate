package frc.trigon.robot.poseestimation.relativerobotposesource;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * A relative robot pose source is a robot pose source that calculates its position externally.
 * The origin point of the relative pose source doesn't necessarily match the origin point of the robot's estimated pose, so the estimated pose needs to be transformed by that difference.
 */
public class RelativeRobotPoseSource {
    private final RelativeRobotPoseSourceInputsAutoLogged inputs = new RelativeRobotPoseSourceInputsAutoLogged();
    private final Transform2d cameraToRobotCenter;
    private final RelativeRobotPoseSourceIO relativeRobotPoseSourceIO;

    private double lastResultTimestampSeconds = 0;
    private Pose2d robotPoseAtSyncTime = new Pose2d();
    private Pose2d relativePoseSourceEstimatedPoseAtSyncTime = new Pose2d();

    /**
     * Constructs a new RelativeRobotPoseSource.
     *
     * @param cameraToRobotCenter the transform of the camera to the robot's origin point
     * @param hostname            the name of the camera in the NetworkTables
     */
    public RelativeRobotPoseSource(Transform2d cameraToRobotCenter, String hostname) {
        this.cameraToRobotCenter = cameraToRobotCenter;
        this.relativeRobotPoseSourceIO = RelativeRobotPoseSourceIO.generateIO(hostname);
    }

    public void update() {
        relativeRobotPoseSourceIO.updateInputs(inputs);
    }

    /**
     * Resets the offset from the relative robot pose source to the robot pose.
     * The offset is stored as two {@link Pose2d}s, one for the robot's pose and the other for the relative pose source's pose both at the same timestamp.
     * With these two poses, we can calculate the total distance moved from the relative pose source's perspective and transform the robot's pose by the same amount.
     *
     * @param robotPose the current pose of the robot
     */
    public void resetOffset(Pose2d robotPose) {
        this.robotPoseAtSyncTime = robotPose;
        this.relativePoseSourceEstimatedPoseAtSyncTime = getRobotPoseFromCameraPose(inputs.pose);
    }

    @AutoLogOutput(key = "RelativeRobotPoseSource/EstimatedRobotPose")
    public Pose2d getEstimatedRobotPose() {
        final Transform2d movementFromSyncedPose = new Transform2d(relativePoseSourceEstimatedPoseAtSyncTime, getRobotPoseFromCameraPose(inputs.pose));
        return robotPoseAtSyncTime.transformBy(movementFromSyncedPose);
    }

    public double getLatestResultTimestampSeconds() {
        return inputs.resultTimestampSeconds;
    }

    public boolean hasNewResult() {
        return inputs.hasResult && isNewTimestamp();
    }

    private boolean isNewTimestamp() {
        if (lastResultTimestampSeconds == getLatestResultTimestampSeconds())
            return false;

        lastResultTimestampSeconds = getLatestResultTimestampSeconds();
        return true;
    }

    private Pose2d getRobotPoseFromCameraPose(Pose2d cameraPose) {
        return cameraPose.transformBy(cameraToRobotCenter);
    }
}
