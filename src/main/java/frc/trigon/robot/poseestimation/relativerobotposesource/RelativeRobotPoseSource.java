package frc.trigon.robot.poseestimation.relativerobotposesource;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class RelativeRobotPoseSource {
    private final RelativeRobotPoseSourceInputsAutoLogged inputs = new RelativeRobotPoseSourceInputsAutoLogged();
    private final RelativeRobotPoseSourceIO relativeRobotPoseSourceIO;

    private Transform2d robotToT265 = new Transform2d(0, 0, new Rotation2d(0));

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
        robotToT265 = inputs.pose.minus(robotPose);
    }

    public Pose2d getEstimatedRobotPose() {
        final Transform2d robotPose = new Transform2d(transform2dToPose2d(robotToT265), inputs.pose);
        return transform2dToPose2d(robotPose);
    }

    public double getLatestResultTimestampSeconds() {
        return inputs.latestResultTimestampSeconds;
    }

    private Pose2d transform2dToPose2d(Transform2d transform) {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }
}
