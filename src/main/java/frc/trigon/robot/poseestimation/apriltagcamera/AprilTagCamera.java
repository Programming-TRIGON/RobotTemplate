package frc.trigon.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.*;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.poseestimator.StandardDeviations;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.RobotHardwareStats;

/**
 * An april tag camera is a class that provides the robot's pose from a camera using one or multiple apriltags.
 * An april tag is like a 2D barcode used to find the robot's position on the field.
 * Since the tag's position on the field is known, we can calculate our position relative to it, therefore estimating our position on the field.
 */
public class AprilTagCamera {
    protected final String name;
    private final AprilTagCameraInputsAutoLogged inputs = new AprilTagCameraInputsAutoLogged();
    private final Transform3d robotCenterToCamera;
    private final StandardDeviations standardDeviations;
    private final AprilTagCameraIO aprilTagCameraIO;
    private Pose2d robotPose = null;

    /**
     * Constructs a new AprilTagCamera.
     *
     * @param aprilTagCameraType  the type of camera
     * @param name                the camera's name
     * @param robotCenterToCamera the transform of the robot's origin point to the camera
     * @param standardDeviations  the calibrated standard deviations for the camera's estimated pose
     */
    public AprilTagCamera(AprilTagCameraConstants.AprilTagCameraType aprilTagCameraType,
                          String name, Transform3d robotCenterToCamera,
                          StandardDeviations standardDeviations) {
        this.name = name;
        this.robotCenterToCamera = robotCenterToCamera;
        this.standardDeviations = standardDeviations;

        aprilTagCameraIO = AprilTagCameraIO.generateIO(aprilTagCameraType, name);
        if (RobotHardwareStats.isSimulation())
            aprilTagCameraIO.addSimulatedCameraToVisionSimulation(robotCenterToCamera);
    }

    public void update() {
        aprilTagCameraIO.updateInputs(inputs);

        robotPose = getRobotPose();
        logCameraInfo();
    }

    public Pose2d getRobotPose() {
        if (inputs.poseAmbiguity < AprilTagCameraConstants.MAXIMUM_AMBIGUITY)
            return cameraPoseToRobotPose(inputs.cameraSolvePNPPose.toPose2d());
        return robotPose;
    }

    public String getName() {
        return name;
    }

    public double getLatestResultTimestampSeconds() {
        return inputs.latestResultTimestampSeconds;
    }

    public boolean hasResult() {
        return inputs.hasResult;
    }

    /**
     * Calculates the range of how inaccurate the estimated pose could be using the distance from the target, the number of targets, and a calibrated gain.
     *
     * @return the standard deviations of the current estimated pose
     */
    public StandardDeviations calculateStandardDeviations() {
        final double translationStandardDeviation = calculateStandardDeviation(standardDeviations.translationStandardDeviation, inputs.distanceFromBestTag, inputs.visibleTagIDs.length);
        final double thetaStandardDeviation = calculateStandardDeviation(standardDeviations.thetaStandardDeviation, inputs.distanceFromBestTag, inputs.visibleTagIDs.length);

        return new StandardDeviations(translationStandardDeviation, thetaStandardDeviation);
    }

    /**
     * Solve PNP is inaccurate the further the camera is from the tag.
     * Because of this, there are some things we might want to do only if we are close enough to get an accurate enough result.
     * This method checks if the current distance from the tag is less than the maximum distance for an accurate result, which is defined as the variable {@link AprilTagCameraConstants#MAXIMUM_DISTANCE_FROM_TAG_FOR_ACCURATE_SOLVE_PNP_RESULT_METERS}.
     *
     * @return if the camera is close enough to the tag to get an accurate result from solve PNP
     */
    public boolean isWithinBestTagRangeForAccurateSolvePNPResult() {
        return inputs.distanceFromBestTag < AprilTagCameraConstants.MAXIMUM_DISTANCE_FROM_TAG_FOR_ACCURATE_SOLVE_PNP_RESULT_METERS;
    }

    /**
     * Calculates an aspect of the standard deviations of the estimated pose using a formula.
     * As we get further from the tag(s), this will return a less trusting (higher deviation) result.
     *
     * @param exponent            a calibrated gain
     * @param distance            the distance from the tag(s)
     * @param numberOfVisibleTags the number of visible tags
     * @return the standard deviation
     */
    private double calculateStandardDeviation(double exponent, double distance, int numberOfVisibleTags) {
        return exponent * (distance * distance) / numberOfVisibleTags;
    }

    private Pose2d cameraPoseToRobotPose(Pose2d cameraPose) {
        final Translation2d robotCenterToCameraTranslation = robotCenterToCamera.getTranslation().toTranslation2d();
        final Rotation2d robotCenterToCameraRotation = robotCenterToCamera.getRotation().toRotation2d();

        return cameraPose.transformBy(new Transform2d(robotCenterToCameraTranslation, robotCenterToCameraRotation).inverse());
    }

    private void logCameraInfo() {
        Logger.processInputs("Cameras/" + name, inputs);
        if (!FieldConstants.TAG_ID_TO_POSE.isEmpty())
            logUsedTags();

        if (!inputs.hasResult || inputs.distanceFromBestTag == Double.POSITIVE_INFINITY || robotPose == null) {
            Logger.recordOutput("Poses/Robot/" + name + "/Pose", robotPose);
            return;
        }
        Logger.recordOutput("Poses/Robot/" + name + "/Pose", AprilTagCameraConstants.EMPTY_POSE_LIST);
    }

    private void logUsedTags() {
        if (!inputs.hasResult) {
            Logger.recordOutput("UsedTags/" + this.getName(), new Pose3d[0]);
            return;
        }

        final Pose3d[] usedTagPoses = isWithinBestTagRangeForAccurateSolvePNPResult() ? new Pose3d[inputs.visibleTagIDs.length] : new Pose3d[1];
        for (int i = 0; i < usedTagPoses.length; i++)
            usedTagPoses[i] = FieldConstants.TAG_ID_TO_POSE.get(inputs.visibleTagIDs[i]);
        Logger.recordOutput("UsedTags/" + this.getName(), usedTagPoses);
    }
}