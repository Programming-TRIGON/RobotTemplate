package frc.trigon.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.lib.utilities.DynamicCameraTransform;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.robotposeestimator.StandardDeviations;
import org.littletonrobotics.junction.Logger;

/**
 * AprilTagCamera is an object that provides the robot's pose from a camera using one or multiple april tags.
 * An apriltag is a 2d QR-code used to find the robot's position on the field.
 * Since the tag's position on the field is known, we can calculate our position relative to it, therefore estimating our position on the field.
 */
public class AprilTagCamera {
    protected final String name;
    private final AprilTagCameraInputsAutoLogged inputs = new AprilTagCameraInputsAutoLogged();
    private final DynamicCameraTransform dynamicCameraTransform;
    private final StandardDeviations standardDeviations;
    private final AprilTagCameraIO aprilTagCameraIO;
    private Pose2d
            previousEstimatedRobotPose = null,
            estimatedRobotPose = new Pose2d();
    private double previousResultTimestampSeconds = 0;
    private StandardDeviations currentStandardDeviations = null;

    /**
     * Constructs a new AprilTagCamera with a static camera transform.
     *
     * @param aprilTagCameraType  the type of camera
     * @param name                the camera's name
     * @param robotCenterToCamera the static transform from robot center to camera
     * @param standardDeviations  the initial calibrated standard deviations for the camera's estimated pose,
     *                            adjusted based on distance from tags and number of visible tags
     */
    public AprilTagCamera(AprilTagCameraConstants.AprilTagCameraType aprilTagCameraType,
                          String name, Transform3d robotCenterToCamera,
                          StandardDeviations standardDeviations) {
        this(aprilTagCameraType, name, new DynamicCameraTransform(robotCenterToCamera), standardDeviations);
    }

    /**
     * Constructs a new AprilTagCamera with a dynamic camera transform.
     *
     * @param aprilTagCameraType     the type of camera
     * @param name                   the camera's name
     * @param dynamicCameraTransform the dynamic transform from robot center to camera,
     *                               supports time-dependent camera positioning
     * @param standardDeviations     the initial calibrated standard deviations for the camera's estimated pose,
     *                               adjusted based on distance from tags and number of visible tags
     */
    public AprilTagCamera(AprilTagCameraConstants.AprilTagCameraType aprilTagCameraType,
                          String name, DynamicCameraTransform dynamicCameraTransform,
                          StandardDeviations standardDeviations) {
        this.name = name;
        this.standardDeviations = standardDeviations;
        this.dynamicCameraTransform = dynamicCameraTransform;

        aprilTagCameraIO = AprilTagCameraIO.generateIO(aprilTagCameraType, name, dynamicCameraTransform);
    }

    public void update() {
        aprilTagCameraIO.updateInputs(inputs);
        Logger.processInputs("Cameras/" + name, inputs);

        estimatedRobotPose = calculateRobotPose();
        if (hasValidResult())
            currentStandardDeviations = calculateStandardDeviations(estimatedRobotPose);
        logCameraInfo();
    }

    public Pose2d getEstimatedRobotPose() {
        return estimatedRobotPose;
    }

    public String getName() {
        return name;
    }

    public double getLatestResultTimestampSeconds() {
        return inputs.latestResultTimestampSeconds;
    }

    public boolean hasValidResult() {
        return inputs.hasResult &&
                inputs.poseAmbiguity < AprilTagCameraConstants.MAXIMUM_AMBIGUITY &&
                inputs.distancesFromTags[0] < AprilTagCameraConstants.MAXIMUM_DISTANCE_FROM_TAG_FOR_ACCURATE_RESULT_METERS;
    }

    /**
     * Solve PNP is inaccurate the further the camera is from the tag.
     * Because of this, there are some things we might want to do only if we are close enough to get an accurate enough result.
     * This method checks if the current distance from the tag is less than the maximum distance for an accurate result, which is defined as the variable {@link AprilTagCameraConstants#MAXIMUM_DISTANCE_FROM_TAG_FOR_ACCURATE_SOLVE_PNP_RESULT_METERS}.
     *
     * @return if the camera is close enough to the tag to get an accurate result from solve PNP
     */
    public boolean isWithinBestTagRangeForAccurateSolvePNPResult() {
        return hasValidResult() && inputs.distancesFromTags[0] < AprilTagCameraConstants.MAXIMUM_DISTANCE_FROM_TAG_FOR_ACCURATE_SOLVE_PNP_RESULT_METERS;
    }

    public StandardDeviations getCurrentStandardDeviations() {
        return currentStandardDeviations;
    }

    private StandardDeviations calculateStandardDeviations(Pose2d estimatedRobotPose) {
        final double averageDistanceFromTags = calculateAverageDistanceFromTags();
        double translationStandardDeviation = calculateStandardDeviation(standardDeviations.translationStandardDeviation(), averageDistanceFromTags, inputs.visibleTagIDs.length);
        double thetaStandardDeviation = calculateStandardDeviation(standardDeviations.thetaStandardDeviation(), averageDistanceFromTags, inputs.visibleTagIDs.length);
        final double tooFarTranslationStandardDeviation = isUpdateTooFarFetched(estimatedRobotPose) ? Double.POSITIVE_INFINITY : 0;

        translationStandardDeviation += tooFarTranslationStandardDeviation;
        thetaStandardDeviation += tooFarTranslationStandardDeviation;

        Logger.recordOutput("StandardDeviations/" + name + "/translations", translationStandardDeviation);
        Logger.recordOutput("StandardDeviations/" + name + "/theta", thetaStandardDeviation);

        return new StandardDeviations(translationStandardDeviation, thetaStandardDeviation);
    }

    private boolean isUpdateTooFarFetched(Pose2d estimatedRobotPose) {
        final Pose2d previousEstimatedRobotPose = this.previousEstimatedRobotPose;
        final double previousResultTimestampSeconds = this.previousResultTimestampSeconds;
        if (hasValidResult())
            updatePreviousResultInfo();

        if (inputs.latestResultTimestampSeconds - previousResultTimestampSeconds > 3 || previousEstimatedRobotPose == null)
            return false;

        return previousEstimatedRobotPose.getTranslation().getDistance(estimatedRobotPose.getTranslation()) > 3 && estimatedRobotPose.getTranslation().getDistance(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation()) > 2;
    }

    private void updatePreviousResultInfo() {
        previousResultTimestampSeconds = inputs.latestResultTimestampSeconds;
        previousEstimatedRobotPose = estimatedRobotPose;
    }

    private Pose2d calculateRobotPose() {
        if (!hasValidResult())
            return null;

        return chooseBestRobotPose();
    }

    private Pose2d chooseBestRobotPose() {
        if (!inputs.hasConstrainedResult || isWithinBestTagRangeForAccurateSolvePNPResult())
            return chooseBestNormalSolvePNPPose();

        return cameraPoseToRobotPose(inputs.constrainedSolvePNPPose, getLatestResultTimestampSeconds());
    }

    private Pose2d chooseBestNormalSolvePNPPose() {
        final Pose2d bestPose = cameraPoseToRobotPose(inputs.bestCameraSolvePNPPose, getLatestResultTimestampSeconds());

        if (inputs.bestCameraSolvePNPPose.equals(inputs.alternateCameraSolvePNPPose))
            return bestPose;
        if (inputs.alternateCameraSolvePNPPose.getTranslation().toTranslation2d().getDistance(FieldConstants.TAG_ID_TO_POSE.get(inputs.visibleTagIDs[0]).getTranslation().toTranslation2d()) < 0.1)
            return bestPose;

        final Pose2d alternatePose = cameraPoseToRobotPose(inputs.alternateCameraSolvePNPPose, getLatestResultTimestampSeconds());
        final Rotation2d robotAngleAtResultTime = RobotContainer.ROBOT_POSE_ESTIMATOR.samplePoseAtTimestamp(getLatestResultTimestampSeconds()).getRotation();

        final double bestAngleDifference = Math.abs(bestPose.getRotation().minus(robotAngleAtResultTime).getRadians());
        final double alternateAngleDifference = Math.abs(alternatePose.getRotation().minus(robotAngleAtResultTime).getRadians());

        return bestAngleDifference > alternateAngleDifference ? alternatePose : bestPose;
    }

    private Pose2d cameraPoseToRobotPose(Pose3d cameraPose, double resultTimestampSeconds) {
        return dynamicCameraTransform.calculate2dRobotPose(cameraPose.toPose2d(), resultTimestampSeconds);
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
        return exponent * (distance * distance) / (numberOfVisibleTags * numberOfVisibleTags);
    }

    private void logCameraInfo() {
        if (!FieldConstants.TAG_ID_TO_POSE.isEmpty())
            logUsedTags();

        if (hasValidResult()) {
            Logger.recordOutput("Poses/Robot/Cameras/" + name + "Pose", new Pose2d[]{estimatedRobotPose});
            return;
        }

        Logger.recordOutput("Poses/Robot/Cameras/" + name + "Pose", AprilTagCameraConstants.EMPTY_POSE_ARRAY);
    }

    private void logUsedTags() {
        if (!hasValidResult()) {
            Logger.recordOutput("UsedTags/" + this.getName(), new Pose3d[0]);
            return;
        }

        final Pose3d[] usedTagPoses = isWithinBestTagRangeForAccurateSolvePNPResult() ? new Pose3d[inputs.visibleTagIDs.length] : new Pose3d[1];
        for (int i = 0; i < usedTagPoses.length; i++)
            usedTagPoses[i] = FieldConstants.TAG_ID_TO_POSE.get(inputs.visibleTagIDs[i]);
        Logger.recordOutput("UsedTags/" + this.getName(), usedTagPoses);
    }

    private double calculateAverageDistanceFromTags() {
        double totalDistance = 0;
        for (int visibleTagID : inputs.visibleTagIDs) {
            totalDistance += FieldConstants.TAG_ID_TO_POSE.get(visibleTagID).getTranslation().getDistance(inputs.bestCameraSolvePNPPose.getTranslation());
        }
        return totalDistance / inputs.visibleTagIDs.length;
    }
}