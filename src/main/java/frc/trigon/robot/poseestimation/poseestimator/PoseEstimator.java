package frc.trigon.robot.poseestimation.poseestimator;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSource;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceConstants;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.utilities.QuickSortHandler;

import java.util.Arrays;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;

/**
 * A class that estimates the robot's pose using team 6328's custom pose estimator.
 */
public class PoseEstimator implements AutoCloseable {
    private final Field2d field = new Field2d();
    private final AprilTagCamera[] aprilTagCameras;
    private final TimeInterpolatableBuffer<Pose2d> previousOdometryPoses = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
    private final RelativeRobotPoseSource relativeRobotPoseSource;
    private final boolean shouldUseRelativeRobotPoseSource;

    private Pose2d
            odometryPose = new Pose2d(),
            estimatedPose = new Pose2d();
    private SwerveModulePosition[] lastSwerveModulePositions = new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };
    private Rotation2d lastGyroHeading = new Rotation2d();

    /**
     * Constructs a new PoseEstimator and sets the relativeRobotPoseSource.
     * This constructor enables usage of a relative robot pose source and disables the use of april tags for pose estimation, and instead uses them to reset the relative robot pose source's offset.
     *
     * @param relativeRobotPoseSource the relative robot pose source that should be used to update the pose estimator
     * @param aprilTagCameras         the apriltag cameras that should be used to update the relative robot pose source
     */
    public PoseEstimator(RelativeRobotPoseSource relativeRobotPoseSource, AprilTagCamera... aprilTagCameras) {
        this.aprilTagCameras = aprilTagCameras;
        this.relativeRobotPoseSource = relativeRobotPoseSource;
        this.shouldUseRelativeRobotPoseSource = true;

        initialize();
    }

    /**
     * Constructs a new PoseEstimator.
     * This constructor disables the use of a relative robot pose source and instead uses april tags cameras for pose estimation.
     *
     * @param aprilTagCameras the cameras that should be used to update the pose estimator
     */
    public PoseEstimator(AprilTagCamera... aprilTagCameras) {
        this.aprilTagCameras = aprilTagCameras;
        this.relativeRobotPoseSource = null;
        this.shouldUseRelativeRobotPoseSource = false;

        initialize();
    }

    @Override
    public void close() {
        field.close();
    }

    public void periodic() {
        if (shouldUseRelativeRobotPoseSource)
            updateFromRelativeRobotPoseSource();
        else
            updateFromAprilTagCameras();

        field.setRobotPose(getCurrentEstimatedPose());
        if (RobotHardwareStats.isSimulation())
            AprilTagCameraConstants.VISION_SIMULATION.update(RobotContainer.POSE_ESTIMATOR.getCurrentOdometryPose());
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param newPose the pose to reset to, relative to the blue alliance's driver station right corner
     */
    public void resetPose(Pose2d newPose) {
        RobotContainer.SWERVE.setHeading(newPose.getRotation());
        odometryPose = newPose;
        estimatedPose = newPose;
        lastGyroHeading = newPose.getRotation();
        previousOdometryPoses.clear();
        if (shouldUseRelativeRobotPoseSource)
            relativeRobotPoseSource.resetOffset(newPose);
    }

    /**
     * @return the estimated pose of the robot, relative to the blue alliance's driver station right corner
     */
    @AutoLogOutput
    public Pose2d getCurrentEstimatedPose() {
        return estimatedPose;
    }

    /**
     * @return the odometry's estimated pose of the robot, relative to the blue alliance's driver station right corner
     */
    @AutoLogOutput
    public Pose2d getCurrentOdometryPose() {
        return odometryPose;
    }

    /**
     * Updates the pose estimator with the given SWERVE wheel positions and gyro rotations.
     * This function accepts an array of SWERVE wheel positions and an array of gyro rotations because the odometry can be updated at a faster rate than the main loop (which is 50 hertz).
     * This means you could have a couple of odometry updates per main loop, and you would want to update the pose estimator with all of them.
     *
     * @param swerveWheelPositions the SWERVE wheel positions accumulated since the last update
     * @param gyroRotations        the gyro rotations accumulated since the last update
     */
    public void updatePoseEstimatorStates(SwerveModulePosition[][] swerveWheelPositions, Rotation2d[] gyroRotations, double[] timestamps) {
        for (int i = 0; i < swerveWheelPositions.length; i++)
            addOdometryObservation(swerveWheelPositions[i], gyroRotations[i], timestamps[i]);
    }

    /**
     * Gets the estimated pose of the robot at the target timestamp.
     *
     * @param timestamp the target timestamp
     * @return the robot's estimated pose at the timestamp
     */
    public Pose2d getEstimatedPoseAtTimestamp(double timestamp) {
        final Pose2d odometryPoseAtTimestamp = getOdometryPoseAtTimestamp(timestamp);
        return calculateEstimatedPoseAtTimestamp(odometryPoseAtTimestamp);
    }

    private void initialize() {
        putAprilTagsOnFieldWidget();
        SmartDashboard.putData("Field", field);
        logTargetPath();
    }

    private void putAprilTagsOnFieldWidget() {
        for (Map.Entry<Integer, Pose3d> entry : FieldConstants.TAG_ID_TO_POSE.entrySet()) {
            final Pose2d tagPose = entry.getValue().toPose2d();
            field.getObject("Tag " + entry.getKey()).setPose(tagPose);
        }
    }

    /**
     * Logs and updates the field widget with the target PathPlanner path as an array of Pose2ds.
     */
    private void logTargetPath() {
        PathPlannerLogging.setLogActivePathCallback((pathPoses) -> {
            field.getObject("path").setPoses(pathPoses);
            Logger.recordOutput("PathPlanner/Path", pathPoses.toArray(new Pose2d[0]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> Logger.recordOutput("PathPlanner/TargetPose", pose));
    }

    /**
     * Sets the estimated robot pose from the odometry at the given timestamp.
     *
     * @param swerveModulePositions the positions of each swerve module
     * @param gyroHeading           the heading of the gyro
     * @param timestamp             the timestamp of the odometry observation
     */
    private void addOdometryObservation(SwerveModulePosition[] swerveModulePositions, Rotation2d gyroHeading, double timestamp) {
        final Twist2d newOdometryPoseDifference = calculateNewOdometryPoseDifference(swerveModulePositions, gyroHeading);
        updateRobotPosesFromNewOdometryPoseDifference(newOdometryPoseDifference, timestamp);

        updateOdometryPositions(swerveModulePositions, gyroHeading);
    }

    private void updateFromRelativeRobotPoseSource() {
        for (AprilTagCamera aprilTagCamera : aprilTagCameras) {
            aprilTagCamera.update();

            if (aprilTagCamera.isWithinBestTagRangeForAccurateSolvePNPResult() && isUnderMaximumSpeedForOffsetResetting())
                relativeRobotPoseSource.resetOffset(aprilTagCamera.getEstimatedRobotPose());
        }

        relativeRobotPoseSource.update();

        if (relativeRobotPoseSource.hasNewResult())
            addPoseSourceObservation(
                    relativeRobotPoseSource.getEstimatedRobotPose(),
                    relativeRobotPoseSource.getLatestResultTimestampSeconds(),
                    RelativeRobotPoseSourceConstants.STANDARD_DEVIATIONS
            );
    }

    private void updateFromAprilTagCameras() {
        final AprilTagCamera[] newResultCameras = getCamerasWithResults();
        sortCamerasByLatestResultTimestamp(newResultCameras);

        for (AprilTagCamera aprilTagCamera : newResultCameras)
            addPoseSourceObservation(
                    aprilTagCamera.getEstimatedRobotPose(),
                    aprilTagCamera.getLatestResultTimestampSeconds(),
                    aprilTagCamera.calculateStandardDeviations()
            );
    }

    /**
     * Checks if the current velocity of the slow enough to get an accurate enough result to reset the offset of the {@link RelativeRobotPoseSource}.
     *
     * @return if the robot is moving slow enough to calculate an accurate offset result.
     */
    private boolean isUnderMaximumSpeedForOffsetResetting() {
        final ChassisSpeeds chassisSpeeds = RobotContainer.SWERVE.getSelfRelativeVelocity();
        final double currentTranslationVelocityMetersPerSecond = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        final double currentThetaVelocityRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond;
        return currentTranslationVelocityMetersPerSecond <= PoseEstimatorConstants.MAXIMUM_TRANSLATION_VELOCITY_FOR_RELATIVE_ROBOT_POSE_SOURCE_OFFSET_RESETTING_METERS_PER_SECOND &&
                currentThetaVelocityRadiansPerSecond <= PoseEstimatorConstants.MAXIMUM_THETA_VELOCITY_FOR_RELATIVE_ROBOT_POSE_SOURCE_OFFSET_RESETTING_RADIANS_PER_SECOND;
    }

    private AprilTagCamera[] getCamerasWithResults() {
        final AprilTagCamera[] camerasWithNewResult = new AprilTagCamera[aprilTagCameras.length];
        int index = 0;

        for (AprilTagCamera aprilTagCamera : aprilTagCameras) {
            aprilTagCamera.update();
            if (aprilTagCamera.hasValidResult()) {
                camerasWithNewResult[index] = aprilTagCamera;
                index++;
            }
        }

        return Arrays.copyOf(camerasWithNewResult, index);
    }

    private void sortCamerasByLatestResultTimestamp(AprilTagCamera[] aprilTagCameras) {
        QuickSortHandler.sort(aprilTagCameras, AprilTagCamera::getLatestResultTimestampSeconds);
    }

    /**
     * Sets the estimated pose from a pose source at the given timestamp.
     *
     * @param observationPose the estimated robot pose
     * @param timestamp       the timestamp of the observation
     */
    private void addPoseSourceObservation(Pose2d observationPose, double timestamp, StandardDeviations standardDeviations) {
        final Pose2d odometryPoseAtTimestamp = getOdometryPoseAtTimestamp(timestamp);
        final Pose2d estimatedPoseAtObservationTime = calculateEstimatedPoseAtTimestamp(odometryPoseAtTimestamp);
        if (estimatedPoseAtObservationTime == null)
            return;

        final Pose2d estimatedPoseAtTimestampWithAmbiguityCompensation = calculateEstimatedPoseAtTimestampWithAmbiguityCompensation(estimatedPoseAtObservationTime, observationPose, standardDeviations);
        final Transform2d odometryPoseAtTimestampToCurrentOdometryPose = new Transform2d(odometryPoseAtTimestamp, odometryPose);
        this.estimatedPose = estimatedPoseAtTimestampWithAmbiguityCompensation.plus(odometryPoseAtTimestampToCurrentOdometryPose);
    }

    private Pose2d getOdometryPoseAtTimestamp(double timestamp) {
        if (isTimestampOutOfHeldPosesRange(timestamp))
            return null;

        final Optional<Pose2d> odometryPoseAtTimestamp = previousOdometryPoses.getSample(timestamp);
        return odometryPoseAtTimestamp.orElse(null);
    }

    private Pose2d calculateEstimatedPoseAtTimestamp(Pose2d odometryPoseAtTimestamp) {
        if (odometryPoseAtTimestamp == null)
            return null;

        final Transform2d currentPoseToSamplePose = new Transform2d(odometryPose, odometryPoseAtTimestamp);
        return estimatedPose.plus(currentPoseToSamplePose);
    }

    private boolean isTimestampOutOfHeldPosesRange(double timestamp) {
        try {
            final double oldestEstimatedRobotPoseTimestamp = previousOdometryPoses.getInternalBuffer().lastKey() - PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS;
            if (oldestEstimatedRobotPoseTimestamp > timestamp)
                return true;
        } catch (NoSuchElementException e) {
            return true;
        }
        return false;
    }

    /**
     * Calculates the estimated pose of a vision observation with compensation for its ambiguity.
     * This is done by finding the difference between the estimated pose at the time of the observation and the estimated pose of the observation and scaling that down using the calibrated standard deviations.
     * This will calculate for the pose at timestamp of `estimatedPoseAtObservationTime`.
     *
     * @param estimatedPoseAtObservationTime the estimated pose of the robot at the time of the observation
     * @param observationPose                the estimated robot pose from the observation
     * @param observationStandardDeviations  the ambiguity of the observation
     * @return the estimated pose with compensation for its ambiguity
     */
    private Pose2d calculateEstimatedPoseAtTimestampWithAmbiguityCompensation(Pose2d estimatedPoseAtObservationTime, Pose2d observationPose, StandardDeviations observationStandardDeviations) {
        final Transform2d observationDifference = new Transform2d(estimatedPoseAtObservationTime, observationPose);
        final Transform2d allowedMovement = calculateAllowedMovementFromAmbiguity(observationDifference, observationStandardDeviations);
        return estimatedPoseAtObservationTime.plus(allowedMovement);
    }

    /**
     * Calculates the scaling needed to reduce noise in the estimated pose from the standard deviations of the observation.
     *
     * @param observationDifference    the difference between the estimated pose of the robot at the time of the observation and the estimated pose of the observation
     * @param cameraStandardDeviations the standard deviations of the camera's estimated pose
     * @return the allowed movement of the estimated pose as a {@link Transform2d}
     */
    private Transform2d calculateAllowedMovementFromAmbiguity(Transform2d observationDifference, StandardDeviations cameraStandardDeviations) {
        final StandardDeviations combinedStandardDeviations = PoseEstimatorConstants.ODOMETRY_STANDARD_DEVIATIONS.combineWith(cameraStandardDeviations);
        return combinedStandardDeviations.scaleTransformFromStandardDeviations(observationDifference);
    }

    /**
     * Calculates the difference between the previous and current odometry poses.
     *
     * @param swerveModulePositions the current positions of each swerve module
     * @param gyroHeading           the current heading of the gyro
     * @return the difference as a {@link edu.wpi.first.math.geometry.Twist2d}
     */
    private Twist2d calculateNewOdometryPoseDifference(SwerveModulePosition[] swerveModulePositions, Rotation2d gyroHeading) {
        final Twist2d odometryDifferenceTwist2d = SwerveConstants.KINEMATICS.toTwist2d(lastSwerveModulePositions, swerveModulePositions);
        return new Twist2d(odometryDifferenceTwist2d.dx, odometryDifferenceTwist2d.dy, gyroHeading.minus(lastGyroHeading).getRadians());
    }

    private void updateOdometryPositions(SwerveModulePosition[] swerveModulePositions, Rotation2d gyroHeading) {
        lastSwerveModulePositions = swerveModulePositions;
        lastGyroHeading = gyroHeading;
    }

    private void updateRobotPosesFromNewOdometryPoseDifference(Twist2d newOdometryPoseDifference, double timestamp) {
        odometryPose = odometryPose.exp(newOdometryPoseDifference);
        estimatedPose = estimatedPose.exp(newOdometryPoseDifference);
        previousOdometryPoses.addSample(timestamp, odometryPose);
    }
}