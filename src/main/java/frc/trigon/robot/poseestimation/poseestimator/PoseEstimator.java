package frc.trigon.robot.poseestimation.poseestimator;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
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

    private RelativeRobotPoseSource relativeRobotPoseSource;
    private boolean shouldUseRelativeRobotPoseSource = false;

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
     *
     * @param relativeRobotPoseSource the relative robot pose source that should be used to update the pose estimator
     * @param aprilTagCameras         the apriltag cameras that should be used to update the relative robot pose source
     */
    public PoseEstimator(RelativeRobotPoseSource relativeRobotPoseSource, AprilTagCamera... aprilTagCameras) {
        this(aprilTagCameras);

        this.relativeRobotPoseSource = relativeRobotPoseSource;
        shouldUseRelativeRobotPoseSource = true;

    }

    /**
     * Constructs a new PoseEstimator.
     *
     * @param aprilTagCameras the cameras that should be used to update the pose estimator
     */
    public PoseEstimator(AprilTagCamera... aprilTagCameras) {
        this.aprilTagCameras = aprilTagCameras;
        putAprilTagsOnFieldWidget();
        SmartDashboard.putData("Field", field);

        logTargetPath();
    }

    @Override
    public void close() {
        field.close();
    }

    public void periodic() {
        if (shouldUseRelativeRobotPoseSource)
            updateFromRelativeRobotPoseSource();
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
    public Pose2d getPoseAtTimestamp(double timestamp) {
        final Optional<Pose2d> odometryPoseAtTimestamp = previousOdometryPoses.getSample(timestamp);
        if (odometryPoseAtTimestamp.isEmpty())
            return null;

        final Transform2d currentOdometryPoseToOdometryPoseAtTimestampTransform = new Transform2d(odometryPose, odometryPoseAtTimestamp.get());
        return estimatedPose.plus(currentOdometryPoseToOdometryPoseAtTimestampTransform);
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
            Logger.recordOutput("Path", pathPoses.toArray(new Pose2d[0]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> Logger.recordOutput("TargetPPPose", pose));
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

            if (aprilTagCamera.isWithinBestTagRangeForAccurateSolvePNPResult())
                relativeRobotPoseSource.resetOffset(aprilTagCamera.getRobotPose());
        }

        relativeRobotPoseSource.updatePeriodically();

        addPoseSourceObservation(
                relativeRobotPoseSource.getEstimatedRobotPose(),
                relativeRobotPoseSource.getLatestResultTimestampSeconds(),
                RelativeRobotPoseSourceConstants.STANDARD_DEVIATIONS
        );
    }

    /**
     * Sets the estimated pose from a pose source at the given timestamp.
     *
     * @param estimatedPose the estimated robot pose
     * @param timestamp     the timestamp of the observation
     */
    private void addPoseSourceObservation(Pose2d estimatedPose, double timestamp, StandardDeviations standardDeviations) {
        if (isObservationTooOld(timestamp))
            return;

        final Pose2d poseSampleAtObservationTimestamp = getPoseAtTimestamp(timestamp);
        if (poseSampleAtObservationTimestamp == null)
            return;

        final Transform2d odometryPoseToSamplePoseTransform = new Transform2d(odometryPose, poseSampleAtObservationTimestamp);
        final Pose2d estimatedPoseAtObservationTime = estimatedPose.plus(odometryPoseToSamplePoseTransform);

        this.estimatedPose = estimatedPose.plus(calculatePoseStandardDeviations(estimatedPoseAtObservationTime, standardDeviations));
    }

    private boolean isObservationTooOld(double timestamp) {
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
     *
     * @param estimatedPoseAtObservationTime the estimated pose of the robot at the time of the observation
     * @param observationPose                the estimated robot pose from the observation
     * @param observationStandardDeviations  the ambiguity of the observation
     * @return the estimated pose with compensation for its ambiguity
     */
    private Pose2d calculateEstimatedPoseWithAmbiguityCompensation(Pose2d estimatedPoseAtObservationTime, Pose2d observationPose, StandardDeviations observationStandardDeviations) {
        final Transform2d estimatedPoseAtObservationTimeToObservationPose = new Transform2d(estimatedPoseAtObservationTime, observationPose);
        final Transform2d allowedMovement = calculateAllowedMovementFromAmbiguity(estimatedPoseAtObservationTimeToObservationPose, observationStandardDeviations);
        return observationPose.plus(allowedMovement);
    }

    /**
     * Calculates the scaling needed to reduce noise in the estimated pose from the standard deviations of the observation.
     *
     * @param estimatedPoseAtObservationTimeToObservationPose the difference between the estimated pose of the robot at the time of the observation and the estimated pose of the observation
     * @param cameraStandardDeviations                        the standard deviations of the camera's estimated pose
     * @return the allowed movement of the estimated pose as a {@link Transform2d}
     */
    private Transform2d calculateAllowedMovementFromAmbiguity(Transform2d estimatedPoseAtObservationTimeToObservationPose, StandardDeviations cameraStandardDeviations) {
        final StandardDeviations estimatedPoseStandardDeviations = cameraStandardDeviations.combineWith(PoseEstimatorConstants.ODOMETRY_STANDARD_DEVIATIONS);
        return estimatedPoseStandardDeviations.scaleTransformFromStandardDeviations(estimatedPoseAtObservationTimeToObservationPose);
    }

    private Transform2d calculatePoseStandardDeviations(Pose2d estimatedPoseAtObservationTime, StandardDeviations observationStandardDeviations) {
        final Transform2d poseEstimateAtObservationTimeToObservationPose = new Transform2d(estimatedPoseAtObservationTime, estimatedPose);
        return observationStandardDeviations.scaleTransformFromStandardDeviations(poseEstimateAtObservationTimeToObservationPose);
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