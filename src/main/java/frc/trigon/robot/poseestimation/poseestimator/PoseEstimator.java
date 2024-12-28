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
import frc.trigon.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.utilities.QuickSort;

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
     * Constructs a new PoseEstimator.
     *
     * @param aprilTagCameras the cameras that should be used to update the pose estimator.
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
        updateFromVision();
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

    public void setGyroHeadingToBestSolvePNPHeading() {
        if (aprilTagCameras.length == 0)
            return;
        int closestCameraToTag = 0;
        for (int i = 0; i < aprilTagCameras.length; i++) {
            if (aprilTagCameras[i].getDistanceToBestTagMeters() < aprilTagCameras[closestCameraToTag].getDistanceToBestTagMeters())
                closestCameraToTag = i;
        }

        final Rotation2d bestRobotHeading = aprilTagCameras[closestCameraToTag].getSolvePNPHeading();
        resetPose(new Pose2d(getCurrentEstimatedPose().getTranslation(), bestRobotHeading));
    }

    /**
     * Gets the estimated pose of the robot at the target timestamp.
     *
     * @param timestamp the target timestamp
     * @return the robot's estimated pose at the timestamp
     */
    public Pose2d getPoseAtTimestamp(double timestamp) {
        final Optional<Pose2d> poseAtTimestamp = previousOdometryPoses.getSample(timestamp);
        if (poseAtTimestamp.isEmpty())
            return null;

        final Transform2d odometryPoseToPoseAtTimestampTransform = new Transform2d(odometryPose, poseAtTimestamp.get());
        return estimatedPose.plus(odometryPoseToPoseAtTimestampTransform);
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
     * Sets the estimated pose from the odometry at the given timestamp.
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

    /**
     * Updates the estimated pose from the cameras.
     */
    private void updateFromVision() {
        final AprilTagCamera[] newResultCameras = getCamerasWithNewResult();
        sortCamerasByLatestResultTimestamp(newResultCameras);

        updateEstimatedPoseFromVision(newResultCameras);
    }

    private AprilTagCamera[] getCamerasWithNewResult() {
        final AprilTagCamera[] camerasWithNewResult = new AprilTagCamera[aprilTagCameras.length];
        int index = 0;

        for (AprilTagCamera aprilTagCamera : aprilTagCameras) {
            aprilTagCamera.update();
            if (aprilTagCamera.hasNewResult()) {
                camerasWithNewResult[index] = aprilTagCamera;
                index++;
            }
        }

        return Arrays.copyOf(camerasWithNewResult, index);
    }

    private void sortCamerasByLatestResultTimestamp(AprilTagCamera[] aprilTagCameras) {
        QuickSort.sort(aprilTagCameras, AprilTagCamera::getLatestResultTimestampSeconds);
    }

    /**
     * Updates the estimated pose from each camera, each at the timestamp of their latest result.
     */
    private void updateEstimatedPoseFromVision(AprilTagCamera[] aprilTagCameras) {
        for (AprilTagCamera aprilTagCamera : aprilTagCameras) {
            addVisionObservation(
                    aprilTagCamera.getEstimatedRobotPose(),
                    aprilTagCamera.calculateStandardDeviations(),
                    aprilTagCamera.getLatestResultTimestampSeconds()
            );
        }
    }

    private void addVisionObservation(Pose2d estimatedPose, StandardDeviations standardDeviations, double timestamp) {
        if (isTimestampOutOfPreviousHeldPosesRange(timestamp))
            return;

        final Pose2d estimatedPoseAtTimestamp = getPoseAtTimestamp(timestamp);
        if (estimatedPoseAtTimestamp == null)
            return;

        final Pose2d estimatedPoseWithAmbiguityCompensation = calculateEstimatedPoseWithAmbiguityCompensation(estimatedPoseAtTimestamp, estimatedPose, standardDeviations);
        final Transform2d estimatedPoseAtTimestampToOdometryPose = new Transform2d(estimatedPoseAtTimestamp, odometryPose);

        this.estimatedPose = estimatedPoseWithAmbiguityCompensation.plus(estimatedPoseAtTimestampToOdometryPose);
    }

    /**
     * Checks if the buffer that stores the previous poses of the robot still has a pose stored at the target timestamp.
     *
     * @param timestamp the target timestamp to check
     * @return whether the buffer contains a value at that timestamp or not
     */
    private boolean isTimestampOutOfPreviousHeldPosesRange(double timestamp) {
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
     * @param estimatedPoseAtTimestamp      the estimated pose of the robot at the time of the observation
     * @param observationEstimatedPose      the estimated robot pose from the observation
     * @param observationStandardDeviations the ambiguity of the observation
     * @return the estimated pose with compensation for its ambiguity
     */
    private Pose2d calculateEstimatedPoseWithAmbiguityCompensation(Pose2d estimatedPoseAtTimestamp, Pose2d observationEstimatedPose, StandardDeviations observationStandardDeviations) {
        final Transform2d estimatedPoseAtTimestampToEstimatedPose = new Transform2d(estimatedPoseAtTimestamp, observationEstimatedPose);
        final Transform2d allowedMovement = calculateAllowedMovementFromAmbiguity(estimatedPoseAtTimestampToEstimatedPose, observationStandardDeviations);
        return estimatedPoseAtTimestamp.plus(allowedMovement);
    }

    /**
     * Calculates the scaling needed to reduce noise in the estimated pose from the standard deviations of the observation.
     *
     * @param estimatedPoseAtTimestampToEstimatedPose the difference between the estimated pose of the robot at the time of the observation and the estimated pose of the observation
     * @param cameraStandardDeviations                the standard deviations of the camera's estimated pose
     * @return the allowed movement of the estimated pose as a {@link Transform2d}
     */
    private Transform2d calculateAllowedMovementFromAmbiguity(Transform2d estimatedPoseAtTimestampToEstimatedPose, StandardDeviations cameraStandardDeviations) {
        final StandardDeviations estimatedPoseStandardDeviations = cameraStandardDeviations.combineWith(PoseEstimatorConstants.ODOMETRY_STANDARD_DEVIATIONS);
        return estimatedPoseStandardDeviations.scaleTransformFromStandardDeviations(estimatedPoseAtTimestampToEstimatedPose);
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