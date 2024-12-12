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

import java.util.Map;
import java.util.NoSuchElementException;

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
    private SwerveModulePosition[] lastSwerveModulePositions = new SwerveModulePosition[4];
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
     * Sets the odometry position at a given timestamp.
     *
     * @param swerveModulePositions the positions of each swerve module
     * @param gyroHeading           the heading of the gyro
     * @param timestamp             the timestamp of the odometry observation
     */
    private void addOdometryObservation(SwerveModulePosition[] swerveModulePositions, Rotation2d gyroHeading, double timestamp) {
        final Twist2d newOdometryPoseDifference = calculateNewOdometryPoseDifference(swerveModulePositions, gyroHeading);
        updateOdometryPositions(swerveModulePositions, gyroHeading);
        updateRobotPosesFromNewOdometryPoseDifference(newOdometryPoseDifference, timestamp);
    }

    private void updateFromVision() {
        updateAllCameras();
        sortCamerasByLastTargetTimestamp();
        addVisionObservations();
    }

    private void updateAllCameras() {
        for (AprilTagCamera aprilTagCamera : aprilTagCameras) {
            aprilTagCamera.update();
        }
    }

    private void sortCamerasByLastTargetTimestamp() {
        VisionObservationSorter.quickSortByDoubleValue(aprilTagCameras, (aprilTagCamera) -> ((AprilTagCamera) aprilTagCamera).getLatestResultTimestampSeconds());
    }

    /**
     * Sets the estimated vision pose at the timestamps of the results from the cameras.
     */
    private void addVisionObservations() {
        for (AprilTagCamera aprilTagCamera : aprilTagCameras) {
            if (!aprilTagCamera.hasNewResult())
                continue;

            addVisionObservation(
                    getCameraEstimatedPose(aprilTagCamera),
                    aprilTagCamera.calculateStandardDeviations(),
                    aprilTagCamera.getLatestResultTimestampSeconds()
            );
        }
    }

    private Pose2d getCameraEstimatedPose(AprilTagCamera aprilTagCamera) {
        final Pose2d robotPose = aprilTagCamera.getEstimatedRobotPose();
        if (robotPose == null || robotPose.getTranslation() == null || robotPose.getRotation() == null)
            return null;
        return robotPose;
    }

    private void addVisionObservation(Pose2d estimatedPose, PoseEstimatorConstants.StandardDeviations standardDeviations, double timestamp) {
        if (isVisionObservationTooOld(timestamp))
            return;

        final Pose2d odometrySample = getPoseSample(timestamp);
        if (odometrySample == null)
            return;

        final Transform2d odometryPoseToSamplePoseTransform = new Transform2d(odometryPose, odometrySample);
        final Pose2d estimatedPoseAtObservationTime = estimatedPose.plus(odometryPoseToSamplePoseTransform);

        final Pose2d estimatedOdometryPose = estimatedPoseAtObservationTime.plus(odometryPoseToSamplePoseTransform.inverse());
        this.estimatedPose = estimatedOdometryPose.plus(calculatePoseStandardDeviations(estimatedPoseAtObservationTime, standardDeviations));
    }

    private boolean isVisionObservationTooOld(double timestamp) {
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
     * Gets the estimated pose of the robot at the target timestamp.
     *
     * @param timestamp the target timestamp
     * @return the robot's estimated pose at the timestamp
     */
    public Pose2d getPoseSample(double timestamp) {
        final Pose2d samplePose = previousOdometryPoses.getSample(timestamp).orElse(new Pose2d());
        final Transform2d odometryPoseToSamplePoseTransform = new Transform2d(odometryPose, samplePose);

        return estimatedPose.plus(odometryPoseToSamplePoseTransform);
    }

    private Transform2d calculatePoseStandardDeviations(Pose2d estimatedPoseAtObservationTime, PoseEstimatorConstants.StandardDeviations cameraStandardDeviations) {
        final Transform2d poseEstimateAtObservationTimeToObservationPose = new Transform2d(estimatedPoseAtObservationTime, estimatedPose);
        final PoseEstimatorConstants.StandardDeviations estimatedPoseStandardDeviations = combineOdometryAndVisionStandardDeviations(cameraStandardDeviations);
        return scaleTransformFromStandardDeviations(poseEstimateAtObservationTimeToObservationPose, estimatedPoseStandardDeviations);
    }

    private PoseEstimatorConstants.StandardDeviations combineOdometryAndVisionStandardDeviations(PoseEstimatorConstants.StandardDeviations observationStandardDeviation) {
        final PoseEstimatorConstants.StandardDeviations odometryStandardDeviations = PoseEstimatorConstants.ODOMETRY_STANDARD_DEVIATIONS;

        return new PoseEstimatorConstants.StandardDeviations(
                combineOdometryAndVisionStandardDeviation(odometryStandardDeviations.translation(), observationStandardDeviation.translation()),
                combineOdometryAndVisionStandardDeviation(odometryStandardDeviations.theta(), observationStandardDeviation.theta())
        );
    }

    /**
     * Combines a standard deviation value of the odometry and of the vision result to get a new standard deviation.
     *
     * @param odometryStandardDeviation a standard deviation value of the estimated odometry pose
     * @param visionStandardDeviation   a standard deviation value of the estimated vision pose
     * @return the combined standard deviation
     */
    private double combineOdometryAndVisionStandardDeviation(double odometryStandardDeviation, double visionStandardDeviation) {
        return odometryStandardDeviation / (odometryStandardDeviation + Math.sqrt(odometryStandardDeviation * visionStandardDeviation));
    }

    private Transform2d scaleTransformFromStandardDeviations(Transform2d transform, PoseEstimatorConstants.StandardDeviations standardDeviations) {
        return new Transform2d(
                transform.getX() * standardDeviations.translation(),
                transform.getY() * standardDeviations.translation(),
                transform.getRotation().times(standardDeviations.theta())
        );
    }

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