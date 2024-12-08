package frc.trigon.robot.poseestimation.poseestimator;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

import java.util.*;

/**
 * A class that estimates the robot's pose using team 6328's custom pose estimator.
 */
public class PoseEstimator implements AutoCloseable {
    private final Field2d field = new Field2d();
    private final AprilTagCamera[] aprilTagCameras;
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
    private final Matrix<N3, N1> standardDeviations = new Matrix<>(Nat.N3(), Nat.N1());
    private final SwerveDriveKinematics swerveDriveKinematics;
    private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[4];
    private Rotation2d lastGyroYaw = new Rotation2d();
    private Pose2d
            odometryPose = new Pose2d(),
            estimatedPose = new Pose2d();

    /**
     * Constructs a new PoseEstimator.
     *
     * @param aprilTagCameras the cameras that should be used to update the pose estimator.
     */
    public PoseEstimator(AprilTagCamera... aprilTagCameras) {
        this.aprilTagCameras = aprilTagCameras;
        putAprilTagsOnFieldWidget();
        SmartDashboard.putData("Field", field);

        for (int i = 0; i < standardDeviations.getNumRows(); i++)
            standardDeviations.set(i, 0, Math.pow(PoseEstimatorConstants.ODOMETRY_AMBIGUITY.get(i, 0), 2));
        swerveDriveKinematics = SwerveConstants.KINEMATICS;

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
     * @param currentPose the pose to reset to, relative to the blue alliance's driver station right corner
     */
    public void resetPose(Pose2d currentPose) {
        RobotContainer.SWERVE.setHeading(currentPose.getRotation());
        estimatedPose = currentPose;
        odometryPose = currentPose;
        lastGyroYaw = currentPose.getRotation();
        poseBuffer.clear();
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
            updateEstimatedOdometryPose(new PoseEstimatorConstants.OdometryEstimatedPose(swerveWheelPositions[i], gyroRotations[i], timestamps[i]));
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
     * Updates the estimated pose of the robot based on the odometry position.
     *
     * @param currentOdometryPose the odometry position to update the estimated pose with
     */
    public void updateEstimatedOdometryPose(PoseEstimatorConstants.OdometryEstimatedPose currentOdometryPose) {
        final Twist2d odometryTwist = swerveDriveKinematics.toTwist2d(lastWheelPositions, currentOdometryPose.wheelPositions());
        lastWheelPositions = currentOdometryPose.wheelPositions();

        if (currentOdometryPose.gyroYaw() != null) {
            odometryTwist.dtheta = currentOdometryPose.gyroYaw().minus(lastGyroYaw).getRadians();
            lastGyroYaw = currentOdometryPose.gyroYaw();
        }

        odometryPose = odometryPose.exp(odometryTwist);
        estimatedPose = estimatedPose.exp(odometryTwist);
        poseBuffer.addSample(currentOdometryPose.timestamp(), odometryPose);
    }

    /**
     * Update the estimated pose of the robot based on the vision position.
     *
     * @param currentVisionPose the vision position to update the estimated pose with
     */
    public void updateEstimatedVisionPose(PoseEstimatorConstants.VisionEstimatedPose currentVisionPose) {
        if (!isPositionInBufferRange(currentVisionPose.timestamp()))
            return;

        final Optional<Pose2d> currentPose = poseBuffer.getSample(currentVisionPose.timestamp());
        if (currentPose.isEmpty())
            return;

        final Matrix<N3, N3> visionKalmanGains = getVisionKalmanGains(currentVisionPose.standardDeviations());
        final Transform2d currentPoseToOdometryPose = new Transform2d(currentPose.get(), odometryPose);
        final Pose2d currentEstimatedPose = estimatedPose.plus(currentPoseToOdometryPose.inverse());
        final Transform2d estimatedPoseToVisionPose = new Transform2d(currentEstimatedPose, currentVisionPose.visionPosition());
        final Matrix<N3, N1> visionCorrectionMatrix = visionKalmanGains.times(VecBuilder.fill(estimatedPoseToVisionPose.getX(), estimatedPoseToVisionPose.getY(), estimatedPoseToVisionPose.getRotation().getRadians()));
        Transform2d scaledEstimatedPoseToVisionPose = new Transform2d(visionCorrectionMatrix.get(0, 0), visionCorrectionMatrix.get(1, 0), Rotation2d.fromRadians(visionCorrectionMatrix.get(2, 0)));
        estimatedPose = estimatedPose.plus(scaledEstimatedPoseToVisionPose).plus(currentPoseToOdometryPose);
    }

    /**
     * Gets the estimated pose of the robot at the target timestamp.
     *
     * @param timestamp the target timestamp
     * @return the robot's estimated pose at the timestamp
     */
    public Pose2d getPoseAtTimestamp(double timestamp) {
        final Pose2d poseAtTimestamp = poseBuffer.getSample(timestamp).orElse(new Pose2d());
        final Transform2d odometryToPoseAtTimestamp = new Transform2d(odometryPose, poseAtTimestamp);
        return estimatedPose.plus(odometryToPoseAtTimestamp);
    }

    /**
     * @return the estimated pose of the robot, relative to the blue alliance's driver station right corner
     */
    @AutoLogOutput(key = "Poses/Robot/EstimatedPose")
    public Pose2d getCurrentEstimatedPose() {
        return estimatedPose;
    }

    /**
     * @return the odometry's estimated pose of the robot, relative to the blue alliance's driver station right corner
     */
    @AutoLogOutput(key = "Poses/Robot/OdometryPose")
    public Pose2d getCurrentOdometryPose() {
        return odometryPose;
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

    private void updateFromVision() {
        getViableVisionObservations().stream()
                .sorted(Comparator.comparingDouble(PoseEstimatorConstants.VisionEstimatedPose::timestamp))
                .forEach(this::updateEstimatedVisionPose);
    }

    private List<PoseEstimatorConstants.VisionEstimatedPose> getViableVisionObservations() {
        List<PoseEstimatorConstants.VisionEstimatedPose> viableVisionObservations = new ArrayList<>();
        for (AprilTagCamera aprilTagCamera : aprilTagCameras) {
            final PoseEstimatorConstants.VisionEstimatedPose visionObservation = getVisionObservation(aprilTagCamera);
            if (visionObservation != null)
                viableVisionObservations.add(visionObservation);
        }
        return viableVisionObservations;
    }

    private PoseEstimatorConstants.VisionEstimatedPose getVisionObservation(AprilTagCamera aprilTagCamera) {
        aprilTagCamera.update();
        if (!aprilTagCamera.hasNewResult())
            return null;
        final Pose2d robotPose = aprilTagCamera.getEstimatedRobotPose();
        if (robotPose == null || robotPose.getTranslation() == null || robotPose.getRotation() == null)
            return null;

        return new PoseEstimatorConstants.VisionEstimatedPose(
                robotPose,
                aprilTagCamera.getLatestResultTimestampSeconds(),
                aprilTagCamera.calculateStandardDeviations()
        );
    }

    private void putAprilTagsOnFieldWidget() {
        for (Map.Entry<Integer, Pose3d> entry : FieldConstants.TAG_ID_TO_POSE.entrySet()) {
            final Pose2d tagPose = entry.getValue().toPose2d();
            field.getObject("Tag " + entry.getKey()).setPose(tagPose);
        }
    }

    private boolean isPositionInBufferRange(double timestamp) {
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS > timestamp) {
                return false;
            }
        } catch (NoSuchElementException ex) {
            return false;
        }
        return true;
    }

    private Matrix<N3, N3> getVisionKalmanGains(Matrix<N3, N1> standardDeviations) {
        final double[] noiseMeasurements = getNoiseMeasurements(standardDeviations);
        final Matrix<N3, N3> visionKalmanGains = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < visionKalmanGains.getNumRows(); row++) {
            double standardDeviation = standardDeviations.get(row, 0);
            if (standardDeviation == 0) {
                visionKalmanGains.set(row, row, 0);
                continue;
            }
            visionKalmanGains.set(row, row, standardDeviation / (standardDeviation + Math.sqrt(standardDeviation * noiseMeasurements[row])));
        }
        return visionKalmanGains;
    }

    private double[] getNoiseMeasurements(Matrix<N3, N1> standardDeviations) {
        final double[] noiseMeasurements = new double[3];
        for (int i = 0; i < noiseMeasurements.length; i++)
            noiseMeasurements[i] = Math.pow(standardDeviations.get(i, 0), 2);
        return noiseMeasurements;
    }
}