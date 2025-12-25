package frc.trigon.robot.poseestimation.poseestimator;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCamera;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSource;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceConstants;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;
import frc.trigon.lib.utilities.QuickSortHandler;
import frc.trigon.lib.utilities.flippable.Flippable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Map;

/**
 * A class that estimates the robot's pose using WPILib's {@link SwerveDrivePoseEstimator} and {@link SwerveDriveOdometry}.
 */
public class PoseEstimator implements AutoCloseable {
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator = createSwerveDrivePoseEstimator();
    private final SwerveDriveOdometry swerveDriveOdometry = createSwerveDriveOdometry();
    private final Field2d field = new Field2d();
    private final AprilTagCamera[] aprilTagCameras;
    private final RelativeRobotPoseSource relativeRobotPoseSource;
    private final boolean shouldUseRelativeRobotPoseSource;

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

        field.setRobotPose(getEstimatedRobotPose());
    }

    public void resetHeading() {
        final Rotation2d resetRotation = Flippable.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero;
        swerveDrivePoseEstimator.resetRotation(resetRotation);
        swerveDriveOdometry.resetRotation(resetRotation);
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param newPose the pose to reset to, relative to the blue alliance's driver station right corner
     */
    public void resetPose(Pose2d newPose) {
        RobotContainer.SWERVE.setHeading(newPose.getRotation());

        swerveDrivePoseEstimator.resetPose(newPose);
        swerveDriveOdometry.resetPose(newPose);
        if (shouldUseRelativeRobotPoseSource)
            relativeRobotPoseSource.resetOffset(newPose);
    }

    public void resetOdometry() {
        swerveDriveOdometry.resetPose(getEstimatedRobotPose());
    }

    /**
     * @return the estimated pose of the robot, relative to the blue alliance's driver station right corner
     */
    @AutoLogOutput(key = "Poses/Robot/PoseEstimator/EstimatedRobotPose")
    public Pose2d getEstimatedRobotPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    /**
     * @return the odometry's estimated pose of the robot, relative to the blue alliance's driver station right corner
     */
    @AutoLogOutput(key = "Poses/Robot/PoseEstimator/EstimatedOdometryPose")
    public Pose2d getEstimatedOdometryPose() {
        return swerveDriveOdometry.getPoseMeters();
    }

    /**
     * Updates the pose estimator with the given swerve wheel positions and gyro rotations.
     * This function accepts an array of swerve wheel positions and an array of gyro rotations because the odometry can be updated at a faster rate than the main loop (which is 50 hertz).
     * This means you could have a couple of odometry updates per main loop, and you would want to update the pose estimator with all of them.
     *
     * @param swerveWheelPositions the swerve wheel positions accumulated since the last update
     * @param gyroRotations        the gyro rotations accumulated since the last update
     */
    public void updatePoseEstimatorOdometry(SwerveModulePosition[][] swerveWheelPositions, Rotation2d[] gyroRotations, double[] timestamps) {
        for (int i = 0; i < swerveWheelPositions.length; i++) {
            swerveDrivePoseEstimator.updateWithTime(timestamps[i], gyroRotations[i], swerveWheelPositions[i]);
            swerveDriveOdometry.update(gyroRotations[i], swerveWheelPositions[i]);
        }
    }

    /**
     * Gets the estimated pose of the robot at the target timestamp.
     * Unlike {@link #getPredictedRobotFuturePose} which predicts a future pose, this gets a stored pose from the estimator's buffer.
     *
     * @param timestamp the Rio's FPGA timestamp
     * @return the robot's estimated pose at the timestamp
     */
    public Pose2d samplePoseAtTimestamp(double timestamp) {
        return swerveDrivePoseEstimator.sampleAt(timestamp).orElse(null);
    }

    /**
     * Predicts the robot's pose after the specified time.
     * Unlike {@link #samplePoseAtTimestamp(double)} which gets a previous pose from the buffer, this predicts the future pose of the robot.
     *
     * @param seconds the number of seconds into the future to predict the robot's pose for
     * @return the predicted pose
     */
    public Pose2d getPredictedRobotPose(double seconds) {
        final ChassisSpeeds robotVelocity = RobotContainer.SWERVE.getSelfRelativeChassisSpeeds();
        final double predictedX = robotVelocity.vxMetersPerSecond * seconds;
        final double predictedY = robotVelocity.vyMetersPerSecond * seconds;
        final Rotation2d predictedRotation = Rotation2d.fromRadians(robotVelocity.omegaRadiansPerSecond * seconds);
        return getEstimatedRobotPose().transformBy(new Transform2d(predictedX, predictedY, predictedRotation));
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
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            RobotContainer.SWERVE.setTargetPathPlannerPose(pose);
            Logger.recordOutput("PathPlanner/TargetPose", pose);
        });
    }

    private void updateFromRelativeRobotPoseSource() {
        for (AprilTagCamera aprilTagCamera : aprilTagCameras) {
            aprilTagCamera.update();

            if (aprilTagCamera.isWithinBestTagRangeForAccurateSolvePNPResult() && isUnderMaximumSpeedForOffsetResetting())
                relativeRobotPoseSource.resetOffset(aprilTagCamera.getEstimatedRobotPose());
        }

        relativeRobotPoseSource.update();

        if (relativeRobotPoseSource.hasNewResult()) {
            swerveDrivePoseEstimator.addVisionMeasurement(
                    relativeRobotPoseSource.getEstimatedRobotPose(),
                    relativeRobotPoseSource.getLatestResultTimestampSeconds(),
                    RelativeRobotPoseSourceConstants.STANDARD_DEVIATIONS.toMatrix()
            );
        }
    }

    private void updateFromAprilTagCameras() {
        final AprilTagCamera[] newResultCameras = getCamerasWithResults();
//        sortCamerasByLatestResultTimestamp(newResultCameras);

        for (AprilTagCamera aprilTagCamera : newResultCameras) {
            swerveDrivePoseEstimator.addVisionMeasurement(
                    aprilTagCamera.getEstimatedRobotPose(),
                    aprilTagCamera.getLatestResultTimestampSeconds(),
                    aprilTagCamera.calculateStandardDeviations().toMatrix()
            );
        }
    }

    /**
     * Checks if the current velocity of the slow enough to get an accurate enough result to reset the offset of the {@link RelativeRobotPoseSource}.
     *
     * @return if the robot is moving slow enough to calculate an accurate offset result.
     */
    private boolean isUnderMaximumSpeedForOffsetResetting() {
        final ChassisSpeeds chassisSpeeds = RobotContainer.SWERVE.getSelfRelativeChassisSpeeds();
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
            if (aprilTagCamera.hasValidResult() && aprilTagCamera.getEstimatedRobotPose() != null) {
                camerasWithNewResult[index] = aprilTagCamera;
                index++;
            }
        }

        return Arrays.copyOf(camerasWithNewResult, index);
    }

    private void sortCamerasByLatestResultTimestamp(AprilTagCamera[] aprilTagCameras) {
        QuickSortHandler.sort(aprilTagCameras, AprilTagCamera::getLatestResultTimestampSeconds);
    }

    private SwerveDriveOdometry createSwerveDriveOdometry() {
        final SwerveModulePosition[] swerveModulePositions = {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };

        return new SwerveDriveOdometry(
                SwerveConstants.KINEMATICS,
                new Rotation2d(),
                swerveModulePositions
        );
    }

    private SwerveDrivePoseEstimator createSwerveDrivePoseEstimator() {
        final SwerveModulePosition[] swerveModulePositions = {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };

        return new SwerveDrivePoseEstimator(
                SwerveConstants.KINEMATICS,
                new Rotation2d(),
                swerveModulePositions,
                new Pose2d(),
                PoseEstimatorConstants.ODOMETRY_STANDARD_DEVIATIONS.toMatrix(),
                VecBuilder.fill(0, 0, 0)
        );
    }
}