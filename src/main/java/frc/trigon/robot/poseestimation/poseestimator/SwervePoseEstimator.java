package frc.trigon.robot.poseestimation.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.NoSuchElementException;

public class SwervePoseEstimator {
    private final TimeInterpolatableBuffer<Pose2d> previousOdometryPoses = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);

    private Pose2d
            odometryPose = new Pose2d(),
            estimatedPose = new Pose2d();
    private SwerveModulePosition[] lastSwerveModulePositions = new SwerveModulePosition[4];
    private Rotation2d lastGyroHeading = new Rotation2d();

    private static SwervePoseEstimator instance;

    public static SwervePoseEstimator getInstance() {
        if (instance == null)
            instance = new SwervePoseEstimator();
        return instance;
    }

    public void addOdometryObservation(SwerveModulePosition[] swerveModulePositions, Rotation2d gyroHeading, double timestamp) {
        final Twist2d odometryDifferenceTwist2d = calculateOdometryDifferenceTwist2d(swerveModulePositions, gyroHeading);
        updateOdometryPositions(swerveModulePositions, gyroHeading);
        updateRobotPosesFromOdometryDifferenceTwist2d(odometryDifferenceTwist2d, timestamp);
    }

    public void addVisionObservation(Pose2d estimatedPose, PoseEstimatorConstants.StandardDeviations standardDeviations, double timestamp) {
        if (isVisionObservationTooOld(timestamp))
            return;

        final Pose2d odometrySample = samplePose(timestamp);
        if (odometrySample == null)
            return;

        final Transform2d odometryPoseToSamplePoseTransform = new Transform2d(odometryPose, odometrySample);
        final Pose2d estimatedPoseAtObservationTime = estimatedPose.plus(odometryPoseToSamplePoseTransform);

        final Pose2d estimatedOdometryPose = estimatedPoseAtObservationTime.plus(odometryPoseToSamplePoseTransform.inverse());
        this.estimatedPose = estimatedOdometryPose.plus(calculatePoseStandardDeviations(estimatedPoseAtObservationTime, standardDeviations));
    }

    /**
     * Gets the estimated pose of the robot at the target timestamp.
     *
     * @param timestamp the target timestamp
     * @return the robot's estimated pose at the timestamp
     */
    public Pose2d samplePose(double timestamp) {
        final Pose2d samplePose = previousOdometryPoses.getSample(timestamp).orElse(new Pose2d());
        final Transform2d odometryPoseToSamplePoseTransform = new Transform2d(odometryPose, samplePose);

        return estimatedPose.plus(odometryPoseToSamplePoseTransform);
    }

    public void resetPose(Pose2d newPose) {
        odometryPose = newPose;
        estimatedPose = newPose;
        lastGyroHeading = newPose.getRotation();
        previousOdometryPoses.clear();
    }

    @AutoLogOutput
    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    @AutoLogOutput
    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    private Twist2d calculateOdometryDifferenceTwist2d(SwerveModulePosition[] swerveModulePositions, Rotation2d gyroHeading) {
        final Twist2d odometryDifferenceTwist2d = SwerveConstants.KINEMATICS.toTwist2d(lastSwerveModulePositions, swerveModulePositions);
        odometryDifferenceTwist2d.dtheta = gyroHeading.minus(lastGyroHeading).getRadians();

        return odometryDifferenceTwist2d;
    }

    private void updateOdometryPositions(SwerveModulePosition[] swerveModulePositions, Rotation2d gyroHeading) {
        lastSwerveModulePositions = swerveModulePositions;
        lastGyroHeading = gyroHeading;
    }

    private void updateRobotPosesFromOdometryDifferenceTwist2d(Twist2d OdometryDifferenceTwist2d, double timestamp) {
        odometryPose = odometryPose.exp(OdometryDifferenceTwist2d);
        estimatedPose = estimatedPose.exp(OdometryDifferenceTwist2d);
        previousOdometryPoses.addSample(timestamp, odometryPose);
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
}
