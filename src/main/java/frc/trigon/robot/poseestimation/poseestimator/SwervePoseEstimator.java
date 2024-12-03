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
import java.util.Optional;

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

    public void addOdometryObservation(OdometryObservation observation) {
        final Twist2d odometryDifferenceTwist2d = calculateOdometryDifferenceTwist2d(observation);
        updateOdometryPositions(observation);
        updateRobotPosesFromOdometryDifferenceTwist2d(odometryDifferenceTwist2d, observation.timestamp());
    }

    public void addVisionObservation(VisionObservation observation) {
        if (isVisionObservationTooOld(observation.timestamp()))
            return;

        final Optional<Pose2d> odometrySample = previousOdometryPoses.getSample(observation.timestamp());
        if (odometrySample.isEmpty())
            return;

        final Transform2d odometryPoseToSamplePoseTransform = new Transform2d(odometryPose, odometrySample.get());
        final Pose2d poseEstimateAtObservationTime = estimatedPose.plus(odometryPoseToSamplePoseTransform);

        final double[] estimatedPoseStandardDeviations = getEstimatedPoseStandardDeviations(observation.standardDeviations());
        final Transform2d poseEstimateAtObservationTimeToObservationPose = new Transform2d(poseEstimateAtObservationTime, observation.estimatedPose());
        final Transform2d scaledPoseEstimateAtObservationTimeToObservationPose = scaleTransform(poseEstimateAtObservationTimeToObservationPose, estimatedPoseStandardDeviations);

        estimatedPose = poseEstimateAtObservationTime.plus(scaledPoseEstimateAtObservationTimeToObservationPose).plus(odometryPoseToSamplePoseTransform.inverse());
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

    private Twist2d calculateOdometryDifferenceTwist2d(OdometryObservation observation) {
        final Twist2d odometryDifferenceTwist2d = SwerveConstants.KINEMATICS.toTwist2d(lastSwerveModulePositions, observation.swerveModulePositions());
        odometryDifferenceTwist2d.dtheta = observation.gyroHeading().minus(lastGyroHeading).getRadians();

        return odometryDifferenceTwist2d;
    }

    private void updateOdometryPositions(OdometryObservation observation) {
        lastSwerveModulePositions = observation.swerveModulePositions();
        lastGyroHeading = observation.gyroHeading();
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

    private double[] getEstimatedPoseStandardDeviations(double[] observationStandardDeviations) {
        final double[] odometryStandardDeviations = PoseEstimatorConstants.AMBIGUITY;
        double[] estimatedPoseStandardDeviations = new double[observationStandardDeviations.length];

        for (int i = 0; i < estimatedPoseStandardDeviations.length; i++)
            estimatedPoseStandardDeviations[i] = odometryStandardDeviations[i] / (odometryStandardDeviations[i] + Math.sqrt(odometryStandardDeviations[i] * observationStandardDeviations[i]));

        return estimatedPoseStandardDeviations;
    }

    private Transform2d scaleTransform(Transform2d transform, double[] scalar) {
        return new Transform2d(
                transform.getX() * scalar[0],
                transform.getY() * scalar[1],
                transform.getRotation().times(scalar[2])
        );
    }

    public record OdometryObservation(
            SwerveModulePosition[] swerveModulePositions, Rotation2d gyroHeading, double timestamp) {
    }

    public record VisionObservation(
            Pose2d estimatedPose, double timestamp, double[] standardDeviations) {
    }
}
