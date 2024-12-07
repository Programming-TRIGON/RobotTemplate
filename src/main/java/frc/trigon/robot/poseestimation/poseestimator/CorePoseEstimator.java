package frc.trigon.robot.poseestimation.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.NoSuchElementException;
import java.util.Optional;

public class CorePoseEstimator {
    private static CorePoseEstimator INSTANCE;

    public static CorePoseEstimator getInstance() {
        if (INSTANCE == null)
            INSTANCE = new CorePoseEstimator();
        return INSTANCE;
    }

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
    private final Matrix<N3, N1> standardDeviations = new Matrix<>(Nat.N3(), Nat.N1());
    private final SwerveDriveKinematics swerveDriveKinematics;
    private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[4];
    private Rotation2d lastGyroYaw = new Rotation2d();
    private Pose2d
            odometryPose = new Pose2d(),
            estimatedPose = new Pose2d();

    private CorePoseEstimator() {
        for (int i = 0; i < standardDeviations.getNumRows(); i++)
            standardDeviations.set(i, 0, Math.pow(PoseEstimatorConstants.ODOMETRY_AMBIGUITY.get(i, 0), 2));
        swerveDriveKinematics = SwerveConstants.KINEMATICS;
    }

    /**
     * Updates the estimated pose of the robot based on the odometry position.
     *
     * @param currentOdometryPose the odometry position to update the estimated pose with
     */
    public void updateEstimatedOdometryPose(OdometryEstimatedPose currentOdometryPose) {
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
    public void updateEstimatedVisionPose(VisionEstimatedPose currentVisionPose) {
        if (!isPositionInBufferRange(currentVisionPose.timestamp()))
            return;

        final Optional<Pose2d> currentPose = poseBuffer.getSample(currentVisionPose.timestamp());
        if (currentPose.isEmpty())
            return;

        final Matrix<N3, N3> visionKalmanGains = getVisionKalmanGains();
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
     * Reset estimated pose and odometry pose to pose <br>
     * Clear pose buffer
     */
    public void resetPose(Pose2d initialPose) {
        estimatedPose = initialPose;
        odometryPose = initialPose;
        lastGyroYaw = initialPose.getRotation();
        poseBuffer.clear();
    }

    @AutoLogOutput(key = "Poses/Robot/EstimatedPose")
    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    @AutoLogOutput(key = "Poses/Robot/OdometryPose")
    public Pose2d getOdometryPose() {
        return odometryPose;
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

    private Matrix<N3, N3> getVisionKalmanGains() {
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

    public record OdometryEstimatedPose(SwerveModulePosition[] wheelPositions, Rotation2d gyroYaw, double timestamp) {
    }

    public record VisionEstimatedPose(Pose2d visionPosition, double timestamp, Matrix<N3, N1> standardDeviations) {
    }
}