package frc.trigon.robot.poseestimation.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class PoseEstimatorConstants {
    public static final double ODOMETRY_FREQUENCY_HERTZ = 250;

    /**
     * The vector represents how ambiguous each value of the odometry is.
     * The first value represents how ambiguous is the x,
     * the second one for the y, and the third one is for the theta (rotation).
     * Increase these numbers to trust the estimate less.
     */
    static final Vector<N3> ODOMETRY_AMBIGUITY = VecBuilder.fill(0.003, 0.003, 0.0002);
    static final double POSE_BUFFER_SIZE_SECONDS = 2.0;

    public record OdometryEstimatedPose(SwerveModulePosition[] wheelPositions, Rotation2d gyroYaw, double timestamp) {
    }

    public record VisionEstimatedPose(Pose2d visionPosition, double timestamp, Matrix<N3, N1> standardDeviations) {
    }
}