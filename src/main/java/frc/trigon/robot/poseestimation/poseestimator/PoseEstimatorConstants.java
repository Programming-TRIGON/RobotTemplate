package frc.trigon.robot.poseestimation.poseestimator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class PoseEstimatorConstants {
    public static final double ODOMETRY_FREQUENCY_HERTZ = 250;

    static final double POSE_BUFFER_SIZE_SECONDS = 2;

    /**
     * The vector represents how ambiguous each value of the odometry is.
     * The first value represents how ambiguous the x is,
     * the second one is for the y, and the third one is for the theta (rotation).
     * The greater these numbers are, the less we trust the estimation.
     */
    static final Vector<N3> ODOMETRY_AMBIGUITY = VecBuilder.fill(0.003, 0.003, 0.0002);
    static final double[] AMBIGUITY = {0.003, 0.003, 0.0002};
}

