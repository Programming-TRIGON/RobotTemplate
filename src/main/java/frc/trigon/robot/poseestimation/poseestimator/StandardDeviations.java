package frc.trigon.robot.poseestimation.poseestimator;

import edu.wpi.first.math.geometry.Transform2d;

public class StandardDeviations {
    private final double translation, theta;

    /**
     * Constructs an object that stores how ambiguous the estimated pose of the robot is.
     * The greater the number, the less trustworthy the pose is.
     *
     * @param translation the ambiguity of the translation aspect of the pose estimation
     * @param theta       the ambiguity of the rotation aspect of the pose estimation
     */
    public StandardDeviations(double translation, double theta) {
        this.translation = translation;
        this.theta = theta;
    }

    StandardDeviations combineStandardDeviations(StandardDeviations other) {
        return new StandardDeviations(
                combineStandardDeviation(other.translation, translation),
                combineStandardDeviation(other.theta, theta)
        );
    }

    Transform2d scaleTransformFromStandardDeviations(Transform2d transform) {
        return new Transform2d(
                transform.getX() * translation,
                transform.getY() * translation,
                transform.getRotation().times(theta)
        );
    }

    /**
     * Combines two standard deviation values.
     *
     * @param firstStandardDeviation  the first standard deviation value as a double
     * @param secondStandardDeviation the second deviation value as a double
     * @return the combined standard deviation
     */
    private double combineStandardDeviation(double firstStandardDeviation, double secondStandardDeviation) {
        return firstStandardDeviation / (firstStandardDeviation + Math.sqrt(firstStandardDeviation * secondStandardDeviation));
    }
}
