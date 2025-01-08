package frc.trigon.robot.poseestimation.poseestimator;

import edu.wpi.first.math.geometry.Transform2d;

/**
 * Standard Deviations represent how ambiguous an estimated pose is using calibrated gains.
 * The higher the value is, the more ambiguous the pose is and the less trustworthy the result.
 * Standard Deviations are used to reduce noise in a pose estimate result by accounting for how much each result is wrong by.
 */
public class StandardDeviations {
    private final double translationStandardDeviation, thetaStandardDeviation;

    /**
     * Constructs an object that stores how ambiguous the estimated pose of the robot is.
     * The greater the number, the less trustworthy the pose is.
     *
     * @param translationStandardDeviation the ambiguity of the translation aspect of the pose estimation
     * @param thetaStandardDeviation       the ambiguity of the rotation aspect of the pose estimation
     */
    public StandardDeviations(double translationStandardDeviation, double thetaStandardDeviation) {
        this.translationStandardDeviation = translationStandardDeviation;
        this.thetaStandardDeviation = thetaStandardDeviation;
    }

    public double getTranslationStandardDeviation() {
        return translationStandardDeviation;
    }

    public double getThetaStandardDeviation() {
        return thetaStandardDeviation;
    }

    /**
     * Combines this with another {@link StandardDeviations}.
     * This might be used when estimating a pose in relation to another estimated pose.
     * In a case like this, you would want to find the combined standard deviations of both estimated poses to find the final standard deviations.
     *
     * @param other the {@link StandardDeviations} to combine with
     * @return the combined {@link StandardDeviations}
     */
    public StandardDeviations combineWith(StandardDeviations other) {
        return new StandardDeviations(
                combineStandardDeviation(translationStandardDeviation, other.translationStandardDeviation),
                combineStandardDeviation(thetaStandardDeviation, other.thetaStandardDeviation)
        );
    }

    public Transform2d scaleTransformFromStandardDeviations(Transform2d transform) {
        return new Transform2d(
                transform.getX() * translationStandardDeviation,
                transform.getY() * translationStandardDeviation,
                transform.getRotation().times(thetaStandardDeviation)
        );
    }

    /**
     * Combines two standard deviation values.
     *
     * @param firstStandardDeviation  the original standard deviation value
     * @param secondStandardDeviation the standard deviation value to combine with
     * @return the combined standard deviation
     */
    private double combineStandardDeviation(double firstStandardDeviation, double secondStandardDeviation) {
        if (firstStandardDeviation == 0.0)
            return 0.0;

        final double squaredSecondStandardDeviation = secondStandardDeviation * secondStandardDeviation;
        final double combinedSquareRoot = Math.sqrt(firstStandardDeviation * squaredSecondStandardDeviation);
        final double denominator = firstStandardDeviation + combinedSquareRoot;
        return firstStandardDeviation / denominator;
    }
}
