package frc.trigon.robot.poseestimation.poseestimator;

public class PoseEstimatorConstants {
    public static final double ODOMETRY_FREQUENCY_HERTZ = 250;

    static final double POSE_BUFFER_SIZE_SECONDS = 2;

    static final StandardDeviations ODOMETRY_STANDARD_DEVIATIONS = new StandardDeviations(0.003, 0.0002);


    /**
     * A record that stores how ambiguous the estimated pose of the robot is.
     * The greater the number, the less trustworthy the pose is.
     *
     * @param translation the ambiguity of the translation aspect of the pose estimation
     * @param theta       the ambiguity of the rotation aspect of the pose estimation
     */
    public record StandardDeviations(double translation, double theta) {
        PoseEstimatorConstants.StandardDeviations combineOdometryAndVisionStandardDeviations() {
            final PoseEstimatorConstants.StandardDeviations odometryStandardDeviations = PoseEstimatorConstants.ODOMETRY_STANDARD_DEVIATIONS;

            return new PoseEstimatorConstants.StandardDeviations(
                    combineOdometryAndVisionStandardDeviation(odometryStandardDeviations.translation(), this.translation()),
                    combineOdometryAndVisionStandardDeviation(odometryStandardDeviations.theta(), this.theta())
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
    }
}

