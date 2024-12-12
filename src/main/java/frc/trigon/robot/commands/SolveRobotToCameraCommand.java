package frc.trigon.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SolveRobotToCameraCommand {
    /**
     * Calculates the transform from the robot to the camera from two estimated poses taken while the robot is rotating in place.
     *
     * @param firstCameraPose  the first estimated pose of the robot
     * @param secondCameraPose the second estimated pose of the robot. This pose should be taken after the robot has rotated without translation.
     * @return the Transform2d of the robot to the camera
     */
    public static Transform2d solveRobotToCamera(Pose2d firstCameraPose, Pose2d secondCameraPose) {
        double firstCameraPoseX = firstCameraPose.getTranslation().getX();
        double firstCameraPoseY = firstCameraPose.getTranslation().getY();
        double firstCameraPoseTheta = firstCameraPose.getRotation().getRadians();
        double firstCameraPoseCosine = Math.cos(firstCameraPoseTheta);
        double firstCameraPoseSine = Math.sin(firstCameraPoseTheta);

        double secondCameraPoseX = secondCameraPose.getTranslation().getX();
        double secondCameraPoseY = secondCameraPose.getTranslation().getY();
        double secondCameraPoseTheta = secondCameraPose.getRotation().getRadians();
        double secondCameraPoseCosine = Math.cos(secondCameraPoseTheta);
        double secondCameraPoseSine = Math.sin(secondCameraPoseTheta);

        double denominator = (firstCameraPoseCosine - secondCameraPoseCosine) *
                (firstCameraPoseCosine - secondCameraPoseCosine) +
                (firstCameraPoseSine - secondCameraPoseSine) *
                        (firstCameraPoseSine - secondCameraPoseSine);

        double xCameraToRobotDistanceMeters = ((secondCameraPoseX - firstCameraPoseX) * (firstCameraPoseCosine - secondCameraPoseCosine) + (secondCameraPoseY - firstCameraPoseY) * (firstCameraPoseSine - secondCameraPoseSine)) / -denominator;
        double yCameraToRobotDistanceMeters = ((secondCameraPoseX - firstCameraPoseX) * (firstCameraPoseSine - secondCameraPoseSine) + (secondCameraPoseY - firstCameraPoseY) * (secondCameraPoseCosine - firstCameraPoseCosine)) / denominator;

        return new Transform2d(new Translation2d(xCameraToRobotDistanceMeters, yCameraToRobotDistanceMeters), new Rotation2d(0));
    }
}
