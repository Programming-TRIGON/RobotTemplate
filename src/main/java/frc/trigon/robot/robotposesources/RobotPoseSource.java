package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.Robot;
import org.littletonrobotics.junction.Logger;

/**
 * A pose source is a class that provides the robot's pose, from a camera.
 */
public class RobotPoseSource extends SubsystemBase {
    protected final String name;
    private final RobotPoseSourceInputsAutoLogged robotPoseSourceInputs = new RobotPoseSourceInputsAutoLogged();
    private Transform3d cameraToRobotCenter;
    private RobotPoseSourceIO robotPoseSourceIO;
    private double lastUpdatedTimestamp;
    private Pose2d lastRobotPose = new Pose2d();

    public RobotPoseSource(PoseSourceConstants.RobotPoseSourceType robotPoseSourceType, String name, Transform3d cameraToRobotCenter) {
        if (robotPoseSourceType != PoseSourceConstants.RobotPoseSourceType.PHOTON_CAMERA)
            this.cameraToRobotCenter = cameraToRobotCenter;
        this.name = name;

        if (Robot.IS_REAL)
            robotPoseSourceIO = robotPoseSourceType.createIOFunction.apply(name, cameraToRobotCenter);
        else
            robotPoseSourceIO = new RobotPoseSourceIO();
    }

    public static double[] pose3dToDoubleArray(Pose3d pose) {
        if (pose == null)
            return null;

        return new double[]{
                pose.getTranslation().getX(),
                pose.getTranslation().getY(),
                pose.getTranslation().getZ(),
                pose.getRotation().getX(),
                pose.getRotation().getY(),
                pose.getRotation().getZ()
        };
    }

    @Override
    public void periodic() {
        robotPoseSourceIO.updateInputs(robotPoseSourceInputs);
        Logger.processInputs(name, robotPoseSourceInputs);
    }

    public boolean hasNewResult() {
        return isNewTimestamp() && robotPoseSourceInputs.hasResult;
    }

    public Pose2d getRobotPose() {
        final Pose3d cameraPose = doubleArrayToPose3d(robotPoseSourceInputs.cameraPose);
        if (cameraPose == null)
            return lastRobotPose;

        lastRobotPose = cameraPose.transformBy(cameraToRobotCenter).toPose2d();
        return lastRobotPose;
    }

    public String getName() {
        return name;
    }

    public double getLastResultTimestamp() {
        return robotPoseSourceInputs.lastResultTimestamp;
    }

    private boolean isNewTimestamp() {
        if (lastUpdatedTimestamp == getLastResultTimestamp())
            return false;

        lastUpdatedTimestamp = getLastResultTimestamp();
        return true;
    }

    private Pose3d doubleArrayToPose3d(double[] doubleArray) {
        if (doubleArray == null)
            return null;

        return new Pose3d(
                new Translation3d(doubleArray[0], doubleArray[1], doubleArray[2]),
                new Rotation3d(doubleArray[3], doubleArray[4], doubleArray[5])
        );
    }
}
