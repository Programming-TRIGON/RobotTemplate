package frc.trigon.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;
import org.trigon.hardware.RobotHardwareStats;

public class AprilTagCameraIO {
    static AprilTagCameraIO generateIO(AprilTagCameraConstants.AprilTagCameraType aprilTagCameraType, String name) {
        if (RobotHardwareStats.isReplay())
            return new AprilTagCameraIO();
        if (RobotHardwareStats.isSimulation())
            return AprilTagCameraConstants.AprilTagCameraType.SIMULATION_CAMERA.createIOFunction.apply(name);
        return aprilTagCameraType.createIOFunction.apply(name);
    }

    protected void updateInputs(AprilTagCameraInputsAutoLogged inputs) {
    }

    /**
     * Adds the simulated camera to the pose estimation simulation.
     *
     * @param robotToCamera the transform of the robot's origin point to the camera
     */
    protected void addSimulatedCameraToVisionSimulation(Transform3d robotToCamera) {
    }

    @AutoLog
    public static class AprilTagCameraInputs {
        public boolean hasResult = false;
        public double latestResultTimestampSeconds = 0;
        public Pose3d cameraSolvePNPPose = new Pose3d();
        public int[] visibleTagIDs = new int[0];
        public double poseAmbiguity = 0;
        public double[] distancesFromTags = new double[0];
    }
}
