package frc.trigon.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;
import frc.trigon.lib.hardware.RobotHardwareStats;

public class AprilTagCameraIO {
    static AprilTagCameraIO generateIO(AprilTagCameraConstants.AprilTagCameraType aprilTagCameraType, String name, Transform3d robotToCamera) {
        if (RobotHardwareStats.isReplay())
            return new AprilTagCameraIO();
        if (RobotHardwareStats.isSimulation())
            aprilTagCameraType = AprilTagCameraConstants.AprilTagCameraType.SIMULATION_CAMERA;

        return aprilTagCameraType.createIOFunction.apply(name, robotToCamera);
    }

    protected void updateInputs(AprilTagCameraInputsAutoLogged inputs) {
    }

    @AutoLog
    public static class AprilTagCameraInputs {
        public boolean hasResult = false;
        public boolean hasConstrainedResult = false;
        public double latestResultTimestampSeconds = 0;
        public Pose3d bestCameraSolvePNPPose = new Pose3d();
        public Pose3d alternateCameraSolvePNPPose = new Pose3d();
        public Pose3d constrainedSolvePNPPose = new Pose3d();
        public int[] visibleTagIDs = new int[0];
        public double poseAmbiguity = 1;
        public double[] distancesFromTags = new double[0];
    }
}