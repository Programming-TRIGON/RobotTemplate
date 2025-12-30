package frc.trigon.robot.poseestimation.relativerobotposesource;

import edu.wpi.first.math.geometry.Pose2d;
import frc.trigon.robot.poseestimation.relativerobotposesource.io.RelativeRobotPoseSourceSimulationIO;
import frc.trigon.robot.poseestimation.relativerobotposesource.io.RelativeRobotPoseSourceT265IO;
import org.littletonrobotics.junction.AutoLog;
import frc.trigon.lib.hardware.RobotHardwareStats;

public class RelativeRobotPoseSourceIO {
    static RelativeRobotPoseSourceIO generateIO(String hostname) {
        if (RobotHardwareStats.isReplay())
            return new RelativeRobotPoseSourceIO();
        if (RobotHardwareStats.isSimulation())
            return new RelativeRobotPoseSourceSimulationIO();
        return new RelativeRobotPoseSourceT265IO(hostname);
    }

    protected void updateInputs(RelativeRobotPoseSourceInputsAutoLogged inputs) {
    }

    @AutoLog
    public static class RelativeRobotPoseSourceInputs {
        public int framesPerSecond = 0;
        public double batteryPercentage = 0;
        public Pose2d pose = new Pose2d();
        public double resultTimestampSeconds = 0;
        public boolean hasResult = false;
    }
}
