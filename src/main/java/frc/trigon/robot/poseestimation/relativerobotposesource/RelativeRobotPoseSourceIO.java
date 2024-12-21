package frc.trigon.robot.poseestimation.relativerobotposesource;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public class RelativeRobotPoseSourceIO {
    protected void updateInputs(RelativeRobotPoseSourceInputsAutoLogged inputs) {
    }

    @AutoLog
    public static class RelativeRobotPoseSourceInputs {
        public int framesPerSecond = 0;
        public double batteryPercentage = 0;
        public Pose2d pose = new Pose2d();
        public boolean hasNewResult = false;
        public double latestResultTimestampSeconds = 0;
    }
}
