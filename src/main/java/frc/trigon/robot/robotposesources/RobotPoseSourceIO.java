package frc.trigon.robot.robotposesources;

import org.littletonrobotics.junction.AutoLog;

public class RobotPoseSourceIO {
    @AutoLog
    public static class RobotPoseSourceInputs {
        public boolean hasResult = false;
        public double lastResultTimestamp = 0;
        public double[] cameraPose = new double[6];
    }

    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
    }
}
