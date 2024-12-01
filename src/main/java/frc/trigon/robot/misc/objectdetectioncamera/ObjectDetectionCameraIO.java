package frc.trigon.robot.misc.objectdetectioncamera;

import org.littletonrobotics.junction.AutoLog;

public class ObjectDetectionCameraIO {
    protected ObjectDetectionCameraIO() {
    }

    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
    }

    @AutoLog
    public static class ObjectDetectionCameraInputs {
        public boolean hasTargets = false;
        /**
         * An array that contains the yaw of all visible targets. The best target is first.
         */
        public double[] visibleObjectsYaw = new double[0];
    }
}
