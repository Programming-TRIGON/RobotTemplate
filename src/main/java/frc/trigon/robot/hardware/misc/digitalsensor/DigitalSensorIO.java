package frc.trigon.robot.hardware.misc.digitalsensor;

import org.littletonrobotics.junction.AutoLog;

public class DigitalSensorIO {
    protected DigitalSensorIO() {
    }

    protected void updateInputs(DigitalSensorInputsAutoLogged inputs) {
    }

    @AutoLog
    protected static class DigitalSensorInputs {
        public boolean isTriggered;
    }
}
