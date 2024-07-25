package frc.trigon.robot.hardware.misc.analogsensor;

import org.littletonrobotics.junction.AutoLog;

public class AnalogSensorIO {
    protected AnalogSensorIO() {
    }

    protected void updateInputs(AnalogSensorInputsAutoLogged inputs) {
    }

    @AutoLog
    protected static class AnalogSensorInputs {
        public double value;
    }
}
