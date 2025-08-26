package trigon.hardware.misc.simplesensor;

import org.littletonrobotics.junction.AutoLog;
import trigon.hardware.misc.simplesensor.SimpleSensorInputsAutoLogged;

import java.util.function.DoubleSupplier;

public class SimpleSensorIO {
    protected SimpleSensorIO() {
    }

    protected void updateInputs(SimpleSensorInputsAutoLogged inputs) {
    }

    protected void setSimulationSupplier(DoubleSupplier simulationValueSupplier) {
    }

    @AutoLog
    protected static class SimpleSensorInputs {
        public double value;
    }
}
