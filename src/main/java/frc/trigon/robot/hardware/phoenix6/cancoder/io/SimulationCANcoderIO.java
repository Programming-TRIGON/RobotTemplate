package frc.trigon.robot.hardware.phoenix6.cancoder.io;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import frc.trigon.robot.hardware.phoenix6.cancoder.CANcoderIO;

import java.util.function.DoubleSupplier;

public class SimulationCANcoderIO extends CANcoderIO {
    private final CANcoder cancoder;
    private final CANcoderSimState simState;
    private final DoubleSupplier positionSupplier, velocitySupplier;

    public SimulationCANcoderIO(int id, DoubleSupplier positionSupplierRotations, DoubleSupplier velocitySupplierRotationsPerSecond) {
        this.cancoder = new CANcoder(id);
        cancoder.setPosition(0);
        this.simState = cancoder.getSimState();
        this.positionSupplier = positionSupplierRotations;
        this.velocitySupplier = velocitySupplierRotationsPerSecond;
    }

    @Override
    public void updateEncoder() {
        simState.setRawPosition(positionSupplier == null ? 0 : positionSupplier.getAsDouble());
        simState.setVelocity(velocitySupplier == null ? 0 : velocitySupplier.getAsDouble());
    }

    @Override
    public void applyConfiguration(CANcoderConfiguration configuration) {
        cancoder.getConfigurator().apply(configuration);
    }

    @Override
    public void optimizeBusUsage() {
        cancoder.optimizeBusUtilization();
    }

    @Override
    public CANcoder getCANcoder() {
        return cancoder;
    }
}
