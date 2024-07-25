package frc.trigon.robot.hardware.phoenix6.pigeon2.io;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.hardware.phoenix6.pigeon2.Pigeon2IO;
import frc.trigon.robot.hardware.simulation.GyroSimulation;

import java.util.function.Supplier;

public class SimulationPigeon2IO extends Pigeon2IO {
    private final Pigeon2 pigeon2;
    private final Pigeon2SimState simState;
    private final Supplier<Double> yawVelocitySupplier;
    private final GyroSimulation gyroSimulation;

    public SimulationPigeon2IO(int id, Supplier<Double> yawVelocitySupplierRotationsPerSecond) {
        this.pigeon2 = new Pigeon2(id);
        this.simState = pigeon2.getSimState();
        this.yawVelocitySupplier = yawVelocitySupplierRotationsPerSecond;
        this.gyroSimulation = new GyroSimulation();
    }

    @Override
    public void updateGyro() {
        if (yawVelocitySupplier == null)
            return;
        gyroSimulation.update(yawVelocitySupplier.get(), RobotConstants.PERIODIC_TIME_SECONDS);
        simState.setRawYaw(gyroSimulation.getGyroYawDegrees());
    }

    @Override
    public void setYaw(Rotation2d currentYaw) {
        gyroSimulation.setYaw(currentYaw);
    }

    @Override
    public void applyConfiguration(Pigeon2Configuration configuration) {
        pigeon2.getConfigurator().apply(configuration);
    }

    @Override
    public void optimizeBusUsage() {
        pigeon2.optimizeBusUtilization();
    }

    @Override
    public Pigeon2 getPigeon2() {
        return pigeon2;
    }
}
