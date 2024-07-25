package frc.trigon.robot.hardware.phoenix6.pigeon2;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.hardware.phoenix6.Phoenix6Inputs;
import frc.trigon.robot.hardware.phoenix6.pigeon2.io.RealPigeon2IO;
import frc.trigon.robot.hardware.phoenix6.pigeon2.io.SimulationPigeon2IO;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Pigeon2Gyro {
    private final String gyroName;
    private final Pigeon2IO gyroIO;
    private final Phoenix6Inputs gyroInputs;
    private final int id;

    public Pigeon2Gyro(int id, String gyroName) {
        this(id, gyroName, null, "");
    }

    public Pigeon2Gyro(int id, String gyroName, String canbus) {
        this(id, gyroName, null, canbus);
    }

    public Pigeon2Gyro(int id, String gyroName, DoubleSupplier yawVelocitySupplierRotationsPerSecond) {
        this(id, gyroName, yawVelocitySupplierRotationsPerSecond, "");
    }

    public Pigeon2Gyro(int id, String gyroName, DoubleSupplier yawVelocitySupplierRotationsPerSecond, String canbus) {
        this.gyroName = gyroName;
        this.gyroIO = generateIO(id, yawVelocitySupplierRotationsPerSecond, canbus);
        this.gyroInputs = new Phoenix6Inputs(gyroName);
        this.id = id;
        gyroIO.optimizeBusUsage();
    }

    public void update() {
        gyroIO.updateGyro();
        Logger.processInputs("Gyros/" + gyroName, gyroInputs);
    }

    public int getID() {
        return id;
    }

    public void applyConfigurations(Pigeon2Configuration realConfiguration, Pigeon2Configuration simulationConfiguration) {
        if (RobotConstants.IS_SIMULATION)
            gyroIO.applyConfiguration(simulationConfiguration);
        else
            gyroIO.applyConfiguration(realConfiguration);
    }

    public void applyConfiguration(Pigeon2Configuration simulationAndRealConfiguration) {
        gyroIO.applyConfiguration(simulationAndRealConfiguration);
    }

    public void applyRealConfiguration(Pigeon2Configuration realConfiguration) {
        if (!RobotConstants.IS_SIMULATION)
            gyroIO.applyConfiguration(realConfiguration);
    }

    public void applySimulationConfiguration(Pigeon2Configuration simulationConfiguration) {
        if (RobotConstants.IS_SIMULATION)
            gyroIO.applyConfiguration(simulationConfiguration);
    }

    public void setYaw(Rotation2d currentYaw) {
        gyroIO.setYaw(currentYaw);
    }

    public double getSignal(Pigeon2Signal signal) {
        return gyroInputs.getSignal(signal.name);
    }

    public double[] getThreadedSignal(Pigeon2Signal signal) {
        return gyroInputs.getThreadedSignal(signal.name);
    }

    public void registerSignal(Pigeon2Signal signal, double updateFrequencyHertz) {
        gyroInputs.registerSignal(pigeon2SignalToStatusSignal(signal), updateFrequencyHertz);
    }

    public void registerThreadedSignal(Pigeon2Signal signal, Pigeon2Signal slopeSignal, double updateFrequencyHertz) {
        gyroInputs.registerThreadedSignal(pigeon2SignalToStatusSignal(signal), pigeon2SignalToStatusSignal(slopeSignal), updateFrequencyHertz);
    }

    private BaseStatusSignal pigeon2SignalToStatusSignal(Pigeon2Signal signal) {
        final Pigeon2 pigeon2 = gyroIO.getPigeon2();
        if (RobotConstants.IS_REPLAY || pigeon2 == null)
            return null;

        return signal.signalFunction.apply(pigeon2);
    }

    private Pigeon2IO generateIO(int id, DoubleSupplier yawVelocitySupplierRotationsPerSecond, String canbus) {
        if (RobotConstants.IS_REPLAY)
            return new Pigeon2IO();
        if (RobotConstants.IS_SIMULATION)
            return new SimulationPigeon2IO(id, yawVelocitySupplierRotationsPerSecond);
        return new RealPigeon2IO(id, canbus);
    }
}
