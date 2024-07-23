package frc.trigon.robot.hardware.simulation;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

/**
 * A wrapper class for the WPILib default simulation classes, that'll act similarly to how the TalonFX motor controller works.
 */
public abstract class MotorSimulation {
    private final TalonFX motor;
    private final TalonFXSimState motorSimState;

    protected MotorSimulation(int id) {
        motor = new TalonFX(id);
        motorSimState = motor.getSimState();
        motorSimState.setSupplyVoltage(12);
    }

    public TalonFX getTalonFX() {
        return motor;
    }

    public void applyConfiguration(TalonFXConfiguration config) {
        motor.getConfigurator().apply(config);
    }

    public void stop() {
        motor.stopMotor();
    }

    public void setControl(ControlRequest request) {
        motor.setControl(request);
    }

    public StatusSignal<Double> getVoltage() {
        return motor.getMotorVoltage();
    }

    public StatusSignal<Double> getClosedLoopReference() {
        return motor.getClosedLoopReference();
    }

    public StatusSignal<Double> getPosition() {
        return motor.getPosition();
    }

    public StatusSignal<Double> getVelocity() {
        return motor.getVelocity();
    }

    public void updateSimulation() {
        setInputVoltage(motorSimState.getMotorVoltage());
        updateMotor();
        motorSimState.setRawRotorPosition(getPositionRevolutions());
        motorSimState.setRotorVelocity(getVelocityRevolutionsPerSecond());
    }

    public abstract double getCurrent();

    abstract double getPositionRevolutions();

    abstract double getVelocityRevolutionsPerSecond();

    abstract void setInputVoltage(double voltage);

    abstract void updateMotor();
}
