package frc.trigon.robot.hardware.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.trigon.robot.hardware.Phoenix6Inputs;

import java.util.function.Function;

public enum TalonFXSignal {
    POSITION(TalonFX::getPosition),
    VELOCITY(TalonFX::getVelocity),
    TORQUE_CURRENT(TalonFX::getTorqueCurrent),
    STATOR_CURRENT(TalonFX::getStatorCurrent),
    SUPPLY_CURRENT(TalonFX::getSupplyCurrent),
    CLOSED_LOOP_REFERENCE(TalonFX::getClosedLoopReference),
    MOTOR_VOLTAGE(TalonFX::getMotorVoltage);

    final String name;
    final Function<TalonFX, BaseStatusSignal> signalFunction;

    TalonFXSignal(Function<TalonFX, BaseStatusSignal> signalFunction) {
        this.name = Phoenix6Inputs.enumNameToSignalName(name());
        this.signalFunction = signalFunction;
    }
}