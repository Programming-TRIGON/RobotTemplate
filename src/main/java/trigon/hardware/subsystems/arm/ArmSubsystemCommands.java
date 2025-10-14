package trigon.hardware.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import trigon.commands.ExecuteEndCommand;
import trigon.commands.GearRatioCalculationCommand;
import trigon.commands.NetworkTablesCommand;

import java.util.Set;
import java.util.function.DoubleConsumer;

public class ArmSubsystemCommands {
    public Command getDebuggingCommand(ArmSubsystem subsystem) {
        return new NetworkTablesCommand(
                (targetAngleDegrees, speedScalar) -> subsystem.setTargetState(Rotation2d.fromDegrees(targetAngleDegrees), speedScalar),
                false,
                Set.of(subsystem),
                subsystem.getName() + "TargetAngleDegrees",
                subsystem.getName() + "SpeedScalar"
        );
    }

    public Command getGearRatioCalculationCommand(DoubleConsumer runVoltage, double backlashAccountabilityTimeSeconds, ArmSubsystem subsystem) {
        return new GearRatioCalculationCommand(
                subsystem::getRotorPositionRotations,
                subsystem::getRawMotorPositionRotations,
                runVoltage,
                backlashAccountabilityTimeSeconds,
                subsystem
        );
    }

    public Command getSetTargetStateCommand(ArmSubsystem.ArmState targetState, ArmSubsystem subsystem) {
        return new ExecuteEndCommand(
                () -> subsystem.setTargetState(targetState),
                subsystem::stop,
                subsystem
        );
    }

    public Command getSetTargetStateCommand(Rotation2d targetAngle, double speedScalar, ArmSubsystem subsystem) {
        return new ExecuteEndCommand(
                () -> subsystem.setTargetState(targetAngle, speedScalar),
                subsystem::stop,
                subsystem
        );
    }

    public Command getPrepareTargetStateCommand(ArmSubsystem.ArmState targetState, ArmSubsystem subsystem) {
        return new ExecuteEndCommand(
                () -> subsystem.setTargetState(targetState.getPrepareState()),
                subsystem::stop,
                subsystem
        );
    }
}
