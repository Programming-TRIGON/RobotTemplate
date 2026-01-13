package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.subsystems.elevator.ElevatorSubsystem;
import frc.trigon.lib.subsystems.elevator.ElevatorSubsystemConfiguration;

public class Elevator extends ElevatorSubsystem {
    public Elevator() {
        super(ElevatorConstants.MASTER_MOTOR, ElevatorConstants.ELEVATOR_CONFIG, new Pose3d());
    }
}
