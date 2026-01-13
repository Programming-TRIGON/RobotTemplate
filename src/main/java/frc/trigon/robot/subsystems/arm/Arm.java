package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.subsystems.arm.ArmSubsystem;
import frc.trigon.lib.subsystems.arm.ArmSubsystemConfiguration;

public class Arm extends ArmSubsystem {
    public Arm() {
        super(ArmConstants.MASTER_MOTOR, ArmConstants.ARM_CONFIG, new Pose3d());
    }
}
