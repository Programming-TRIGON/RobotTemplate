package frc.trigon.robot.subsystems.flyWheel;

import frc.trigon.lib.subsystems.flywheel.SimpleMotorSubsystem;

public class FlyWheel extends SimpleMotorSubsystem {
    public FlyWheel() {
        super(FlyWheelConstants.MASTER_MOTOR, FlyWheelConstants.FLY_WHEEL_CONFIG);
    }
}