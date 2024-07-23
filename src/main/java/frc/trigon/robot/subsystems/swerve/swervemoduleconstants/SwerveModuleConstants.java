package frc.trigon.robot.subsystems.swerve.swervemoduleconstants;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.swerve.swerveconstants.SwerveConstants;
import frc.trigon.robot.utilities.Conversions;

public abstract class SwerveModuleConstants {
    public static final SwerveModuleConstants SYSTEM_SPECIFIC_CONSTANTS = generateConstants();
    static final double
            STEER_GEAR_RATIO = 12.8,
            DRIVE_GEAR_RATIO = 6.12;
    public static final double MAX_SPEED_REVOLUTIONS_PER_SECOND = Conversions.distanceToRevolutions(SwerveConstants.SYSTEM_SPECIFIC_CONSTANTS.getMaxSpeedMetersPerSecond(), SYSTEM_SPECIFIC_CONSTANTS.getWheelDiameterMeters());
    public static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    public abstract double getWheelDiameterMeters();

    private static SwerveModuleConstants generateConstants() {
        if (RobotConstants.IS_SIMULATION)
            return new SimulationSwerveModuleConstants();
        return new RealSwerveModuleConstants();
    }
}
