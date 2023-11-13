package frc.trigon.robot.subsystems.swerve.simulationswerve;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimulationSwerveModuleConstants {
    static final double MAX_MOTOR_VOLTAGE = 12;

    static final double
            DRIVE_GEAR_RATIO = 8.14,
            STEER_GEAR_RATIO = 12.8;
    static final double WHEEL_DIAMETER_METERS = 0.1016;
    static final double MAX_THEORETICAL_SPEED_METERS_PER_SECOND = 4.2;

    private static final double
            DRIVE_MOMENT_OF_INERTIA = 0.003,
            STEER_MOMENT_OF_INERTIA = 0.003;
    static final double
            STEER_MOTOR_P = 0.2,
            STEER_MOTOR_I = 0,
            STEER_MOTOR_D = 0;
    private static final DCMotorSim
            FRONT_LEFT_DRIVE_MOTOR = new DCMotorSim(DCMotor.getFalcon500(1), DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            FRONT_RIGHT_DRIVE_MOTOR = new DCMotorSim(DCMotor.getFalcon500(1), DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            REAR_LEFT_DRIVE_MOTOR = new DCMotorSim(DCMotor.getFalcon500(1), DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            REAR_RIGHT_DRIVE_MOTOR = new DCMotorSim(DCMotor.getFalcon500(1), DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA);
    private static final DCMotorSim
            FRONT_LEFT_STEER_MOTOR = new DCMotorSim(DCMotor.getFalcon500(1), STEER_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
            FRONT_RIGHT_STEER_MOTOR = new DCMotorSim(DCMotor.getFalcon500(1), STEER_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
            REAR_LEFT_STEER_MOTOR = new DCMotorSim(DCMotor.getFalcon500(1), STEER_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
            REAR_RIGHT_STEER_MOTOR = new DCMotorSim(DCMotor.getFalcon500(1), STEER_GEAR_RATIO, STEER_MOMENT_OF_INERTIA);

    static final SimulationSwerveModuleConstants
            FRONT_LEFT_SWERVE_MODULE_CONSTANTS = new SimulationSwerveModuleConstants(
                    FRONT_LEFT_DRIVE_MOTOR,
                    FRONT_LEFT_STEER_MOTOR
            ),
            FRONT_RIGHT_SWERVE_MODULE_CONSTANTS = new SimulationSwerveModuleConstants(
                    FRONT_RIGHT_DRIVE_MOTOR,
                    FRONT_RIGHT_STEER_MOTOR
            ),
            REAR_LEFT_SWERVE_MODULE_CONSTANTS = new SimulationSwerveModuleConstants(
                    REAR_LEFT_DRIVE_MOTOR,
                    REAR_LEFT_STEER_MOTOR
            ),
            REAR_RIGHT_SWERVE_MODULE_CONSTANTS = new SimulationSwerveModuleConstants(
                    REAR_RIGHT_DRIVE_MOTOR,
                    REAR_RIGHT_STEER_MOTOR
            );

    final DCMotorSim driveMotor, steerMotor;

    public SimulationSwerveModuleConstants(DCMotorSim driveMotor, DCMotorSim steerMotor) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
    }
}
