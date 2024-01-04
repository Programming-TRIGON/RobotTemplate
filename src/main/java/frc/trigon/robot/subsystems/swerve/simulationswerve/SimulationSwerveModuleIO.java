package frc.trigon.robot.subsystems.swerve.simulationswerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class SimulationSwerveModuleIO extends SwerveModuleIO {
    private final DCMotorSim driveMotor, steerMotor;
    private final PIDController steerPIDController;
    private double driveAppliedVoltage;

    SimulationSwerveModuleIO(SimulationSwerveModuleConstants moduleConstants, String moduleName) {
        super(moduleName);

        driveMotor = moduleConstants.driveMotor;
        steerMotor = moduleConstants.steerMotor;
        steerPIDController = new PIDController(
                SimulationSwerveModuleConstants.STEER_MOTOR_P,
                SimulationSwerveModuleConstants.STEER_MOTOR_I,
                SimulationSwerveModuleConstants.STEER_MOTOR_D
        );
        steerPIDController.enableContinuousInput(-0.5, 0.5);
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        steerMotor.update(RobotConstants.PERIODIC_TIME_SECONDS);
        driveMotor.update(RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.steerAngleDegrees = Conversions.revolutionsToDegrees(steerMotor.getAngularPositionRotations());

        inputs.driveDistanceMeters = Conversions.revolutionsToDistance(driveMotor.getAngularPositionRotations(), SimulationSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveVelocityMetersPerSecond = Conversions.revolutionsToDistance(Units.radiansToRotations(driveMotor.getAngularVelocityRadPerSec()), SimulationSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveCurrent = driveMotor.getCurrentDrawAmps();
    }

    @Override
    protected void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        final double voltage = velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                SimulationSwerveModuleConstants.WHEEL_DIAMETER_METERS,
                driveMotor.getAngularVelocityRadPerSec(),
                SimulationSwerveModuleConstants.DRIVE_GEAR_RATIO,
                SimulationSwerveModuleConstants.MAX_SPEED_REVOLUTIONS_PER_SECOND,
                SimulationSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        setDriveVoltage(voltage);
    }

    @Override
    protected void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        final double pidOutput = steerPIDController.calculate(
                steerMotor.getAngularPositionRotations(),
                angle.getRotations()
        );

        setSteerVoltage(pidOutput);
    }

    @Override
    protected void stop() {
        setDriveVoltage(0);
        setSteerVoltage(0);
    }

    private void setDriveVoltage(double voltage) {
        driveAppliedVoltage = voltageToMaxedVoltage(voltage);
        driveMotor.setInputVoltage(driveAppliedVoltage);
    }

    private void setSteerVoltage(double voltage) {
        steerMotor.setInputVoltage(voltageToMaxedVoltage(voltage));
    }

    private double voltageToMaxedVoltage(double voltage) {
        return MathUtil.clamp(
                voltage,
                -SimulationSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION,
                SimulationSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
    }
}
