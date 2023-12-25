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
    private SwerveModuleInputsAutoLogged lastInputs = new SwerveModuleInputsAutoLogged();

    SimulationSwerveModuleIO(SimulationSwerveModuleConstants moduleConstants, String moduleName) {
        super(moduleName);

        driveMotor = moduleConstants.driveMotor;
        steerMotor = moduleConstants.steerMotor;
        steerPIDController = new PIDController(
                SimulationSwerveModuleConstants.STEER_MOTOR_P,
                SimulationSwerveModuleConstants.STEER_MOTOR_I,
                SimulationSwerveModuleConstants.STEER_MOTOR_D
        );
        steerPIDController.enableContinuousInput(-180, 180);
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        steerMotor.update(RobotConstants.PERIODIC_TIME_SECONDS);
        driveMotor.update(RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.steerAngleDegrees = Conversions.revolutionsToDegrees(steerMotor.getAngularPositionRotations());

        inputs.driveDistanceMeters = Conversions.revolutionsToDistance(driveMotor.getAngularPositionRotations(), SimulationSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveVelocityMetersPerSecond = Conversions.revolutionsToDistance(Units.radiansToRotations(driveMotor.getAngularVelocityRadPerSec()), SimulationSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveCurrent = driveMotor.getCurrentDrawAmps();

        lastInputs = inputs;
    }

    @Override
    protected void setTargetOpenLoopVelocity(double velocity) {
        final double power = velocity / SimulationSwerveConstants.MAX_MODULE_SPEED_METERS_PER_SECOND;
        final double voltage = power * SimulationSwerveModuleConstants.MAX_MOTOR_VOLTAGE;
        setDriveVoltage(voltage);
    }

    @Override
    protected void setTargetClosedLoopVelocity(double velocity) {
        setTargetOpenLoopVelocity(velocity);
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        final double pidOutput = steerPIDController.calculate(
                lastInputs.steerAngleDegrees,
                angle.getDegrees()
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
                -SimulationSwerveModuleConstants.MAX_MOTOR_VOLTAGE,
                SimulationSwerveModuleConstants.MAX_MOTOR_VOLTAGE
        );
    }
}
