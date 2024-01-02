package frc.trigon.robot.subsystems.swerve.trihardswerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class TrihardSwerveModuleIO extends SwerveModuleIO {
    private final TalonFX steerMotor, driveMotor;
    private final TrihardSwerveModuleConstants moduleConstants;

    private final VelocityTorqueCurrentFOC driveVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0).withEnableFOC(TrihardSwerveModuleConstants.ENABLE_FOC);
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0).withEnableFOC(TrihardSwerveModuleConstants.ENABLE_FOC);

    TrihardSwerveModuleIO(TrihardSwerveModuleConstants moduleConstants, String moduleName) {
        super(moduleName);

        this.steerMotor = moduleConstants.steerMotor;
        this.driveMotor = moduleConstants.driveMotor;
        this.moduleConstants = moduleConstants;
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        inputs.steerAngleDegrees = getAngleDegrees();

        inputs.driveDistanceMeters = getDriveDistance(Rotation2d.fromDegrees(inputs.steerAngleDegrees));
        inputs.driveVelocityMetersPerSecond = Conversions.revolutionsToDistance(moduleConstants.driveVelocitySignal.getValue(), TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveCurrent = moduleConstants.driveStatorCurrentSignal.refresh().getValue();
    }

    @Override
    protected void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        final double optimizedVelocityRevolutionsPerSecond = optimizeVelocity(targetVelocityMetersPerSecond);
        final double optimizedVelocityMetersPerSecond = Conversions.revolutionsToDistance(optimizedVelocityRevolutionsPerSecond, TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        final double power = optimizedVelocityMetersPerSecond / TrihardSwerveConstants.MAX_SPEED_METERS_PER_SECOND;
        final double voltage = Conversions.compensatedPowerToVoltage(power, TrihardSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION);

        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }

    @Override
    protected void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        final double optimizedVelocityRevolutionsPerSecond = optimizeVelocity(targetVelocityMetersPerSecond);
        driveMotor.setControl(driveVelocityRequest.withVelocity(optimizedVelocityRevolutionsPerSecond));
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }

    @Override
    protected void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    @Override
    protected void setBrake(boolean brake) {
        final NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        setBrake(driveMotor, neutralModeValue);
        setBrake(steerMotor, neutralModeValue);
    }

    /**
     * When the steer motor moves, the drive motor moves as well due to the coupling.
     * This will the real output velocity of the drive motor, so we need to remove the coupling from the velocity.
     * Note that this function accepts meters per second and returns revolutions per second
     *
     * @param targetDriveVelocityMetersPerSecond the target velocity of the drive motor in meters per second
     * @return the velocity without the coupling in revolutions
     */
    private double optimizeVelocity(double targetDriveVelocityMetersPerSecond) {
        final double velocityRevolutions = Conversions.distanceToRevolutions(targetDriveVelocityMetersPerSecond, TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        final double driveRateBackOut = moduleConstants.steerVelocitySignal.getValue() * TrihardSwerveModuleConstants.COUPLING_RATIO;
        return velocityRevolutions - driveRateBackOut;
    }

    private double getAngleDegrees() {
        BaseStatusSignal.refreshAll(moduleConstants.steerPositionSignal, moduleConstants.steerVelocitySignal);
        final double latencyCompensatedRevolutions = BaseStatusSignal.getLatencyCompensatedValue(moduleConstants.steerPositionSignal, moduleConstants.steerVelocitySignal);
        return Conversions.revolutionsToDegrees(latencyCompensatedRevolutions);
    }

    private double getDriveDistance(Rotation2d moduleAngle) {
        BaseStatusSignal.refreshAll(moduleConstants.drivePositionSignal, moduleConstants.driveVelocitySignal);
        final double latencyCompensatedRevolutions = BaseStatusSignal.getLatencyCompensatedValue(moduleConstants.drivePositionSignal, moduleConstants.driveVelocitySignal);
        final double revolutionsWithoutCoupling = removeCouplingFromRevolutions(latencyCompensatedRevolutions, moduleAngle);
        return Conversions.revolutionsToDistance(revolutionsWithoutCoupling, TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    /**
     * When the steer motor moves, the drive motor moves as well due to the coupling.
     * This will affect the current position of the drive motor, so we need to remove the coupling from the position.
     *
     * @param drivePosition the position in revolutions
     * @param moduleAngle   the angle of the module
     * @return the distance without the coupling
     */
    private double removeCouplingFromRevolutions(double drivePosition, Rotation2d moduleAngle) {
        final double coupledAngle = moduleAngle.getRotations() * TrihardSwerveModuleConstants.COUPLING_RATIO;
        return drivePosition - coupledAngle;
    }

    private void setBrake(TalonFX motor, NeutralModeValue neutralModeValue) {
        final MotorOutputConfigs config = new MotorOutputConfigs();
        final TalonFXConfigurator configurator = motor.getConfigurator();

        configurator.refresh(config, 10);
        config.NeutralMode = neutralModeValue;
        configurator.apply(config, 10);
    }
}
