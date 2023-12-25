package frc.trigon.robot.subsystems.swerve.trihardswerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;
import frc.trigon.robot.utilities.TalonFXBrakeThread;

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

        inputs.driveDistanceMeters = getDriveDistance(inputs.steerAngleDegrees);
        inputs.driveVelocityMetersPerSecond = Conversions.revolutionsToDistance(moduleConstants.driveVelocitySignal.getValue(), TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveCurrent = moduleConstants.driveStatorCurrentSignal.refresh().getValue();
    }

    @Override
    protected void setTargetOpenLoopVelocity(double velocity) {
        final double optimizedVelocity = optimizeVelocity(velocity);
        final double power = optimizedVelocity / TrihardSwerveConstants.MAX_MODULE_SPEED_METERS_PER_SECOND;
        final double voltage = Conversions.compensatedPowerToVoltage(power, TrihardSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION);

        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }

    @Override
    protected void setTargetClosedLoopVelocity(double velocity) {
        final double optimizedVelocity = optimizeVelocity(velocity);
        driveMotor.setControl(driveVelocityRequest.withVelocity(optimizedVelocity));
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
        TalonFXBrakeThread.setBrakeAsynchronous(driveMotor, brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        TalonFXBrakeThread.setBrakeAsynchronous(steerMotor, brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
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

    private double getDriveDistance(double moduleAngleDegrees) {
        BaseStatusSignal.refreshAll(moduleConstants.drivePositionSignal, moduleConstants.driveVelocitySignal);
        final double latencyCompensatedRevolutions = BaseStatusSignal.getLatencyCompensatedValue(moduleConstants.drivePositionSignal, moduleConstants.driveVelocitySignal);
        final double revolutionsWithoutCoupling = removeCouplingFromRevolutions(latencyCompensatedRevolutions, moduleAngleDegrees);
        return Conversions.revolutionsToDistance(revolutionsWithoutCoupling, TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    /**
     * When the steer motor moves, the drive motor moves as well due to the coupling.
     * This will affect the current position of the drive motor, so we need to remove the coupling from the position.
     *
     * @param drivePosition      the position in revolutions
     * @param moduleAngleDegrees the angle of the module in degrees
     * @return the distance without the coupling
     */
    private double removeCouplingFromRevolutions(double drivePosition, double moduleAngleDegrees) {
        final double coupledAngle = moduleAngleDegrees * TrihardSwerveModuleConstants.COUPLING_RATIO;
        return drivePosition - coupledAngle;
    }
}
