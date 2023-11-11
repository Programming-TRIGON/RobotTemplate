package frc.trigon.robot.subsystems.swerve.trihardswerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class TrihardSwerveModuleIO extends SwerveModuleIO {
    private final TalonFX steerMotor, driveMotor;
    private final StatusSignal<Double> steerPositionSignal, steerStatorCurrentSignal, steerVelocitySignal, steerClosedLoopErrorSignal, drivePositionSignal, driveVelocitySignal, driveDutyCycleSignal;

    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0, 0, TrihardSwerveModuleConstants.ENABLE_FOC, 0, 0, false);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0, TrihardSwerveModuleConstants.ENABLE_FOC, false);
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0, 0, TrihardSwerveModuleConstants.ENABLE_FOC, 0, 0, false);
    private final ControlRequest
            coastRequest = new CoastOut(),
            brakeRequest = new StaticBrake();

    TrihardSwerveModuleIO(TrihardSwerveModuleConstants moduleConstants, String moduleName) {
        super(moduleName);

        this.steerMotor = moduleConstants.steerMotor;
        this.driveMotor = moduleConstants.driveMotor;

        steerPositionSignal = steerMotor.getPosition();
        steerVelocitySignal = steerMotor.getVelocity();
        steerClosedLoopErrorSignal = steerMotor.getClosedLoopError();
        steerStatorCurrentSignal = steerMotor.getStatorCurrent();

        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();
        driveDutyCycleSignal = driveMotor.getDutyCycle();
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        inputs.steerAngleDegrees = getAngleDegrees();
        inputs.steerCurrent = steerStatorCurrentSignal.refresh().getValue();
        inputs.steerClosedLoopErrorDegrees = Conversions.revolutionsToDegrees(steerClosedLoopErrorSignal.refresh().getValue());

        inputs.driveDistanceMeters = getDriveDistance(inputs.steerAngleDegrees);
        inputs.driveVelocityMetersPerSecond = Conversions.revolutionsToDistance(driveVelocitySignal.getValue(), TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveDutyCycle = driveDutyCycleSignal.refresh().getValue();
    }

    @Override
    protected void setTargetOpenLoopVelocity(double velocity) {
        final double optimizedVelocity = removeCouplingFromTargetVelocity(velocity);
        final double power = optimizedVelocity / TrihardSwerveModuleConstants.MAX_THEORETICAL_SPEED_METERS_PER_SECOND;
        final double voltage = Conversions.compensatedPowerToVoltage(power, TrihardSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION);

        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }

    @Override
    protected void setTargetClosedLoopVelocity(double velocity) {
        final double optimizedVelocity = removeCouplingFromTargetVelocity(velocity);
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
        if (brake) {
            driveMotor.setControl(brakeRequest);
            steerMotor.setControl(brakeRequest);
        } else {
            driveMotor.setControl(coastRequest);
            steerMotor.setControl(coastRequest);
        }
    }

    /**
     * When the steer motor moves, the drive motor moves as well due to the coupling.
     * This will the real output velocity of the drive motor, so we need to remove the coupling from the velocity.
     *
     * @param targetDriveVelocityMetersPerSecond the target velocity of the drive motor in meters per second
     * @return the velocity without the coupling
     */
    private double removeCouplingFromTargetVelocity(double targetDriveVelocityMetersPerSecond) {
        final double velocityRevolutions = Conversions.distanceToRevolutions(targetDriveVelocityMetersPerSecond, TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        final double driveRateBackOut = steerVelocitySignal.getValue() * TrihardSwerveModuleConstants.COUPLING_RATIO;
        return velocityRevolutions - driveRateBackOut;
    }

    private double getAngleDegrees() {
        BaseStatusSignal.refreshAll(steerPositionSignal, steerVelocitySignal);
        final double latencyCompensatedRevolutions = BaseStatusSignal.getLatencyCompensatedValue(steerPositionSignal, steerVelocitySignal);
        return Conversions.revolutionsToDegrees(latencyCompensatedRevolutions);
    }

    private double getDriveDistance(double moduleAngleDegrees) {
        BaseStatusSignal.refreshAll(drivePositionSignal, driveVelocitySignal);
        final double latencyCompensatedRevolutions = BaseStatusSignal.getLatencyCompensatedValue(drivePositionSignal, driveVelocitySignal);
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
