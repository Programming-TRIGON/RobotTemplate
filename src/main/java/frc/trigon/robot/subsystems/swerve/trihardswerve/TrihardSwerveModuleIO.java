package frc.trigon.robot.subsystems.swerve.trihardswerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;


public class TrihardSwerveModuleIO extends SwerveModuleIO {
    private static final SetBrakeNotifier SET_BRAKE_NOTIFIER = new SetBrakeNotifier();
    private final TalonFX steerMotor, driveMotor;
    private final StatusSignal<Double> steerPositionSignal, steerVelocitySignal, driveStatorCurrentSignal, drivePositionSignal, driveVelocitySignal;

    private final VelocityTorqueCurrentFOC driveVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0, TrihardSwerveModuleConstants.ENABLE_FOC, false);
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0, 0, TrihardSwerveModuleConstants.ENABLE_FOC, 0, 0, false);

    TrihardSwerveModuleIO(TrihardSwerveModuleConstants moduleConstants, String moduleName) {
        super(moduleName);

        this.steerMotor = moduleConstants.steerMotor;
        this.driveMotor = moduleConstants.driveMotor;

        steerPositionSignal = moduleConstants.statusSignals[0];
        steerVelocitySignal = moduleConstants.statusSignals[1];
        drivePositionSignal = moduleConstants.statusSignals[2];
        driveVelocitySignal = moduleConstants.statusSignals[3];
        driveStatorCurrentSignal = moduleConstants.statusSignals[4];

        // TODO: check if this is needed.
        makeRequestsSynchronous();
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        inputs.steerAngleDegrees = getAngleDegrees();

        inputs.driveDistanceMeters = getDriveDistance(inputs.steerAngleDegrees);
        inputs.driveVelocityMetersPerSecond = Conversions.revolutionsToDistance(driveVelocitySignal.getValue(), TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveCurrent = driveStatorCurrentSignal.refresh().getValue();
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
        Logger.recordOutput(getLoggingPath() + "targetAngle", angle.getDegrees());
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }

    @Override
    protected void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    @Override
    protected void setBrake(boolean brake) {
        SET_BRAKE_NOTIFIER.setBrake(driveMotor, brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        SET_BRAKE_NOTIFIER.setBrake(steerMotor, brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
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

    private void makeRequestsSynchronous() {
        driveVelocityRequest.UpdateFreqHz = 0;
        driveVoltageRequest.UpdateFreqHz = 0;
        steerPositionRequest.UpdateFreqHz = 0;
    }

    public static class SetBrakeNotifier implements AutoCloseable {
        private final Notifier notifier = new Notifier(this::setBrakeForAllMotors);
        private final HashMap<TalonFX, NeutralModeValue> motorsToSet = new HashMap<>();

        public SetBrakeNotifier() {
            notifier.startPeriodic(0.5);
        }

        @Override
        public void close() {
            notifier.close();
        }

        public void setBrake(TalonFX talonFX, NeutralModeValue neutralModeValue) {
            motorsToSet.put(talonFX, neutralModeValue);
        }

        private void setBrakeForAllMotors() {
            motorsToSet.forEach(this::setBrakeSynchronous);
            motorsToSet.clear();
        }

        private void setBrakeSynchronous(TalonFX motor, NeutralModeValue neutralModeValue) {
            final MotorOutputConfigs config = new MotorOutputConfigs();
            final TalonFXConfigurator configurator = motor.getConfigurator();

            configurator.refresh(config, 10);
            config.NeutralMode = neutralModeValue;
            configurator.apply(config, 10);
        }
    }
}
