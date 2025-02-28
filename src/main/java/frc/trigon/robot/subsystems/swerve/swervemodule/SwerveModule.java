package frc.trigon.robot.subsystems.swerve.swervemodule;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;

public class SwerveModule {
    private final TalonFXMotor
            driveMotor,
            steerMotor;
    private final CANcoderEncoder steerEncoder;
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0).withEnableFOC(SwerveModuleConstants.ENABLE_FOC);
    private final VelocityTorqueCurrentFOC driveVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0);
    private final TorqueCurrentFOC driveTorqueCurrentFOCRequest = new TorqueCurrentFOC(0);
    private boolean shouldDriveMotorUseClosedLoop = false;
    private SwerveModuleState targetState = new SwerveModuleState();
    private double[]
            latestOdometryDrivePositions,
            latestOdometrySteerPositions;

    /**
     * Constructs a new SwerveModule with the given module ID, wheel diameter, and offset rotations.
     *
     * @param moduleID        the ID of the module
     * @param offsetRotations the module's encoder offset in rotations
     */
    public SwerveModule(int moduleID, double offsetRotations) {
        driveMotor = new TalonFXMotor(moduleID, "Module" + moduleID + "Drive", RobotConstants.CANIVORE_NAME);
        steerMotor = new TalonFXMotor(moduleID + 4, "Module" + moduleID + "Steer", RobotConstants.CANIVORE_NAME);
        steerEncoder = new CANcoderEncoder(moduleID + 4, "Module" + moduleID + "SteerEncoder", RobotConstants.CANIVORE_NAME);

        configureHardware(offsetRotations);
    }

    public void setTargetState(SwerveModuleState targetState) {
        targetState.optimize(getCurrentAngle());
        this.targetState = targetState;
        setTargetAngle(targetState.angle);
        setTargetVelocity(targetState.speedMetersPerSecond);
    }

    public void setBrake(boolean brake) {
        driveMotor.setBrake(brake);
        steerMotor.setBrake(brake);
    }

    public void updateSysIDLog(SysIdRoutineLog log) {
        log.motor("Module" + driveMotor.getID() + "Drive")
                .angularPosition(Units.Rotations.of(driveMotor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(driveMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(driveMotor.getSignal(TalonFXSignal.TORQUE_CURRENT)));
    }

    /**
     * Updates the swerve module. Should be called periodically.
     * This method updates the hardware, and their position variables.
     * We save their positions to a variable instead of getting them directly because their signals update at a higher frequency than the main code loop.
     */
    public void updatePeriodically() {
        driveMotor.update();
        steerMotor.update();
        steerEncoder.update();

        latestOdometryDrivePositions = driveMotor.getThreadedSignal(TalonFXSignal.POSITION);
        latestOdometrySteerPositions = steerMotor.getThreadedSignal(TalonFXSignal.POSITION);
    }

    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    public void shouldDriveMotorUseClosedLoop(boolean shouldDriveMotorUseClosedLoop) {
        this.shouldDriveMotorUseClosedLoop = shouldDriveMotorUseClosedLoop;
    }

    public void setDriveMotorTargetCurrent(double targetCurrent) {
        driveMotor.setControl(driveTorqueCurrentFOCRequest.withOutput(targetCurrent));
    }

    public void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(driveWheelRotationsToMeters(driveMotor.getSignal(TalonFXSignal.VELOCITY)), getCurrentAngle());
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    /**
     * Gets the position of the drive wheel in meters. We don't use a {@link Rotation2d} because this function returns distance, not rotations.
     *
     * @return the position of the drive wheel in meters
     */
    public double getDriveWheelPositionRadians() {
        return edu.wpi.first.math.util.Units.rotationsToRadians(driveMotor.getSignal(TalonFXSignal.POSITION));
    }

    /**
     * The odometry thread can update itself faster than the main code loop (which is 50 hertz).
     * Instead of using the latest odometry update, we use the accumulated odometry positions since the last loop to get a more accurate position.
     *
     * @param odometryUpdateIndex the index of the odometry update
     * @return the position of the module at the given odometry update index
     */
    public SwerveModulePosition getOdometryPosition(int odometryUpdateIndex) {
        return new SwerveModulePosition(
                driveWheelRotationsToMeters(latestOdometryDrivePositions[odometryUpdateIndex]),
                Rotation2d.fromRotations(latestOdometrySteerPositions[odometryUpdateIndex])
        );
    }

    /**
     * Sets the target velocity for the module.
     * The target velocity is set using either closed loop or open loop depending on {@link this#shouldDriveMotorUseClosedLoop}.
     *
     * @param targetVelocityMetersPerSecond the target velocity, in meters per second
     */
    private void setTargetVelocity(double targetVelocityMetersPerSecond) {
        if (shouldDriveMotorUseClosedLoop) {
            setTargetClosedLoopVelocity(targetVelocityMetersPerSecond);
            return;
        }

        setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
    }

    private void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        final double targetVelocityRotationsPerSecond = metersToDriveWheelRotations(targetVelocityMetersPerSecond);
        driveMotor.setControl(driveVelocityRequest.withVelocity(targetVelocityRotationsPerSecond));
    }

    private void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        final double power = targetVelocityMetersPerSecond / SwerveConstants.MAXIMUM_SPEED_METERS_PER_SECOND;
        final double voltage = Conversions.compensatedPowerToVoltage(power, SwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION);
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(steerMotor.getSignal(TalonFXSignal.POSITION));
    }

    /**
     * Converts the drive wheel rotations to meters.
     *
     * @param rotations the rotations of the drive wheel
     * @return the distance the drive wheel has traveled in meters
     */
    private double driveWheelRotationsToMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, SwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    /**
     * Converts meters to the rotations of the drive wheel.
     *
     * @param meters the meters to be converted
     * @return the distance the drive wheel has traveled in drive wheel rotations
     */
    private double metersToDriveWheelRotations(double meters) {
        return Conversions.distanceToRotations(meters, SwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    private void configureHardware(double offsetRotations) {
        configureDriveMotor();
        configureSteerMotor();
        configureSteerEncoder(offsetRotations);
    }

    private void configureDriveMotor() {
        driveMotor.applyConfiguration(SwerveModuleConstants.DRIVE_MOTOR_CONFIGURATION);
        driveMotor.setPhysicsSimulation(SwerveModuleConstants.createDriveMotorSimulation());

        driveMotor.registerSignal(TalonFXSignal.VELOCITY, 100);
        driveMotor.registerSignal(TalonFXSignal.TORQUE_CURRENT, 100);
        driveMotor.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        driveMotor.registerThreadedSignal(TalonFXSignal.POSITION, PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
    }

    private void configureSteerMotor() {
        SwerveModuleConstants.STEER_MOTOR_CONFIGURATION.Feedback.FeedbackRemoteSensorID = steerEncoder.getID();
        steerMotor.applyConfiguration(SwerveModuleConstants.STEER_MOTOR_CONFIGURATION);
        steerMotor.setPhysicsSimulation(SwerveModuleConstants.createSteerMotorSimulation());

        steerMotor.registerSignal(TalonFXSignal.VELOCITY, 100);
        steerMotor.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        steerMotor.registerThreadedSignal(TalonFXSignal.POSITION, PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
    }

    private void configureSteerEncoder(double offsetRotations) {
        SwerveModuleConstants.STEER_ENCODER_CONFIGURATION.MagnetSensor.MagnetOffset = offsetRotations;
        steerEncoder.applyConfiguration(SwerveModuleConstants.STEER_ENCODER_CONFIGURATION);
        steerEncoder.setSimulationInputsFromTalonFX(steerMotor);

        steerEncoder.registerSignal(CANcoderSignal.POSITION, 100);
        steerEncoder.registerSignal(CANcoderSignal.VELOCITY, 100);
    }
}