package frc.trigon.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.robot.hardware.phoenix6.cancoder.CANcoderSignal;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimatorConstants;
import frc.trigon.robot.subsystems.swerve.swervemoduleconstants.RealSwerveModuleConstants;
import frc.trigon.robot.subsystems.swerve.swervemoduleconstants.SimulationSwerveModuleConstants;
import frc.trigon.robot.subsystems.swerve.swervemoduleconstants.SwerveModuleConstants;
import frc.trigon.robot.utilities.Conversions;

public class SwerveModule {
    private final TalonFXMotor driveMotor, steerMotor;
    private final CANcoderEncoder steerEncoder;
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0);
    private final VelocityTorqueCurrentFOC driveVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0);
    private boolean driveMotorClosedLoop = false;
    private double[]
            latestOdometryDrivePositions = new double[0],
            latestOdometrySteerPositions = new double[0];
    private SwerveModuleState targetState = new SwerveModuleState();

    public SwerveModule(int moduleID, double offsetRotations) {
        driveMotor = new TalonFXMotor(moduleID, "Module" + moduleID + "Drive", SimulationSwerveModuleConstants.DRIVE_PROPERTIES, RobotConstants.CANIVORE_NAME);
        steerMotor = new TalonFXMotor(moduleID + 4, "Module" + moduleID + "Steer", SimulationSwerveModuleConstants.STEER_PROPERTIES, RobotConstants.CANIVORE_NAME);
        steerEncoder = new CANcoderEncoder(moduleID, "Module" + moduleID + "SteerEncoder", steerMotor, RobotConstants.CANIVORE_NAME);
        configureHardware(offsetRotations);
    }

    void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    void setBrake(boolean brake) {
        driveMotor.setBrake(brake);
        steerMotor.setBrake(brake);
    }

    void update() {
        driveMotor.update();
        steerMotor.update();
        steerEncoder.update();
        latestOdometryDrivePositions = driveMotor.getThreadedSignal(TalonFXSignal.POSITION);
        latestOdometrySteerPositions = steerMotor.getThreadedSignal(TalonFXSignal.POSITION);
    }

    void setDriveMotorClosedLoop(boolean closedLoop) {
        driveMotorClosedLoop = closedLoop;
    }

    void setTargetState(SwerveModuleState targetState) {
        this.targetState = SwerveModuleState.optimize(targetState, getCurrentAngle());
        setTargetAngle(this.targetState.angle);
        setTargetVelocity(this.targetState.speedMetersPerSecond, this.targetState.angle);
    }

    /**
     * The odometry thread can update itself faster than the main code loop (which is 50 hertz).
     * Instead of using the latest odometry update, the accumulated odometry positions since the last loop to get a more accurate position.
     *
     * @param odometryUpdateIndex the index of the odometry update
     * @return the position of the module at the given odometry update index
     */
    SwerveModulePosition getOdometryPosition(int odometryUpdateIndex) {
        return new SwerveModulePosition(
                driveRotationsToMeters(latestOdometryDrivePositions[odometryUpdateIndex]),
                Rotation2d.fromRotations(latestOdometrySteerPositions[odometryUpdateIndex])
        );
    }

    int getLastOdometryUpdateIndex() {
        return driveMotor.getThreadedSignal(TalonFXSignal.POSITION).length - 1;
    }

    SwerveModuleState getCurrentState() {
        return new SwerveModuleState(driveRotationsToMeters(driveMotor.getSignal(TalonFXSignal.VELOCITY)), getCurrentAngle());
    }

    SwerveModuleState getTargetState() {
        return targetState;
    }

    private double driveRotationsToMeters(double rotations) {
        return Conversions.revolutionsToDistance(rotations, SwerveModuleConstants.SYSTEM_SPECIFIC_CONSTANTS.getWheelDiameterMeters());
    }

    private void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }

    /**
     * Sets the target velocity for the module.
     *
     * @param targetVelocityMetersPerSecond the target velocity, in meters per second
     * @param targetSteerAngle              the target steer angle, to calculate for skew reduction
     */
    private void setTargetVelocity(double targetVelocityMetersPerSecond, Rotation2d targetSteerAngle) {
        targetVelocityMetersPerSecond = reduceSkew(targetVelocityMetersPerSecond, targetSteerAngle);
        final double targetVelocityRotationsPerSecond = Conversions.distanceToRevolutions(targetVelocityMetersPerSecond, SwerveModuleConstants.SYSTEM_SPECIFIC_CONSTANTS.getWheelDiameterMeters());

        if (driveMotorClosedLoop) {
            driveMotor.setControl(driveVelocityRequest.withVelocity(targetVelocityRotationsPerSecond));
        } else {
            final double power = targetVelocityRotationsPerSecond / SwerveModuleConstants.MAX_SPEED_REVOLUTIONS_PER_SECOND;
            final double voltage = Conversions.compensatedPowerToVoltage(power, SwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION);
            driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
        }
    }

    /**
     * When changing direction, the module will skew since the angle motor is not at its target angle.
     * This method will counter that by reducing the target velocity according to the angle motor's error cosine.
     *
     * @param targetVelocityMetersPerSecond the target velocity, in meters per second
     * @param targetSteerAngle              the target steer angle
     * @return the reduced target velocity in revolutions per second
     */
    private double reduceSkew(double targetVelocityMetersPerSecond, Rotation2d targetSteerAngle) {
        final double closedLoopError = targetSteerAngle.getRadians() - getCurrentAngle().getRadians();
        final double cosineScalar = Math.abs(Math.cos(closedLoopError));
        return targetVelocityMetersPerSecond * cosineScalar;
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(steerMotor.getSignal(TalonFXSignal.POSITION));
    }

    private void configureHardware(double offsetRotations) {
        driveMotor.applyConfigurations(RealSwerveModuleConstants.DRIVE_CONFIGURATION, SimulationSwerveModuleConstants.DRIVE_CONFIGURATION);
        steerMotor.applyConfigurations(RealSwerveModuleConstants.STEER_CONFIGURATION, SimulationSwerveModuleConstants.STEER_CONFIGURATION);
        RealSwerveModuleConstants.STEER_ENCODER_CONFIGURATION.MagnetSensor.MagnetOffset = offsetRotations;
        steerEncoder.applyConfigurations(RealSwerveModuleConstants.STEER_ENCODER_CONFIGURATION, SimulationSwerveModuleConstants.STEER_ENCODER_CONFIGURATION);
        configureSignals();
    }

    private void configureSignals() {
        driveMotor.registerThreadedSignal(TalonFXSignal.POSITION, TalonFXSignal.VELOCITY, PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
        driveMotor.registerSignal(TalonFXSignal.TORQUE_CURRENT, 100);
        driveMotor.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        steerMotor.registerThreadedSignal(TalonFXSignal.POSITION, TalonFXSignal.VELOCITY, PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
        steerMotor.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        steerEncoder.registerSignal(CANcoderSignal.POSITION, 100);
        steerEncoder.registerSignal(CANcoderSignal.VELOCITY, 100);
    }
}