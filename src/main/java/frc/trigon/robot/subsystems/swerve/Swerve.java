package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.PathPlannerConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.swerve.swervemodule.SwerveModule;
import frc.trigon.robot.subsystems.swerve.swervemodule.SwerveModuleConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.Phoenix6SignalThread;
import org.trigon.hardware.phoenix6.pigeon2.Pigeon2Gyro;
import org.trigon.hardware.phoenix6.pigeon2.Pigeon2Signal;
import org.trigon.utilities.flippable.Flippable;
import org.trigon.utilities.flippable.FlippablePose2d;
import org.trigon.utilities.flippable.FlippableRotation2d;

public class Swerve extends MotorSubsystem {
    private final Pigeon2Gyro gyro = SwerveConstants.GYRO;
    private final SwerveModule[] swerveModules = SwerveConstants.SWERVE_MODULES;
    private final Phoenix6SignalThread phoenix6SignalThread = Phoenix6SignalThread.getInstance();
    private final SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(PathPlannerConstants.ROBOT_CONFIG, SwerveModuleConstants.MAXIMUM_MODULE_ROTATIONAL_SPEED_RADIANS_PER_SECOND);
    private SwerveSetpoint previousSetpoint;

    public Swerve() {
        setName("Swerve");
        phoenix6SignalThread.setThreadFrequencyHertz(PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
    }

    @Override
    public void setBrake(boolean brake) {
        for (SwerveModule module : swerveModules)
            module.setBrake(brake);
    }

    @Override
    public void drive(double targetCurrent) {
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.setDriveMotorTargetCurrent(targetCurrent);
            swerveModule.setTargetAngle(new Rotation2d());
        }
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        for (SwerveModule module : swerveModules)
            module.updateSysIDLog(log);
    }

    @Override
    public void updatePeriodically() {
        Phoenix6SignalThread.SIGNALS_LOCK.lock();
        updateHardware();
        Phoenix6SignalThread.SIGNALS_LOCK.unlock();

        updatePoseEstimatorStates();
        RobotContainer.POSE_ESTIMATOR.periodic();
        updateNetworkTables();
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return SwerveModuleConstants.DRIVE_MOTOR_SYSID_CONFIG;
    }

    @Override
    public void stop() {
        for (SwerveModule module : swerveModules)
            module.stop();
    }

    public void setHeading(Rotation2d heading) {
        gyro.setYaw(heading);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(SwerveConstants.GYRO.getSignal(Pigeon2Signal.YAW));
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        final ChassisSpeeds selfRelativeSpeeds = getSelfRelativeVelocity();
        return ChassisSpeeds.fromRobotRelativeSpeeds(selfRelativeSpeeds, RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose().getRotation());
    }

    public ChassisSpeeds getSelfRelativeVelocity() {
        return SwerveConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    /**
     * Checks if the robot is at a pose.
     *
     * @param pose2d a pose, relative to the blue alliance driver station's right corner
     * @return whether the robot is at the pose
     */
    public boolean atPose(FlippablePose2d pose2d) {
        final Pose2d flippedPose = pose2d.get();
        return atXAxisPosition(flippedPose.getX()) && atYAxisPosition(flippedPose.getY()) && atAngle(pose2d.getRotation());
    }

    public boolean atXAxisPosition(double xAxisPosition) {
        final double currentXAxisVelocity = getFieldRelativeVelocity().vxMetersPerSecond;
        return atTranslationPosition(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose().getX(), xAxisPosition, currentXAxisVelocity);
    }

    public boolean atYAxisPosition(double yAxisPosition) {
        final double currentYAxisVelocity = getFieldRelativeVelocity().vyMetersPerSecond;
        return atTranslationPosition(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose().getY(), yAxisPosition, currentYAxisVelocity);
    }

    public boolean atAngle(FlippableRotation2d angle) {
        final boolean atTargetAngle = Math.abs(angle.get().minus(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose().getRotation()).getDegrees()) < SwerveConstants.ROTATION_TOLERANCE_DEGREES;
        final boolean isAngleStill = Math.abs(getSelfRelativeVelocity().omegaRadiansPerSecond) < SwerveConstants.ROTATION_VELOCITY_TOLERANCE;
        Logger.recordOutput("Swerve/AtTargetAngle/isStill", isAngleStill);
        Logger.recordOutput("Swerve/AtTargetAngle/atTargetAngle", atTargetAngle);
        return atTargetAngle;
    }

    /**
     * Gets the positions of the drive wheels in radians. We don't use a {@link Rotation2d} because this method returns distance, not rotations.
     *
     * @return the positions of the drive wheels in radians
     */
    public double[] getDriveWheelPositionsRadians() {
        final double[] swerveModulesPositions = new double[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++)
            swerveModulesPositions[i] = swerveModules[i].getDriveWheelPositionRadians();
        return swerveModulesPositions;
    }

    /**
     * Drives the swerve to a certain angular velocity. Used for Wheel Radius Characterization.
     *
     * @param omegaRadiansPerSecond the target angular velocity in radians per second
     */
    public void runWheelRadiusCharacterization(double omegaRadiansPerSecond) {
        selfRelativeDrive(new ChassisSpeeds(0, 0, omegaRadiansPerSecond));
    }

    /**
     * Drives the swerve with robot-relative targetSpeeds without using setpoint generation.
     * Used for PathPlanner because it generates setpoints automatically.
     *
     * @param targetSpeeds the pre generated robot relative target speeds
     */
    public void selfRelativeDriveWithoutSetpointGeneration(ChassisSpeeds targetSpeeds) {
        if (isStill(targetSpeeds)) {
            stop();
            return;
        }

        final SwerveModuleState[] swerveModuleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(targetSpeeds);
        setTargetModuleStates(swerveModuleStates);
    }

    void initializeDrive(boolean shouldUseClosedLoop) {
        previousSetpoint = new SwerveSetpoint(getSelfRelativeVelocity(), getModuleStates(), DriveFeedforwards.zeros(PathPlannerConstants.ROBOT_CONFIG.numModules));
        setClosedLoop(shouldUseClosedLoop);
        resetRotationController();
    }

    void resetRotationController() {
        SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.reset(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose().getRotation().getDegrees());
        SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.setGoal(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose().getRotation().getDegrees());
    }

    /**
     * Moves the robot to a certain pose using PID.
     *
     * @param targetPose the target pose, relative to the blue alliance driver station's right corner
     */
    void pidToPose(FlippablePose2d targetPose) {
        final Pose2d currentPose = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose();
        final Pose2d flippedTargetPose = targetPose.get();
        final double xSpeed = SwerveConstants.TRANSLATION_PID_CONTROLLER.calculate(currentPose.getX(), flippedTargetPose.getX());
        final double ySpeed = SwerveConstants.TRANSLATION_PID_CONTROLLER.calculate(currentPose.getY(), flippedTargetPose.getY());
        final int direction = Flippable.isRedAlliance() ? -1 : 1;
        final ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
                xSpeed * direction,
                ySpeed * direction,
                calculateProfiledAngleSpeedToTargetAngle(targetPose.getRotation())
        );
        selfRelativeDrive(fieldRelativeSpeedsToSelfRelativeSpeeds(targetFieldRelativeSpeeds));
    }

    /**
     * Drives the swerve with the given powers and a target angle, relative to the field's frame of reference.
     *
     * @param xPower      the x power
     * @param yPower      the y power
     * @param targetAngle the target angle, relative to the blue alliance's forward position
     */
    void fieldRelativeDrive(double xPower, double yPower, FlippableRotation2d targetAngle) {
        final ChassisSpeeds speeds = selfRelativeSpeedsFromFieldRelativePowers(xPower, yPower, 0);
        speeds.omegaRadiansPerSecond = calculateProfiledAngleSpeedToTargetAngle(targetAngle);
        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers, relative to the field's frame of reference.
     *
     * @param xPower     the x power
     * @param yPower     the y power
     * @param thetaPower the theta power
     */
    void fieldRelativeDrive(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds speeds = selfRelativeSpeedsFromFieldRelativePowers(xPower, yPower, thetaPower);
        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers, relative to the robot's frame of reference.
     *
     * @param xPower     the x power
     * @param yPower     the y power
     * @param thetaPower the theta power
     */
    void selfRelativeDrive(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, thetaPower);
        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers and a target angle, relative to the robot's frame of reference.
     *
     * @param xPower      the x power
     * @param yPower      the y power
     * @param targetAngle the target angle
     */
    void selfRelativeDrive(double xPower, double yPower, FlippableRotation2d targetAngle) {
        final ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, 0);
        speeds.omegaRadiansPerSecond = calculateProfiledAngleSpeedToTargetAngle(targetAngle);
        selfRelativeDrive(speeds);
    }

    /**
     * This method will take in desired robot-relative chassis targetSpeeds,
     * generate a swerve setpoint, then set the target state for each module
     *
     * @param targetSpeeds the desired robot-relative targetSpeeds
     */
    private void selfRelativeDrive(ChassisSpeeds targetSpeeds) {
        previousSetpoint = setpointGenerator.generateSetpoint(
                previousSetpoint,
                targetSpeeds,
                RobotHardwareStats.getPeriodicTimeSeconds()
        );

        if (isStill(previousSetpoint.robotRelativeSpeeds())) {
            stop();
            return;
        }

        setTargetModuleStates(previousSetpoint.moduleStates());
    }

    private void setTargetModuleStates(SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < swerveModules.length; i++)
            swerveModules[i].setTargetState(swerveModuleStates[i]);
    }

    private Rotation2d getDriveRelativeAngle() {
        final Rotation2d currentAngle = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose().getRotation();
        return Flippable.isRedAlliance() ? currentAngle.rotateBy(Rotation2d.fromDegrees(180)) : currentAngle;
    }

    private void setClosedLoop(boolean shouldUseClosedLoop) {
        for (SwerveModule currentModule : swerveModules)
            currentModule.shouldDriveMotorUseClosedLoop(shouldUseClosedLoop);
    }

    /**
     * Returns whether the given chassis speeds are considered to be "still" by the swerve neutral deadband.
     *
     * @param chassisSpeeds the chassis speeds to check
     * @return true if the chassis speeds are considered to be "still"
     */
    private boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    private ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower) {
        return new ChassisSpeeds(
                xPower * SwerveConstants.MAXIMUM_SPEED_METERS_PER_SECOND,
                yPower * SwerveConstants.MAXIMUM_SPEED_METERS_PER_SECOND,
                thetaPower * SwerveConstants.MAXIMUM_ROTATIONAL_SPEED_RADIANS_PER_SECOND
        );
    }

    private void updatePoseEstimatorStates() {
        final double[] odometryUpdatesYawDegrees = gyro.getThreadedSignal(Pigeon2Signal.YAW);
        final int odometryUpdates = odometryUpdatesYawDegrees.length;
        final SwerveModulePosition[][] swerveWheelPositions = new SwerveModulePosition[odometryUpdates][];
        final Rotation2d[] gyroRotations = new Rotation2d[odometryUpdates];

        for (int i = 0; i < odometryUpdates; i++) {
            swerveWheelPositions[i] = getSwerveWheelPositions(i);
            gyroRotations[i] = Rotation2d.fromDegrees(odometryUpdatesYawDegrees[i]);
        }

        RobotContainer.POSE_ESTIMATOR.updatePoseEstimatorStates(swerveWheelPositions, gyroRotations, phoenix6SignalThread.getLatestTimestamps());
    }

    private SwerveModulePosition[] getSwerveWheelPositions(int odometryUpdateIndex) {
        final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++)
            swerveModulePositions[i] = swerveModules[i].getOdometryPosition(odometryUpdateIndex);
        return swerveModulePositions;
    }

    private boolean atTranslationPosition(double currentTranslationPosition, double targetTranslationPosition, double currentTranslationVelocity) {
        return Math.abs(currentTranslationPosition - targetTranslationPosition) < SwerveConstants.TRANSLATION_TOLERANCE_METERS &&
                Math.abs(currentTranslationVelocity) < SwerveConstants.TRANSLATION_VELOCITY_TOLERANCE;
    }

    private double calculateProfiledAngleSpeedToTargetAngle(FlippableRotation2d targetAngle) {
        if (targetAngle != null)
            SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.setGoal(targetAngle.get().getDegrees());

        final Rotation2d currentAngle = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose().getRotation();
        return Units.degreesToRadians(SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.calculate(currentAngle.getDegrees()));
    }

    private ChassisSpeeds selfRelativeSpeedsFromFieldRelativePowers(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds fieldRelativeSpeeds = powersToSpeeds(xPower, yPower, thetaPower);
        return fieldRelativeSpeedsToSelfRelativeSpeeds(fieldRelativeSpeeds);
    }

    private ChassisSpeeds fieldRelativeSpeedsToSelfRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getDriveRelativeAngle());
    }

    private void updateHardware() {
        gyro.update();

        for (SwerveModule currentModule : swerveModules)
            currentModule.updatePeriodically();

        phoenix6SignalThread.updateLatestTimestamps();
    }

    private void updateNetworkTables() {
        final ChassisSpeeds selfRelativeVelocity = getSelfRelativeVelocity();
        Logger.recordOutput("Swerve/Velocity/OmegaRadiansPerSecond", selfRelativeVelocity.omegaRadiansPerSecond);
        Logger.recordOutput("Swerve/Velocity/XMetersPerSecond", selfRelativeVelocity.vxMetersPerSecond);
        Logger.recordOutput("Swerve/Velocity/YMetersPerSecond", selfRelativeVelocity.vyMetersPerSecond);
    }

    @AutoLogOutput(key = "Swerve/CurrentStates")
    private SwerveModuleState[] getModuleStates() {
        final SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];

        for (int i = 0; i < swerveModules.length; i++)
            states[i] = swerveModules[i].getCurrentState();

        return states;
    }

    @AutoLogOutput(key = "Swerve/TargetStates")
    @SuppressWarnings("unused")
    private SwerveModuleState[] getTargetStates() {
        final SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];

        for (int i = 0; i < swerveModules.length; i++)
            states[i] = swerveModules[i].getTargetState();

        return states;
    }
}