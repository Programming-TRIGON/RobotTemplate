package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.AbstractSubsystem;
import frc.trigon.robot.utilities.AllianceUtilities;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class Swerve extends AbstractSubsystem {
    private static Swerve INSTANCE = null;
    private final SwerveInputsAutoLogged swerveInputs = new SwerveInputsAutoLogged();
    private final SwerveIO swerveIO = SwerveIO.generateIO();
    private final SwerveConstants constants = SwerveConstants.generateConstants();
    private final SwerveModuleIO[] modulesIO;
    private final List<Double> previousLoopTimestamps = new ArrayList<>();

    public static Swerve getInstance() {
        if (INSTANCE == null)
            INSTANCE = new Swerve();
        return INSTANCE;
    }

    private Swerve() {
        modulesIO = getModulesIO();
        configurePathPlanner();
    }

    @Override
    public void periodic() {
        swerveIO.updateInputs(swerveInputs);
        Logger.processInputs("Swerve", swerveInputs);

        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.periodic();

        updateNetworkTables();
        updatePreviousLoopTimestamps();
    }

    @Override
    public void stop() {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.stop();
    }

    @Override
    public void setBrake(boolean brake) {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.setBrake(brake);
    }

    public SwerveConstants getConstants() {
        return constants;
    }

    public double getGyroZAcceleration() {
        return swerveInputs.accelerationZ;
    }

    public double getGyroYAcceleration() {
        return swerveInputs.accelerationY;
    }

    public double getGyroXAcceleration() {
        return swerveInputs.accelerationX;
    }

    public Rotation2d getHeading() {
        final double heading = swerveInputs.gyroYawDegrees;
        final double inputtedHeading = MathUtil.inputModulus(heading, -180, 180);

        return Rotation2d.fromDegrees(inputtedHeading);
    }

    public ChassisSpeeds getSelfRelativeVelocity() {
        return constants.getKinematics().toChassisSpeeds(getModuleStates());
    }

    public void setHeading(Rotation2d heading) {
        swerveIO.setHeading(heading);
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(swerveInputs.gyroPitchDegrees);
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(swerveInputs.gyroRollDegrees);
    }

    public Rotation2d getPitchVelocity() {
        return Rotation2d.fromDegrees(swerveInputs.gyroPitchVelocity);
    }

    public Rotation2d getRollVelocity() {
        return Rotation2d.fromDegrees(swerveInputs.gyroRollVelocity);
    }

    public SwerveModulePosition[] getModulePositions() {
        final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            swerveModulePositions[i] = modulesIO[i].getCurrentPosition();

        return swerveModulePositions;
    }

    public boolean atPose(Pose2d pose2d) {
        return atXAxisPosition(pose2d.getX()) && atYAxisPosition(pose2d.getY()) && atAngle(pose2d.getRotation());
    }

    public boolean atXAxisPosition(double xAxisPosition) {
        final double currentXAxisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(getSelfRelativeVelocity(), RobotContainer.POSE_ESTIMATOR.getCurrentPose().toCurrentAlliancePose().getRotation()).vxMetersPerSecond;
        return atTranslationPosition(RobotContainer.POSE_ESTIMATOR.getCurrentPose().toCurrentAlliancePose().getX(), xAxisPosition, currentXAxisVelocity);
    }

    public boolean atYAxisPosition(double yAxisPosition) {
        final double currentYAxisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(getSelfRelativeVelocity(), RobotContainer.POSE_ESTIMATOR.getCurrentPose().toCurrentAlliancePose().getRotation()).vyMetersPerSecond;
        return atTranslationPosition(RobotContainer.POSE_ESTIMATOR.getCurrentPose().toCurrentAlliancePose().getY(), yAxisPosition, currentYAxisVelocity);
    }

    public boolean atAngle(Rotation2d angle) {
        return Math.abs(angle.getDegrees() - RobotContainer.POSE_ESTIMATOR.getCurrentPose().toCurrentAlliancePose().getRotation().getDegrees()) < SwerveConstants.ROTATION_TOLERANCE &&
                Math.abs(getSelfRelativeVelocity().omegaRadiansPerSecond) < SwerveConstants.ROTATION_VELOCITY_TOLERANCE;
    }

    void initializeDrive(boolean closedLoop) {
        setClosedLoop(closedLoop);
        resetRotationController();
    }

    /**
     * Locks the swerve, so it'll be hard to move it. This will make the modules look in the middle of a robot in a "x" shape.
     */
    void lockSwerve() {
        setBrake(true);
        final SwerveModuleState
                right = new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                left = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

        modulesIO[0].setTargetState(left);
        modulesIO[1].setTargetState(right);
        modulesIO[2].setTargetState(right);
        modulesIO[3].setTargetState(left);
    }

    void resetRotationController() {
        constants.getProfiledRotationController().reset(RobotContainer.POSE_ESTIMATOR.getCurrentPose().toCurrentAlliancePose().getRotation().getDegrees());
    }

    void setClosedLoop(boolean closedLoop) {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.setDriveMotorClosedLoop(closedLoop);
    }

    /**
     * Drives the swerve with the given powers and a target angle, relative to the field's frame of reference.
     *
     * @param xPower      the x power
     * @param yPower      the y power
     * @param targetAngle the target angle
     */
    void fieldRelativeDrive(double xPower, double yPower, Rotation2d targetAngle) {
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

    private void selfRelativeDrive(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds = discretize(chassisSpeeds);
        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }

        final SwerveModuleState[] swerveModuleStates = constants.getKinematics().toSwerveModuleStates(chassisSpeeds);
        setTargetModuleStates(swerveModuleStates);
    }

    private void setTargetModuleStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, constants.getMaxModuleSpeedMetersPerSecond());
        for (int i = 0; i < modulesIO.length; i++)
            modulesIO[i].setTargetState(swerveModuleStates[i]);
    }

    /**
     * When the robot drives while rotating it skews a bit to the side.
     * This should fix the chassis speeds, so they won't make the robot skew while rotating.
     *
     * @param chassisSpeeds the chassis speeds to fix skewing for
     * @return the fixed speeds
     */
    private ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
        return ChassisSpeeds.discretize(chassisSpeeds, getAverageLoopTime());
    }

    private double getAverageLoopTime() {
        if (previousLoopTimestamps.size() < SwerveConstants.MAX_SAVED_PREVIOUS_LOOP_TIMESTAMPS)
            return RobotConstants.PERIODIC_TIME_SECONDS;

        double differenceSum = 0;
        double lastTimestamp = -1;

        for (double currentTimestamp : previousLoopTimestamps) {
            if (lastTimestamp == -1) {
                lastTimestamp = currentTimestamp;
                continue;
            }
            differenceSum += currentTimestamp - lastTimestamp;
            lastTimestamp = currentTimestamp;
        }

        return differenceSum / (previousLoopTimestamps.size() - 1);
    }

    private void updatePreviousLoopTimestamps() {
        if (previousLoopTimestamps.size() < SwerveConstants.MAX_SAVED_PREVIOUS_LOOP_TIMESTAMPS) {
            previousLoopTimestamps.add(Timer.getFPGATimestamp());
            return;
        }

        previousLoopTimestamps.remove(0);
        previousLoopTimestamps.add(Timer.getFPGATimestamp());
    }

    private void configurePathPlanner() {
        System.out.println(Timer.getFPGATimestamp());
        AutoBuilder.configureHolonomic(
                () -> RobotContainer.POSE_ESTIMATOR.getCurrentPose().toCurrentAlliancePose(),
                (pose2d) -> RobotContainer.POSE_ESTIMATOR.resetPose(AllianceUtilities.AlliancePose2d.fromCurrentAlliancePose(pose2d)),
                this::getSelfRelativeVelocity,
                this::selfRelativeDrive,
                constants.getPathFollowerConfig(),
                this
        );
    }

    private boolean atTranslationPosition(double currentTranslationPosition, double targetTranslationPosition, double currentTranslationVelocity) {
        return Math.abs(currentTranslationPosition - targetTranslationPosition) < SwerveConstants.TRANSLATION_TOLERANCE &&
                Math.abs(currentTranslationVelocity) < SwerveConstants.TRANSLATION_VELOCITY_TOLERANCE;
    }

    private double calculateProfiledAngleSpeedToTargetAngle(Rotation2d targetAngle) {
        constants.getProfiledRotationController().setGoal(targetAngle.getDegrees());
        final Rotation2d currentAngle = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toCurrentAlliancePose().getRotation();
        return Units.degreesToRadians(constants.getProfiledRotationController().calculate(currentAngle.getDegrees()));
    }

    private ChassisSpeeds selfRelativeSpeedsFromFieldRelativePowers(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds fieldRelativeSpeeds = powersToSpeeds(xPower, yPower, thetaPower);
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, RobotContainer.POSE_ESTIMATOR.getCurrentPose().toCurrentAlliancePose().getRotation());
    }

    private ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower) {
        return new ChassisSpeeds(
                xPower * constants.getMaxSpeedMetersPerSecond(),
                yPower * constants.getMaxSpeedMetersPerSecond(),
                Math.pow(thetaPower, 2) * Math.signum(thetaPower) * constants.getMaxRotationalSpeedRadiansPerSecond()
        );
    }

    private void updateNetworkTables() {
        Logger.recordOutput("Swerve/Velocity/Rot", getSelfRelativeVelocity().omegaRadiansPerSecond);
        Logger.recordOutput("Swerve/Velocity/X", getSelfRelativeVelocity().vxMetersPerSecond);
        Logger.recordOutput("Swerve/Velocity/Y", getSelfRelativeVelocity().vyMetersPerSecond);
    }

    @AutoLogOutput(key = "Swerve/CurrentStates")
    private SwerveModuleState[] getModuleStates() {
        final SwerveModuleState[] states = new SwerveModuleState[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            states[i] = modulesIO[i].getCurrentState();

        return states;
    }

    @AutoLogOutput(key = "Swerve/TargetStates")
    private SwerveModuleState[] getTargetStates() {
        final SwerveModuleState[] states = new SwerveModuleState[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            states[i] = modulesIO[i].getTargetState();

        return states;
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

    private SwerveModuleIO[] getModulesIO() {
        if (RobotConstants.IS_REPLAY) {
            return new SwerveModuleIO[]{
                    new SwerveModuleIO("FrontLeft"),
                    new SwerveModuleIO("FrontRight"),
                    new SwerveModuleIO("RearLeft"),
                    new SwerveModuleIO("RearRight")
            };
        }

        return constants.getModulesIO();
    }
}