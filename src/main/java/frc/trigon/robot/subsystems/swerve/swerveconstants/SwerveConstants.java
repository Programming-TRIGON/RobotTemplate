package frc.trigon.robot.subsystems.swerve.swerveconstants;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.hardware.pigeon2.Pigeon2Gyro;
import frc.trigon.robot.hardware.pigeon2.Pigeon2Signal;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveModule;
import frc.trigon.robot.utilities.Conversions;

public abstract class SwerveConstants {
    public static final SwerveConstants SYSTEM_SPECIFIC_CONSTANTS = generateConstants();

    public static final int PIGEON_ID = 0;
    public static final Pigeon2Gyro GYRO = new Pigeon2Gyro(SwerveConstants.PIGEON_ID, "SwerveGyro", () -> 0.0, RobotConstants.CANIVORE_NAME);

    private static final double
            FRONT_LEFT_STEER_ENCODER_OFFSET = -Conversions.degreesToRevolutions(225.263672 - 360),
            FRONT_RIGHT_STEER_ENCODER_OFFSET = -Conversions.degreesToRevolutions(-256.904297 + 360),
            REAR_LEFT_STEER_ENCODER_OFFSET = -Conversions.degreesToRevolutions(108.369141),
            REAR_RIGHT_STEER_ENCODER_OFFSET = -Conversions.degreesToRevolutions(-36.035156);
    static final int
            FRONT_LEFT_ID = 0,
            FRONT_RIGHT_ID = 1,
            REAR_LEFT_ID = 2,
            REAR_RIGHT_ID = 3;
    public static final SwerveModule[] SWERVE_MODULES = {
            new SwerveModule(FRONT_LEFT_ID, FRONT_LEFT_STEER_ENCODER_OFFSET),
            new SwerveModule(FRONT_RIGHT_ID, FRONT_RIGHT_STEER_ENCODER_OFFSET),
            new SwerveModule(REAR_LEFT_ID, REAR_LEFT_STEER_ENCODER_OFFSET),
            new SwerveModule(REAR_RIGHT_ID, REAR_RIGHT_STEER_ENCODER_OFFSET)
    };

    private static final double
            MODULE_X_DISTANCE_FROM_CENTER = 0.6457 / 2,
            MODULE_Y_DISTANCE_FROM_CENTER = 0.5357 / 2;
    private static final Translation2d[] LOCATIONS = {
            new Translation2d(MODULE_X_DISTANCE_FROM_CENTER, MODULE_Y_DISTANCE_FROM_CENTER),
            new Translation2d(MODULE_X_DISTANCE_FROM_CENTER, -MODULE_Y_DISTANCE_FROM_CENTER),
            new Translation2d(-MODULE_X_DISTANCE_FROM_CENTER, MODULE_Y_DISTANCE_FROM_CENTER),
            new Translation2d(-MODULE_X_DISTANCE_FROM_CENTER, -MODULE_Y_DISTANCE_FROM_CENTER)
    };
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);
    public static final double DRIVE_RADIUS_METERS = Math.hypot(
            MODULE_X_DISTANCE_FROM_CENTER, MODULE_Y_DISTANCE_FROM_CENTER
    );

    public static final double
            TRANSLATION_TOLERANCE_METERS = 0.05,
            ROTATION_TOLERANCE_DEGREES = 2,
            TRANSLATION_VELOCITY_TOLERANCE = 0.05,
            ROTATION_VELOCITY_TOLERANCE = 0.3;
    public static final double
            DRIVE_NEUTRAL_DEADBAND = 0.2,
            ROTATION_NEUTRAL_DEADBAND = 0.2;

    private static final ReplanningConfig REPLANNING_CONFIG = new ReplanningConfig(true, false);
    public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            SYSTEM_SPECIFIC_CONSTANTS.getAutoTranslationPIDConstants(),
            SYSTEM_SPECIFIC_CONSTANTS.getAutoRotationPIDConstants(),
            SYSTEM_SPECIFIC_CONSTANTS.getMaxRotationalSpeedRadiansPerSecond(),
            SwerveConstants.DRIVE_RADIUS_METERS,
            REPLANNING_CONFIG
    );

    static {
        GYRO.applyConfiguration(SYSTEM_SPECIFIC_CONSTANTS.generateGyroConfiguration());

        GYRO.registerThreadedSignal(Pigeon2Signal.YAW, Pigeon2Signal.ANGULAR_VELOCITY_Z_WORLD, PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
    }

    private static SwerveConstants generateConstants() {
        if (RobotConstants.IS_SIMULATION)
            return new SimulationSwerveConstants();
        return new RealSwerveConstants();
    }

    public abstract double getMaxSpeedMetersPerSecond();

    public abstract double getMaxRotationalSpeedRadiansPerSecond();

    public abstract ProfiledPIDController getProfiledRotationController();

    public abstract PIDController getTranslationsController();

    public abstract PIDConstants getAutoTranslationPIDConstants();

    public abstract PIDConstants getAutoRotationPIDConstants();

    protected abstract Pigeon2Configuration generateGyroConfiguration();
}