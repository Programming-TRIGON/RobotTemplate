package frc.trigon.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.pigeon2.Pigeon2Gyro;
import frc.trigon.lib.hardware.phoenix6.pigeon2.Pigeon2Signal;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimatorConstants;
import frc.trigon.robot.subsystems.swerve.swervemodule.SwerveModule;

public class SwerveConstants {
    private static final int GYRO_ID = 0;
    private static final String GYRO_NAME = "SwerveGyro";
    static final Pigeon2Gyro GYRO = new Pigeon2Gyro(GYRO_ID, GYRO_NAME, RobotConstants.CANIVORE_NAME);

    public static final int
            FRONT_LEFT_ID = 1,
            FRONT_RIGHT_ID = 2,
            REAR_LEFT_ID = 3,
            REAR_RIGHT_ID = 4;
    private static final double
            FRONT_LEFT_STEER_ENCODER_OFFSET_ROTATIONS = 0,
            FRONT_RIGHT_STEER_ENCODER_OFFSET_ROTATIONS = 0,
            REAR_LEFT_STEER_ENCODER_OFFSET_ROTATIONS = 0,
            REAR_RIGHT_STEER_ENCODER_OFFSET_ROTATIONS = 0;
    private static final double//TODO:Calibrate
            FRONT_LEFT_WHEEL_DIAMETER = 0.05 * 2,
            FRONT_RIGHT_WHEEL_DIAMETER = 0.05 * 2,
            REAR_LEFT_WHEEL_DIAMETER = 0.05 * 2,
            REAR_RIGHT_WHEEL_DIAMETER = 0.05 * 2;
    static final SwerveModule[] SWERVE_MODULES = new SwerveModule[]{
            new SwerveModule(FRONT_LEFT_ID, FRONT_LEFT_STEER_ENCODER_OFFSET_ROTATIONS, FRONT_LEFT_WHEEL_DIAMETER),
            new SwerveModule(FRONT_RIGHT_ID, FRONT_RIGHT_STEER_ENCODER_OFFSET_ROTATIONS, FRONT_RIGHT_WHEEL_DIAMETER),
            new SwerveModule(REAR_LEFT_ID, REAR_LEFT_STEER_ENCODER_OFFSET_ROTATIONS, REAR_LEFT_WHEEL_DIAMETER),
            new SwerveModule(REAR_RIGHT_ID, REAR_RIGHT_STEER_ENCODER_OFFSET_ROTATIONS, REAR_RIGHT_WHEEL_DIAMETER)
    };

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(AutonomousConstants.ROBOT_CONFIG.moduleLocations);
    static final double
            TRANSLATION_TOLERANCE_METERS = 0.035,
            ROTATION_TOLERANCE_DEGREES = 1.5,
            TRANSLATION_VELOCITY_TOLERANCE = 0.15,
            ROTATION_VELOCITY_TOLERANCE = 0.3;
    static final double
            DRIVE_NEUTRAL_DEADBAND = 0.2,
            ROTATION_NEUTRAL_DEADBAND = 0.2;

    public static final double
            MAXIMUM_SPEED_METERS_PER_SECOND = AutonomousConstants.ROBOT_CONFIG.moduleConfig.maxDriveVelocityMPS,
            MAXIMUM_ROTATIONAL_SPEED_RADIANS_PER_SECOND = AutonomousConstants.ROBOT_CONFIG.moduleConfig.maxDriveVelocityMPS / AutonomousConstants.ROBOT_CONFIG.modulePivotDistance[0];

    private static final PIDConstants
            TRANSLATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
            new PIDConstants(5, 0, 0) :
            new PIDConstants(6.3, 0, 0),
            PROFILED_ROTATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
                    new PIDConstants(4, 0, 0) :
                    new PIDConstants(10, 0, 0.1);
    private static final double
            MAXIMUM_ROTATION_VELOCITY = RobotHardwareStats.isSimulation() ? 720 : Units.radiansToDegrees(MAXIMUM_ROTATIONAL_SPEED_RADIANS_PER_SECOND),
            MAXIMUM_ROTATION_ACCELERATION = RobotHardwareStats.isSimulation() ? 720 : 900;
    private static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAXIMUM_ROTATION_VELOCITY,
            MAXIMUM_ROTATION_ACCELERATION
    );
    static final double MAXIMUM_PID_ANGLE = 180;
    static final ProfiledPIDController PROFILED_ROTATION_PID_CONTROLLER = new ProfiledPIDController(
            PROFILED_ROTATION_PID_CONSTANTS.kP,
            PROFILED_ROTATION_PID_CONSTANTS.kI,
            PROFILED_ROTATION_PID_CONSTANTS.kD,
            ROTATION_CONSTRAINTS
    );
    static final PIDController
            X_TRANSLATION_PID_CONTROLLER = new PIDController(
            TRANSLATION_PID_CONSTANTS.kP,
            TRANSLATION_PID_CONSTANTS.kI,
            TRANSLATION_PID_CONSTANTS.kD
    ),
            Y_TRANSLATION_PID_CONTROLLER = new PIDController(
                    TRANSLATION_PID_CONSTANTS.kP,
                    TRANSLATION_PID_CONSTANTS.kI,
                    TRANSLATION_PID_CONSTANTS.kD
            );
    private static final double
            ROTATION_PID_TOLERANCE_DEGREES = 1,
            TRANSLATION_PID_TOLERANCE_METERS = 0.02;
    static final double PID_TO_POSE_PREDICTION_TIME_SECONDS = 0.13;//TODO:Calibrate

    static {
        configureGyro();
        configurePIDControllers();
    }

    private static void configureGyro() {
        final Pigeon2Configuration config = new Pigeon2Configuration();
        //TODO:Calibrate
        config.MountPose.MountPoseYaw = 0;
        config.MountPose.MountPosePitch = 0;
        config.MountPose.MountPoseRoll = 0;

        GYRO.applyConfiguration(config);
        GYRO.setSimulationYawVelocitySupplier(() -> Units.radiansToDegrees(RobotContainer.SWERVE.getRotationalVelocityRadiansPerSecond()));

        GYRO.registerThreadedSignal(Pigeon2Signal.YAW, PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
    }

    private static void configurePIDControllers() {
        SwerveConstants.X_TRANSLATION_PID_CONTROLLER.setTolerance(TRANSLATION_PID_TOLERANCE_METERS);
        SwerveConstants.Y_TRANSLATION_PID_CONTROLLER.setTolerance(TRANSLATION_PID_TOLERANCE_METERS);

        SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.setTolerance(ROTATION_PID_TOLERANCE_DEGREES);
        SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.enableContinuousInput(-SwerveConstants.MAXIMUM_PID_ANGLE, SwerveConstants.MAXIMUM_PID_ANGLE);
    }
}