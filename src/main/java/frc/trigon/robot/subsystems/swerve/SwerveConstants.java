package frc.trigon.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.PathPlannerConstants;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimatorConstants;
import frc.trigon.robot.subsystems.swerve.swervemodule.SwerveModule;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.pigeon2.Pigeon2Gyro;
import org.trigon.hardware.phoenix6.pigeon2.Pigeon2Signal;

import java.util.function.DoubleSupplier;

public class SwerveConstants {
    private static final int PIGEON_ID = 0;
    static final Pigeon2Gyro GYRO = new Pigeon2Gyro(PIGEON_ID, "SwerveGyro", RobotConstants.CANIVORE_NAME);
    private static final double
            GYRO_MOUNT_POSITION_YAW = 0,
            GYRO_MOUNT_POSITION_PITCH = 0,
            GYRO_MOUNT_POSITION_ROLL = 0;
    private static final double
            FRONT_LEFT_STEER_ENCODER_OFFSET = 0,
            FRONT_RIGHT_STEER_ENCODER_OFFSET = 0,
            REAR_LEFT_STEER_ENCODER_OFFSET = 0,
            REAR_RIGHT_STEER_ENCODER_OFFSET = 0;
    private static final int
            FRONT_LEFT_ID = 1,
            FRONT_RIGHT_ID = 2,
            REAR_LEFT_ID = 3,
            REAR_RIGHT_ID = 4;
    private static final double
            FRONT_LEFT_WHEEL_DIAMETER_METERS = RobotHardwareStats.isSimulation() ? 0.1016 : 0.049274 * 2,
            FRONT_RIGHT_WHEEL_DIAMETER_METERS = RobotHardwareStats.isSimulation() ? 0.1016 : 0.049274 * 2,
            REAR_LEFT_WHEEL_DIAMETER_METERS = RobotHardwareStats.isSimulation() ? 0.1016 : 0.049274 * 2,
            REAR_RIGHT_WHEEL_DIAMETER_METERS = RobotHardwareStats.isSimulation() ? 0.1016 : 0.049274 * 2;
    static final SwerveModule[] SWERVE_MODULES = {
            new SwerveModule(FRONT_LEFT_ID, FRONT_LEFT_WHEEL_DIAMETER_METERS, FRONT_LEFT_STEER_ENCODER_OFFSET),
            new SwerveModule(FRONT_RIGHT_ID, FRONT_RIGHT_WHEEL_DIAMETER_METERS, FRONT_RIGHT_STEER_ENCODER_OFFSET),
            new SwerveModule(REAR_LEFT_ID, REAR_LEFT_WHEEL_DIAMETER_METERS, REAR_LEFT_STEER_ENCODER_OFFSET),
            new SwerveModule(REAR_RIGHT_ID, REAR_RIGHT_WHEEL_DIAMETER_METERS, REAR_RIGHT_STEER_ENCODER_OFFSET)
    };

    private static final DoubleSupplier SIMULATION_YAW_VELOCITY_SUPPLIER = () -> RobotContainer.SWERVE.getSelfRelativeVelocity().omegaRadiansPerSecond;
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(PathPlannerConstants.getRobotConfig().moduleLocations);
    private static final double FURTHEST_MODULE_DISTANCE_FROM_CENTER = Math.hypot(
            PathPlannerConstants.getRobotConfig().moduleLocations[0].getX(),
            PathPlannerConstants.getRobotConfig().moduleLocations[3].getY()
    );
    static final double
            TRANSLATION_TOLERANCE_METERS = 0.05,
            ROTATION_TOLERANCE_DEGREES = 2,
            TRANSLATION_VELOCITY_TOLERANCE = 0.05,
            ROTATION_VELOCITY_TOLERANCE = 0.3;
    static final double
            DRIVE_NEUTRAL_DEADBAND = 0.2,
            ROTATION_NEUTRAL_DEADBAND = 0.2;

    private static final PIDConstants
            TRANSLATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
            new PIDConstants(5, 0, 0) :
            new PIDConstants(5, 0, 0),
            PROFILED_ROTATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
                    new PIDConstants(4, 0, 0) :
                    new PIDConstants(3, 0, 0);
    private static final double
            MAXIMUM_ROTATION_VELOCITY = RobotHardwareStats.isSimulation() ? 720 : 720,
            MAXIMUM_ROTATION_ACCELERATION = RobotHardwareStats.isSimulation() ? 720 : 720;
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
    static final PIDController TRANSLATION_PID_CONTROLLER = new PIDController(
            TRANSLATION_PID_CONSTANTS.kP,
            TRANSLATION_PID_CONSTANTS.kI,
            TRANSLATION_PID_CONSTANTS.kD
    );
    public static final double
            MAXIMUM_SPEED_METERS_PER_SECOND = RobotHardwareStats.isSimulation() ? 4.9 : 4.04502,
            MAXIMUM_ROTATIONAL_SPEED_RADIANS_PER_SECOND = RobotHardwareStats.isSimulation() ? 12.03 : 12.03;

    static {
        final Pigeon2Configuration config = new Pigeon2Configuration();
        config.MountPose.MountPoseYaw = GYRO_MOUNT_POSITION_YAW;
        config.MountPose.MountPosePitch = GYRO_MOUNT_POSITION_PITCH;
        config.MountPose.MountPoseRoll = GYRO_MOUNT_POSITION_ROLL;
        GYRO.applyConfiguration(config);
        GYRO.setSimulationYawVelocitySupplier(SIMULATION_YAW_VELOCITY_SUPPLIER);

        GYRO.registerThreadedSignal(Pigeon2Signal.YAW, PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
    }
}