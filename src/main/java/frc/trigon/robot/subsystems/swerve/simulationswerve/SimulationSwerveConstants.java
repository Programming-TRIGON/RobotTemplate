package frc.trigon.robot.subsystems.swerve.simulationswerve;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;

public class SimulationSwerveConstants extends SwerveConstants {
    private static final double RATE_LIMIT = 10;
    static final SlewRateLimiter
            X_SLEW_RATE_LIMITER = new SlewRateLimiter(RATE_LIMIT),
            Y_SLEW_RATE_LIMITER = new SlewRateLimiter(RATE_LIMIT);
    private static final double
            MAX_SPEED_METERS_PER_SECOND = 4.25,
            MAX_MODULE_SPEED_METERS_PER_SECOND = 4.25,
            MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND = 12.03;
    static final double
            SIDE_LENGTH_METERS = 0.7,
            DISTANCE_FROM_CENTER_OF_BASE = SIDE_LENGTH_METERS / 2;
    private static final Translation2d[] LOCATIONS = {
            SimulationSwerveModuleConstants.FRONT_RIGHT_MODULE_LOCATION,
            SimulationSwerveModuleConstants.FRONT_LEFT_MODULE_LOCATION,
            SimulationSwerveModuleConstants.REAR_RIGHT_MODULE_LOCATION,
            SimulationSwerveModuleConstants.REAR_LEFT_MODULE_LOCATION
    };
    private static final SimulationSwerveModuleIO[] MODULES_IO = {
            new SimulationSwerveModuleIO(SimulationSwerveModuleConstants.FRONT_RIGHT_SWERVE_MODULE_CONSTANTS, "FrontRight"),
            new SimulationSwerveModuleIO(SimulationSwerveModuleConstants.FRONT_LEFT_SWERVE_MODULE_CONSTANTS, "FrontLeft"),
            new SimulationSwerveModuleIO(SimulationSwerveModuleConstants.REAR_RIGHT_SWERVE_MODULE_CONSTANTS, "RearRight"),
            new SimulationSwerveModuleIO(SimulationSwerveModuleConstants.REAR_LEFT_SWERVE_MODULE_CONSTANTS, "RearLeft")
    };

    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);
    private static final PIDConstants
            TRANSLATION_PID_CONSTANTS = new PIDConstants(20, 0, 0),
            ROTATION_PID_CONSTANTS = new PIDConstants(12, 0, 0),
            AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(6, 0, 0);
    private static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
            720,
            720
    );

    private static final PIDController ROTATION_PID_CONTROLLER = new PIDController(
            12, 0, 0
    );

    private static final ProfiledPIDController PROFILED_PID_CONTROLLER = new ProfiledPIDController(
            ROTATION_PID_CONSTANTS.kP,
            ROTATION_PID_CONSTANTS.kI,
            ROTATION_PID_CONSTANTS.kD,
            ROTATION_CONSTRAINTS
    );

    private static final double DRIVE_RADIUS_METERS = Math.hypot(
            DISTANCE_FROM_CENTER_OF_BASE, DISTANCE_FROM_CENTER_OF_BASE
    );
    private static final ReplanningConfig REPLANNING_CONFIG = new ReplanningConfig(true, false);
    private static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            TRANSLATION_PID_CONSTANTS,
            AUTO_ROTATION_PID_CONSTANTS,
            MAX_MODULE_SPEED_METERS_PER_SECOND,
            DRIVE_RADIUS_METERS,
            REPLANNING_CONFIG // TODO: check how this works
    );

    static {
        PROFILED_PID_CONTROLLER.enableContinuousInput(-180, 180);
        PROFILED_PID_CONTROLLER.setIntegratorRange(-30, 30);
        ROTATION_PID_CONTROLLER.enableContinuousInput(-180, 180);
    }

    @Override
    protected SwerveModuleIO[] getModulesIO() {
        return MODULES_IO;
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return KINEMATICS;
    }

    @Override
    protected HolonomicPathFollowerConfig getPathFollowerConfig() {
        return HOLONOMIC_PATH_FOLLOWER_CONFIG;
    }

    @Override
    protected double getMaxSpeedMetersPerSecond() {
        return MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    protected double getMaxRotationalSpeedRadiansPerSecond() {
        return MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND;
    }

    @Override
    protected PIDController getRotationController() {
        return ROTATION_PID_CONTROLLER;
    }

    @Override
    public ProfiledPIDController getProfiledRotationController() {
        return PROFILED_PID_CONTROLLER;
    }

    @Override
    public double getRobotSideLength() {
        return SIDE_LENGTH_METERS;
    }

    @Override
    protected SlewRateLimiter getXSlewRateLimiter() {
        return X_SLEW_RATE_LIMITER;
    }

    @Override
    protected SlewRateLimiter getYSlewRateLimiter() {
        return Y_SLEW_RATE_LIMITER;
    }
}
