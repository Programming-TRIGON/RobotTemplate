package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import frc.trigon.robot.RobotContainer;
import org.json.simple.parser.ParseException;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.utilities.LocalADStarAK;
import org.trigon.utilities.mirrorable.Mirrorable;

import java.io.IOException;

/**
 * A class that contains the constants and configurations for everything related to PathPlanner.
 */
public class PathPlannerConstants {
    public static final PathConstraints REAL_TIME_PATH_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);
    public static final RobotConfig ROBOT_CONFIG = getRobotConfig();

    private static final PIDConstants
            AUTO_TRANSLATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
            new PIDConstants(5, 0, 0) :
            new PIDConstants(2, 0, 0),
            AUTO_ROTATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
                    new PIDConstants(2.5, 0, 0) :
                    new PIDConstants(6.5, 0, 0);

    private static final PPHolonomicDriveController AUTO_PATH_FOLLOWING_CONTROLLER = new PPHolonomicDriveController(
            AUTO_TRANSLATION_PID_CONSTANTS,
            AUTO_ROTATION_PID_CONSTANTS
    );

    /**
     * Initializes PathPlanner. This needs to be called before any PathPlanner function can be used.
     */
    public static void init() {
        Pathfinding.setPathfinder(new LocalADStarAK());
        configureAutoBuilder();
        PathfindingCommand.warmupCommand().schedule();
        registerCommands();
    }

    private static void configureAutoBuilder() {
        AutoBuilder.configure(
                RobotContainer.POSE_ESTIMATOR::getCurrentEstimatedPose,
                RobotContainer.POSE_ESTIMATOR::resetPose,
                RobotContainer.SWERVE::getSelfRelativeVelocity,
                RobotContainer.SWERVE::selfRelativeDriveWithoutSetpointGeneration,
                AUTO_PATH_FOLLOWING_CONTROLLER,
                ROBOT_CONFIG,
                Mirrorable::isRedAlliance,
                RobotContainer.SWERVE
        );
    }

    private static RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }

    private static void registerCommands() {
//        NamedCommands.registerCommand(name, command);//TODO:Implement NamedCommands
    }
}