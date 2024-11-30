package frc.trigon.robot.misc.pathplanner;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import frc.trigon.robot.RobotContainer;
import org.json.simple.parser.ParseException;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.utilities.LocalADStarAK;
import org.trigon.utilities.mirrorable.Mirrorable;

import java.io.IOException;

public class PathPlannerConstants {
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

    public static void init() {
        configureAutoBuilder();
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathfindingCommand.warmupCommand().schedule();
    }

    private static void configureAutoBuilder() {
        AutoBuilder.configure(
                RobotContainer.POSE_ESTIMATOR::getCurrentEstimatedPose,
                null,
                RobotContainer.SWERVE::getSelfRelativeVelocity,
                (speeds, feedforwards) -> RobotContainer.SWERVE.selfRelativeDrive(speeds),
                AUTO_PATH_FOLLOWING_CONTROLLER,
                getRobotConfig(),
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
}
