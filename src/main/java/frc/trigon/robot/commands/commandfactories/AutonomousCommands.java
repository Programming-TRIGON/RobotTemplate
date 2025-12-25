package frc.trigon.robot.commands.commandfactories;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.GamePieceAutoDriveCommand;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.Supplier;

/**
 * A class that contains command factories for preparation commands and commands used during the 15-second autonomous period at the start of each match.
 */
public class AutonomousCommands {
    private static FlippablePose2d TARGET_SCORING_POSE = null;

    /**
     * Creates a dynamic autonomous command intended for the 15-second autonomous period at the beginning of a match.
     * Dynamic means that the command isn't pre-programmed and instead autonomously decides what game pieces to collect and where to score.
     *
     * @param intakeLocations  the locations at which to collect game pieces
     * @param scoringLocations the locations at which to score
     * @return the command
     */
    public static Command getDynamicAutonmousCommand(FlippablePose2d[] intakeLocations, FlippablePose2d... scoringLocations) {
        return new SequentialCommandGroup(
                getDriveAndScoreCommand(scoringLocations),
                getCollectCommand(intakeLocations)
        ).repeatedly().withName(generateDynamicAutonomousRoutineName(intakeLocations, scoringLocations));
    }

    private static Command getCollectCommand(FlippablePose2d[] intakeLocations) {
        return new ParallelCommandGroup(
                getIntakeSequenceCommand(),
                getDriveToGamePieceCommand(intakeLocations)
        ).until(AutonomousCommands::hasGamePiece);
    }

    private static boolean hasGamePiece() {
        //TODO: implement
        return false;
    }

    private static Command getDriveToGamePieceCommand(FlippablePose2d[] intakeLocations) {
        return new ConditionalCommand(
                new GamePieceAutoDriveCommand(SimulatedGamePieceConstants.GamePieceType.GAME_PIECE_TYPE).onlyWhile(() -> RobotContainer.OBJECT_POSE_ESTIMATOR.getClosestObjectToRobot() != null),
                getFindGamePieceCommand(intakeLocations),
                AutonomousCommands::shouldCollectGamePiece
        );
    }

    private static boolean shouldCollectGamePiece() {
        return RobotContainer.OBJECT_POSE_ESTIMATOR.getClosestObjectToRobot() != null;
    }

    private static Command getFindGamePieceCommand(FlippablePose2d[] intakeLocations) {
        //TODO: implement
        return null;
    }

    private static Command getIntakeSequenceCommand() {
        //TODO: implement
        return null;
    }

    private static Command getDriveAndScoreCommand(FlippablePose2d[] scoringLocations) {
        return new ParallelCommandGroup(
                getDriveToScoringLocationCommand(scoringLocations),
                getScoringSequenceCommand()
        );
    }

    private static Command getScoringSequenceCommand() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> TARGET_SCORING_POSE != null),
                getScoreCommand()
        );
    }

    private static Command getScoreCommand() {
        //TODO: implement
        return null;
    }

    private static Command getDriveToScoringLocationCommand(FlippablePose2d[] scoringLocations) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> TARGET_SCORING_POSE = calculateBestOpenScoringPose(scoringLocations)),
                new WaitUntilCommand(() -> TARGET_SCORING_POSE != null).raceWith(SwerveCommands.getClosedLoopSelfRelativeDriveCommand(() -> 0, () -> 0, () -> 0)),
                SwerveCommands.getDriveToPoseCommand(() -> TARGET_SCORING_POSE, AutonomousConstants.DRIVE_TO_SCORING_LOCATION_CONSTRAINTS).repeatedly()
        );
    }

    private static FlippablePose2d calculateBestOpenScoringPose(FlippablePose2d[] scoringLocations) {
        //TODO: implement
        return null;
    }

    /**
     * Generates a name for the dynamic autonomous routine based on the intake and scoring locations.
     *
     * @param intakeLocations  the intake locations
     * @param scoringLocations the scoring locations
     * @return the name of the dynamic autonomous routine
     */
    private static String generateDynamicAutonomousRoutineName(FlippablePose2d[] intakeLocations, FlippablePose2d[] scoringLocations) {
        //TODO: implement
        return "";
    }

    /**
     * Creates a command that resets the pose estimator's pose to the starting pose of the given autonomous as long as the robot is not enabled.
     *
     * @param autoName the name of the autonomous
     * @return a command that resets the robot's pose estimator pose to the start position of the given autonomous
     */
    public static Command getResetPoseToAutoPoseCommand(Supplier<String> autoName) {
        return new InstantCommand(
                () -> {
                    if (DriverStation.isEnabled())
                        return;
                    RobotContainer.ROBOT_POSE_ESTIMATOR.resetPose(getAutoStartPose(autoName.get()));
                }
        ).ignoringDisable(true);
    }

    /**
     * Gets the starting position of the target PathPlanner autonomous.
     *
     * @param autoName the name of the autonomous group
     * @return the staring pose of the autonomous
     */
    public static Pose2d getAutoStartPose(String autoName) {
        try {
            final Pose2d nonFlippedAutoStartPose = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get();
            final FlippablePose2d flippedAutoStartPose = new FlippablePose2d(nonFlippedAutoStartPose, true);
            return flippedAutoStartPose.get();
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            return new Pose2d();
        }
    }
}