package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import lib.utilities.flippable.FlippablePose2d;

import java.util.function.Supplier;

import static frc.trigon.robot.RobotContainer.OBJECT_POSE_ESTIMATOR;

/**
 * A dynamic autonomous command intended for the 15-second autonomous period at the beginning of a match.
 * By dynamic, we mean that the command isn't pre-programmed and instead autonomously decides what game pieces to collect and where to score.
 */
public class DynamicAutonomousCommand extends SequentialCommandGroup {
    private final Supplier<Command>
            intakeSequenceCommandSupplier,
            findGamePieceCommandSupplier,
            scoreCommandSupplier;
    private final Supplier<FlippablePose2d> scoringPoseSupplier;
    private final Supplier<Boolean> hasGamePieceSupplier;
    private FlippablePose2d targetScoringPose = null;

    /**
     * Creates a dynamic autonomous command intended for the 15-second autonomous period at the beginning of a match.
     *
     * @param intakeSequenceCommandSupplier a supplier for the intake sequence command
     * @param findGamePieceCommandSupplier  a supplier for the find game piece command
     * @param scoreCommandSupplier          a supplier for the score command
     * @param scoringPoseSupplier           a supplier for the target scoring pose
     * @param hasGamePieceSupplier          a supplier that returns whether the robot currently has a game piece or not
     */
    public DynamicAutonomousCommand(
            Supplier<Command> intakeSequenceCommandSupplier,
            Supplier<Command> findGamePieceCommandSupplier,
            Supplier<Command> scoreCommandSupplier,
            Supplier<FlippablePose2d> scoringPoseSupplier,
            Supplier<Boolean> hasGamePieceSupplier
    ) {
        this.intakeSequenceCommandSupplier = intakeSequenceCommandSupplier;
        this.findGamePieceCommandSupplier = findGamePieceCommandSupplier;
        this.scoreCommandSupplier = scoreCommandSupplier;
        this.scoringPoseSupplier = scoringPoseSupplier;
        this.hasGamePieceSupplier = hasGamePieceSupplier;

        this.addCommands(
                new SequentialCommandGroup(
                        getDriveAndScoreCommand(),
                        getCollectionCommand()
                ).repeatedly()
        );
    }

    private Command getCollectionCommand() {
        return new ParallelCommandGroup(
                intakeSequenceCommandSupplier.get(),
                getDriveToGamePieceCommand()
        ).until(hasGamePieceSupplier::get);
    }

    private Command getDriveToGamePieceCommand() {
        return new ConditionalCommand(
                IntakeAssistCommand.getAssistIntakeCommand(IntakeAssistCommand.AssistMode.FULL_ASSIST, IntakeAssistCommand::calculateDistanceFromTrackedGamePiece, OperatorConstants.INTAKE_ASSIST_SCALAR).onlyWhile(() -> OBJECT_POSE_ESTIMATOR.getClosestObjectToRobot() != null),
                findGamePieceCommandSupplier.get().until(() -> OBJECT_POSE_ESTIMATOR.getClosestObjectToRobot() != null),
                () -> OBJECT_POSE_ESTIMATOR.getClosestObjectToRobot() != null
        );
    }

    private Command getDriveAndScoreCommand() {
        return new ParallelCommandGroup(
                getDriveToScoringLocationCommand(),
                scoreCommandSupplier.get()
        );
    }

    private Command getDriveToScoringLocationCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> targetScoringPose = scoringPoseSupplier.get()),
                new WaitUntilCommand(() -> targetScoringPose != null).raceWith(SwerveCommands.getClosedLoopSelfRelativeDriveCommand(() -> 0, () -> 0, () -> 0)),
                SwerveCommands.getDriveToPoseCommand(() -> targetScoringPose, AutonomousConstants.DRIVE_TO_SCORING_LOCATION_CONSTRAINTS).repeatedly()
        );
    }

}