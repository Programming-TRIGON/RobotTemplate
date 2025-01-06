package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.InitExecuteCommand;
import org.trigon.utilities.flippable.FlippablePose2d;
import org.trigon.utilities.flippable.FlippableRotation2d;

import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveCommands {
    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getClosedLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return new InitExecuteCommand(
                () -> RobotContainer.SWERVE.initializeDrive(true),
                () -> RobotContainer.SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                RobotContainer.SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed loop mode.
     * This command will use pid to reach the target angle.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param angleSupplier the target angle supplier
     * @return the command
     */
    public static Command getClosedLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<FlippableRotation2d> angleSupplier) {
        return new InitExecuteCommand(
                () -> RobotContainer.SWERVE.initializeDrive(true),
                () -> RobotContainer.SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), angleSupplier.get()),
                RobotContainer.SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getOpenLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return new InitExecuteCommand(
                () -> RobotContainer.SWERVE.initializeDrive(false),
                () -> RobotContainer.SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                RobotContainer.SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in open loop mode.
     * This command will use pid to reach the target angle.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param angleSupplier the target angle supplier
     * @return the command
     */
    public static Command getOpenLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<FlippableRotation2d> angleSupplier) {
        return new InitExecuteCommand(
                () -> RobotContainer.SWERVE.initializeDrive(false),
                () -> RobotContainer.SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), angleSupplier.get()),
                RobotContainer.SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the robot's frame of reference, in closed loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getClosedLoopSelfRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return new InitExecuteCommand(
                () -> RobotContainer.SWERVE.initializeDrive(true),
                () -> RobotContainer.SWERVE.selfRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                RobotContainer.SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the robot's frame of reference, in closed loop mode.
     * This command will use pid to reach the target angle.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param angleSupplier the target angle supplier
     * @return the command
     */
    public static Command getClosedLoopSelfRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<FlippableRotation2d> angleSupplier) {
        return new InitExecuteCommand(
                () -> RobotContainer.SWERVE.initializeDrive(true),
                () -> RobotContainer.SWERVE.selfRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), angleSupplier.get()),
                RobotContainer.SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the robot's frame of reference, in open loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getOpenLoopSelfRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return new InitExecuteCommand(
                () -> RobotContainer.SWERVE.initializeDrive(false),
                () -> RobotContainer.SWERVE.selfRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                RobotContainer.SWERVE
        );
    }

    public static Command getDriveToPoseCommand(Supplier<FlippablePose2d> targetPose, PathConstraints constraints) {
        return new DeferredCommand(() -> getCurrentDriveToPoseCommand(targetPose.get(), constraints), Set.of(RobotContainer.SWERVE));
    }

    private static Command getCurrentDriveToPoseCommand(FlippablePose2d targetPose, PathConstraints constraints) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> RobotContainer.SWERVE.initializeDrive(true)),
                getPathfindToPoseCommand(targetPose, constraints),
                getPIDToPoseCommand(targetPose)
        );
    }

    private static Command getPathfindToPoseCommand(FlippablePose2d targetPose, PathConstraints pathConstraints) {
        final Pose2d targetFlippedPose = targetPose.get();
        final Pose2d currentPose = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose();
        if (currentPose.getTranslation().getDistance(targetFlippedPose.getTranslation()) < 0.35)
            return createOnTheFlyPathCommand(targetPose, pathConstraints);
        return AutoBuilder.pathfindToPose(targetFlippedPose, pathConstraints);
    }

    private static Command getPIDToPoseCommand(FlippablePose2d targetPose) {
        return new InstantCommand(RobotContainer.SWERVE::resetRotationController)
                .andThen(new RunCommand(() -> RobotContainer.SWERVE.pidToPose(targetPose))
                        .until(() -> RobotContainer.SWERVE.atPose(targetPose)));
    }

    private static Command createOnTheFlyPathCommand(FlippablePose2d targetPose, PathConstraints constraints) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose(),
                targetPose.get()
        );

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                new IdealStartingState(0, RobotContainer.SWERVE.getHeading()),
                new GoalEndState(0, targetPose.get().getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }
}