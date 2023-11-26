package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveCommands {
    private static final Swerve SWERVE = Swerve.getInstance();

    /**
     * @return a command that waits a bit, then coasts the swerve modules, runs when disabled (should be called on disabled)
     */
    public static Command getDelayedCoastCommand() {
        return new WaitCommand(SwerveConstants.BRAKE_TIME_SECONDS)
                .andThen(new InstantCommand(() -> SWERVE.setBrake(false)))
                .ignoringDisable(true);
    }

    public static Command getDriveToPoseCommand(PathConstraints constraints, Pose2d targetPose) {
        return new DeferredCommand(() -> AutoBuilder.pathfindToPose(targetPose, constraints), Set.of(SWERVE));
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static FunctionalCommand getClosedLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, boolean rateLimit) {
        return new FunctionalCommand(
                () -> SWERVE.initializeDrive(true),
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble(), rateLimit),
                (interrupted) -> {
                },
                () -> false,
                SWERVE
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
    public static FunctionalCommand getClosedLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> angleSupplier, boolean rateLimit) {
        return new FunctionalCommand(
                () -> {
                    SWERVE.initializeDrive(true);
                    SWERVE.resetRotationController();
                    SWERVE.setLastRotationMovementAngle(null);
                },
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), angleSupplier.get(), rateLimit),
                (interrupted) -> {
                },
                () -> false,
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed open mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static FunctionalCommand getOpenLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, boolean rateLimit) {
        return new FunctionalCommand(
                () -> SWERVE.initializeDrive(false),
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble(), rateLimit),
                (interrupted) -> {
                },
                () -> false,
                SWERVE
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
    public static FunctionalCommand getOpenLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> angleSupplier, boolean rateLimit) {
        return new FunctionalCommand(
                () -> {
                    SWERVE.initializeDrive(false);
                    SWERVE.resetRotationController();
                    SWERVE.setLastRotationMovementAngle(null);
                },
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), angleSupplier.get(), rateLimit),
                (interrupted) -> {
                },
                () -> false,
                SWERVE
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
    public static FunctionalCommand getClosedLoopSelfRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, boolean rateLimit) {
        return new FunctionalCommand(
                () -> SWERVE.initializeDrive(true),
                () -> SWERVE.selfRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble(), rateLimit),
                (interrupted) -> {
                },
                () -> false,
                SWERVE
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
    public static FunctionalCommand getOpenLoopSelfRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, boolean rateLimit) {
        return new FunctionalCommand(
                () -> SWERVE.initializeDrive(false),
                () -> SWERVE.selfRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble(), rateLimit),
                (interrupted) -> {
                },
                () -> false,
                SWERVE
        );
    }
}
