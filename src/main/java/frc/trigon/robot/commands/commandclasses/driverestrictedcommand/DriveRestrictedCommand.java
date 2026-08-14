package frc.trigon.robot.commands.commandclasses.driverestrictedcommand;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions.DriveRestriction;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * An abstract class for a command that drives the robot while restricting its movement.
 * All restrictions are applied sequentially, each further restricting the previous result.
 */
public abstract class DriveRestrictedCommand extends ParallelCommandGroup {
    private final DriveRestriction[] driveRestrictions;
    protected Translation2d robotRelativeCenterOfRotation = Translation2d.kZero; /** Stored as a field in this class so that the command can properly interface with the Swerve method that changes the custom center of rotation,{@link SwerveCommands#getClosedLoopFieldRelativeDriveCommand(DoubleSupplier, DoubleSupplier, DoubleSupplier, Supplier)} for the swerve command */
    protected double
            restrictedXPower = 0,
            restrictedYPower = 0,
            restrictedRotationPower = 0;

    /**
     * Constructs a command that drives the robot and restricts its movement.
     *
     * @param driveRestrictions the restrictions to apply, in order. Each receives the output of the previous one.
     */
    protected DriveRestrictedCommand(DriveRestriction... driveRestrictions) {
        this.driveRestrictions = driveRestrictions;
        addCommands(
                new InstantCommand(this::init),
                new RunCommand(this::setRestrictedTranslation),
                new RunCommand(this::setRestrictedRotation),
                new RunCommand(this::setRestrictedCenterOfRotation),
                getDriveCommand()
        );
    }

    protected abstract Translation2d toFieldRelativeDrive(Translation2d targetTranslation);

    protected abstract Translation2d fromFieldRelativeDrive(Translation2d targetTranslation);

    protected abstract Command getDriveCommand();

    private void init() {
        restrictedXPower = 0;
        restrictedYPower = 0;
        restrictedRotationPower = 0;
        robotRelativeCenterOfRotation = Translation2d.kZero;
        for (DriveRestriction driveRestriction : driveRestrictions)
            driveRestriction.init();
    }

    private void setRestrictedTranslation() {
        Translation2d targetRobotTranslationPower = toFieldRelativeDrive(calculateTargetRobotTranslationPower());

        for (DriveRestriction driveRestriction : driveRestrictions) {
            targetRobotTranslationPower = driveRestriction.applyTranslationRestriction(targetRobotTranslationPower);
        }

        targetRobotTranslationPower = fromFieldRelativeDrive(targetRobotTranslationPower);

        restrictedXPower = targetRobotTranslationPower.getX();
        restrictedYPower = targetRobotTranslationPower.getY();
    }

    private void setRestrictedCenterOfRotation() {
        Translation2d targetRobotCenterOfRotation = Translation2d.kZero;

        for (DriveRestriction driveRestriction : driveRestrictions) {
            targetRobotCenterOfRotation = driveRestriction.applyCenterOfRotationRestriction(targetRobotCenterOfRotation);
        }

        robotRelativeCenterOfRotation = targetRobotCenterOfRotation;
    }

    private void setRestrictedRotation() {
        double targetRobotRotationPower = CommandConstants.calculateRotationStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getRightX());

        for (DriveRestriction driveRestriction : driveRestrictions) {
            targetRobotRotationPower = driveRestriction.applyRotationRestriction(targetRobotRotationPower);
        }

        restrictedRotationPower = targetRobotRotationPower;
    }
    /**
     * Calculates the robot's target translation based on driver input.
     *
     * @return the robot's target translation from the joystick, as powers (1,-1)
     */
    private Translation2d calculateTargetRobotTranslationPower() {
        final Translation2d rawJoystickPosition = getRawJoystickPosition();

        return new Translation2d(
                CommandConstants.calculateDriveStickAxisValue(rawJoystickPosition.getX()),
                CommandConstants.calculateDriveStickAxisValue(rawJoystickPosition.getY())
        );
    }

    /**
     * Gets the raw left joystick position of the controller.
     *
     * @return the left joystick's position as a power (1,-1), as a Translation2d
     */
    private Translation2d getRawJoystickPosition() {
        final double
                xAxis = OperatorConstants.DRIVER_CONTROLLER.getLeftY(),
                yAxis = OperatorConstants.DRIVER_CONTROLLER.getLeftX();
        return new Translation2d(xAxis, yAxis);
    }
}