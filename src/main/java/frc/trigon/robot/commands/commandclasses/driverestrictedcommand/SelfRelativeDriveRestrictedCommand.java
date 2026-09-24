package frc.trigon.robot.commands.commandclasses.driverestrictedcommand;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions.DriveRestriction;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

/**
 * A command that drives the robot relative to itself while restricting its movement.
 * All restrictions are applied sequentially, each further restricting the previous result.
 */
public class SelfRelativeDriveRestrictedCommand extends DriveRestrictedCommand {
    /**
     * Constructs a command that drives the robot relative to itself and restricts its movement.
     *
     * @param driveRestrictions restricts the robot's movement
     */
    public SelfRelativeDriveRestrictedCommand(DriveRestriction... driveRestrictions) {
        super(driveRestrictions);
    }

    @Override
    protected Command getDriveCommand() {
        return SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                () -> restrictedXPower,
                () -> restrictedYPower,
                () -> restrictedRotationPower,
                () -> robotRelativeCenterOfRotation
        );
    }

    @Override
    protected Translation2d toFieldRelativeDrive(Translation2d targetTranslation) {
        return targetTranslation.rotateBy(RobotContainer.SWERVE.getDriveRelativeAngle());
    }

    @Override
    protected Translation2d fromFieldRelativeDrive(Translation2d targetTranslation) {
        return targetTranslation.rotateBy(RobotContainer.SWERVE.getDriveRelativeAngle().unaryMinus());
    }
}