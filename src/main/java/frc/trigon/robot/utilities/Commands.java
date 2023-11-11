package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.constants.CommandConstants;
import frc.trigon.robot.subsystems.swerve.Swerve;

public class Commands {
    private static final Swerve SWERVE = Swerve.getInstance();

    /**
     * @return a command that toggles between the swerve's default command, from field relative to self relative
     */
    public static Command getToggleFieldAndSelfRelativeDriveCommand() {
        return new InstantCommand(() -> {
            if (SWERVE.getDefaultCommand().equals(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND))
                SWERVE.setDefaultCommand(CommandConstants.SELF_RELATIVE_DRIVE_COMMAND);
            else
                SWERVE.setDefaultCommand(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND);

            SWERVE.getDefaultCommand().schedule();
        });
    }
}
