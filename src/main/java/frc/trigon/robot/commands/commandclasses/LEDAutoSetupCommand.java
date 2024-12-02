package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.AutonomousCommands;
import org.trigon.hardware.misc.leds.LEDCommands;
import org.trigon.hardware.misc.leds.LEDStrip;

import java.util.function.Supplier;

/**
 * A command that sets the LED strips to a color based on the robot's position and orientation relative to the starting
 * pose of the selected autonomous path.
 * This is very useful for placing the robot in the correct starting position and orientation for autonomous mode before a match.
 */
public class LEDAutoSetupCommand extends SequentialCommandGroup {
    private static final double
            TOLERANCE_METERS = 0.1,
            TOLERANCE_DEGREES = 2;
    private final Supplier<String> autoName;
    private Pose2d autoStartPose;

    /**
     * Constructs a new LEDAutoSetupCommand.
     *
     * @param autoName a supplier that returns the name of the selected autonomous path
     */
    public LEDAutoSetupCommand(Supplier<String> autoName) {
        this.autoName = autoName;

        final Supplier<Color>[] ledColors = new Supplier[]{
                () -> getDesiredLEDColorFromRobotPose(this.autoStartPose.getRotation().getDegrees() - RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose().getRotation().getDegrees(), TOLERANCE_DEGREES),
                () -> getDesiredLEDColorFromRobotPose(this.autoStartPose.getX() - RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose().getX(), TOLERANCE_METERS),
                () -> getDesiredLEDColorFromRobotPose(this.autoStartPose.getY() - RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose().getY(), TOLERANCE_METERS)
        };
        addCommands(
                getUpdateAutoStartPoseCommand(),
                LEDCommands.getSectionColorCommand(ledColors, LEDStrip.LED_STRIPS)
        );
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    private Command getUpdateAutoStartPoseCommand() {
        return new InstantCommand(() -> {
            this.autoStartPose = AutonomousCommands.getAutoStartPose(autoName.get());
        });
    }

    /**
     * Gets the correct color of a section of the LED based on the difference of between the auto start pose and the current robot pose.
     *
     * @param differenceMeters the difference between the robot's position and the auto's start position
     * @param toleranceMeters  the maximum distance from the auto start position to display as the correct start position
     * @return the desired color
     */
    private Color getDesiredLEDColorFromRobotPose(double differenceMeters, double toleranceMeters) {
        if (differenceMeters < -toleranceMeters)
            return Color.kBlack;
        else if (differenceMeters > toleranceMeters)
            return Color.kRed;
        return Color.kGreen;
    }
}
