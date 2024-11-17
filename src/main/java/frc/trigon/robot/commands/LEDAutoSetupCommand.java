package frc.trigon.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripConstants;
import org.json.simple.parser.ParseException;
import org.trigon.utilities.mirrorable.MirrorablePose2d;

import java.awt.*;
import java.io.IOException;

/**
 * A command that sets the LED strips to a color based on the robot's position and orientation relative to the starting
 * pose of the selected autonomous path.
 * This is very useful for placing the robot in the correct starting position and orientation for autonomous mode before a match.
 */
public class LEDAutoSetupCommand extends SequentialCommandGroup {
    private static final double
            TOLERANCE_METERS = 0.1,
            TOLERANCE_DEGREES = 2;
    private Pose2d autoStartPose;

    public LEDAutoSetupCommand() {
        addCommands(
                getUpdateAutoStartPoseCommand(),
                LEDStripCommands.getThreeSectionColorCommand(
                        () -> getLeftStripColor(this.autoStartPose.getRotation().getDegrees() - getCurrentRobotPose().getRotation().getDegrees(), TOLERANCE_DEGREES),
                        () -> getLeftStripColor(this.autoStartPose.getX() - getCurrentRobotPose().getX(), TOLERANCE_METERS),
                        () -> getLeftStripColor(this.autoStartPose.getY() - getCurrentRobotPose().getY(), TOLERANCE_METERS),
                        LEDStripConstants.REAR_RIGHT_STRIP, LEDStripConstants.FRONT_RIGHT_STRIP
                ).alongWith(
                        LEDStripCommands.getThreeSectionColorCommand(
                                () -> getRightStripColor(this.autoStartPose.getRotation().getDegrees() - getCurrentRobotPose().getRotation().getDegrees(), TOLERANCE_DEGREES),
                                () -> getRightStripColor(this.autoStartPose.getX() - getCurrentRobotPose().getX(), TOLERANCE_METERS),
                                () -> getRightStripColor(this.autoStartPose.getY() - getCurrentRobotPose().getY(), TOLERANCE_METERS),
                                LEDStripConstants.REAR_LEFT_STRIP, LEDStripConstants.FRONT_LEFT_STRIP
                        )
                )
        );
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    private Command getUpdateAutoStartPoseCommand() {
        return new InstantCommand(
                () -> {
                    try {
                        final Pose2d nonMirroredAutoStartPose = PathPlannerPath.fromPathFile(PathPlannerAuto.currentPathName).getStartingHolonomicPose().get();
                        final MirrorablePose2d mirroredAutoStartPose = new MirrorablePose2d(nonMirroredAutoStartPose, true);
                        this.autoStartPose = mirroredAutoStartPose.get();
                    } catch (IOException | ParseException e) {
                        throw new RuntimeException(e);
                    }
                }
        ).ignoringDisable(true);
    }

    private Color getLeftStripColor(double difference, double tolerance) {
        if (difference < -tolerance)
            return Color.black;
        else if (difference > tolerance)
            return Color.red;
        return Color.green;
    }

    private Color getRightStripColor(double difference, double tolerance) {
        if (difference > tolerance)
            return Color.black;
        else if (difference < -tolerance)
            return Color.red;
        return Color.green;
    }

    private Pose2d getCurrentRobotPose() {
        return RobotContainer.POSE_ESTIMATOR.getCurrentPose();
    }
}
