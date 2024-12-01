package frc.trigon.robot.commands.commandfactories;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.RobotContainer;
import org.json.simple.parser.ParseException;
import org.trigon.utilities.mirrorable.MirrorablePose2d;

import java.io.IOException;
import java.util.function.Supplier;

/**
 * A class that contains commands that are used during the 15-second autonomous period at the start of each match.
 */
public class AutonomousCommands {
    public static Command getResetPoseToAutoPoseCommand(Supplier<String> autoName) {
        return new InstantCommand(
                () -> {
                    if (DriverStation.isEnabled())
                        return;
                    RobotContainer.POSE_ESTIMATOR.resetPose(getAutoStartPose(autoName.get()));
                }
        );
    }

    public static Pose2d getAutoStartPose(String autoName) {
        try {
            final Pose2d nonMirroredAutoStartPose = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get();
            final MirrorablePose2d mirroredAutoStartPose = new MirrorablePose2d(nonMirroredAutoStartPose, true);
            return mirroredAutoStartPose.get();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }
}
