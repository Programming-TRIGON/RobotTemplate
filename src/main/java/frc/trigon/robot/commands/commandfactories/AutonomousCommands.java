package frc.trigon.robot.commands.commandfactories;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import org.json.simple.parser.ParseException;
import org.trigon.utilities.mirrorable.MirrorablePose2d;

import java.io.IOException;

public class AutonomousCommands {
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
