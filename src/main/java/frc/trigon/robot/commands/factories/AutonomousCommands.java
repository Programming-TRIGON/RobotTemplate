package frc.trigon.robot.commands.factories;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.utilities.mirrorable.MirrorablePose2d;

import java.util.Optional;
import java.util.function.Supplier;

public class AutonomousCommands {
    public static Command getResetPoseToAutoPoseCommand(Supplier<String> pathName) {
        return new InstantCommand(
                () -> {
                    if (DriverStation.isEnabled())
                        return;
                    final Pose2d autoStartPose = PathPlannerAuto.getStaringPoseFromAutoFile(pathName.get());
                    RobotContainer.POSE_ESTIMATOR.resetPose(new MirrorablePose2d(autoStartPose, true).get());
                }
        ).ignoringDisable(true);
    }

    private static void overrideRotation(Supplier<Optional<Rotation2d>> rotationOverride) {
        PPHolonomicDriveController.setRotationTargetOverride(rotationOverride);
    }
}
