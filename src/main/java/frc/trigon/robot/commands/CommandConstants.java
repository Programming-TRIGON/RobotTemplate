package frc.trigon.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.trigon.hardware.misc.XboxController;
import org.trigon.utilities.mirrorable.MirrorablePose2d;

import java.awt.*;

public class CommandConstants {
    private static final XboxController DRIVER_CONTROLLER = OperatorConstants.DRIVER_CONTROLLER;
    private static final double
            MINIMUM_TRANSLATION_SHIFT_POWER = 0.18,
            MINIMUM_ROTATION_SHIFT_POWER = 0.3;

    public static final Command
            FIELD_RELATIVE_DRIVE_COMMAND = SwerveCommands.getOpenLoopFieldRelativeDriveCommand(
            () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftX()),
            () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftY()),
            () -> calculateRotationStickAxisValue(DRIVER_CONTROLLER.getRightX())
    ),
            SELF_RELATIVE_DRIVE_COMMAND = SwerveCommands.getOpenLoopSelfRelativeDriveCommand(
                    () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftX()),
                    () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftY()),
                    () -> calculateRotationStickAxisValue(DRIVER_CONTROLLER.getRightX())
            ),
            RESET_HEADING_COMMAND = new InstantCommand(() -> RobotContainer.POSE_ESTIMATOR.resetPose(changeRotation(new MirrorablePose2d(RobotContainer.POSE_ESTIMATOR.getCurrentPose(), false), new Rotation2d()).get())),
            SET_GYRO_HEADING_TO_SOLVE_PNP_HEADING_COMMAND = new InstantCommand(RobotContainer.POSE_ESTIMATOR::setGyroHeadingToBestSolvePNPHeading).ignoringDisable(true),
            SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND = SwerveCommands.getOpenLoopSelfRelativeDriveCommand(
                    () -> getXPowerFromPov(DRIVER_CONTROLLER.getPov()) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(MINIMUM_TRANSLATION_SHIFT_POWER),
                    () -> getYPowerFromPov(DRIVER_CONTROLLER.getPov()) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(MINIMUM_TRANSLATION_SHIFT_POWER),
                    () -> 0
            ),
            STATIC_WHITE_LED_COLOR_COMMAND = LEDStripCommands.getStaticColorCommand(Color.white, LEDStripConstants.LED_STRIPS);

    public static double calculateDriveStickAxisValue(double axisValue) {
        return axisValue / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(MINIMUM_TRANSLATION_SHIFT_POWER);
    }

    public static double calculateRotationStickAxisValue(double axisValue) {
        return axisValue / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(MINIMUM_ROTATION_SHIFT_POWER);
    }

    /**
     * The shift mode is a mode of the robot that slows down the robot relative to how much the right trigger axis is pressed.
     * This method will take the given power, and slow it down relative to how much the right trigger is pressed.
     *
     * @param minimumPower the minimum amount of power the shift mode can limit (as an absolute number)
     * @return the power to apply to the robot
     */
    public static double calculateShiftModeValue(double minimumPower) {
        final double squaredShiftModeValue = Math.pow(DRIVER_CONTROLLER.getRightTriggerAxis(), 2);
        final double minimumShiftValueCoefficient = 1 - (1 / minimumPower);

        return 1 - squaredShiftModeValue * minimumShiftValueCoefficient;
    }

    private static double getXPowerFromPov(double pov) {
        final double povRadians = Units.degreesToRadians(pov);
        return Math.cos(povRadians);
    }

    private static double getYPowerFromPov(double pov) {
        final double povRadians = Units.degreesToRadians(pov);
        return Math.sin(-povRadians);
    }

    private static MirrorablePose2d changeRotation(MirrorablePose2d pose2d, Rotation2d newRotation) {
        return new MirrorablePose2d(
                pose2d.get().getTranslation(),
                newRotation.getRadians(),
                false
        );
    }
}