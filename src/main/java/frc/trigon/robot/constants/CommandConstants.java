package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.subsystems.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class CommandConstants {
    private static final PoseEstimator POSE_ESTIMATOR = RobotContainer.POSE_ESTIMATOR;
    private static final XboxController DRIVER_CONTROLLER = OperatorConstants.DRIVER_CONTROLLER;

    private static final Swerve SWERVE = Swerve.getInstance();

    public static final Command
            FIELD_RELATIVE_DRIVE_COMMAND = SwerveCommands.getOpenLoopFieldRelativeDriveCommand(
                    () -> applyShiftModeToPower(DRIVER_CONTROLLER.getLeftY() / OperatorConstants.STICKS_SPEED_DIVIDER, OperatorConstants.MINIMUM_TRANSLATION_SHIFT_POWER),
                    () -> applyShiftModeToPower(DRIVER_CONTROLLER.getLeftX() / OperatorConstants.STICKS_SPEED_DIVIDER, OperatorConstants.MINIMUM_TRANSLATION_SHIFT_POWER),
                    () -> applyShiftModeToPower(DRIVER_CONTROLLER.getRightX() / OperatorConstants.STICKS_SPEED_DIVIDER, OperatorConstants.MINIMUM_ROTATION_SHIFT_POWER),
                    true
            ),
            SELF_RELATIVE_DRIVE_COMMAND = SwerveCommands.getOpenLoopSelfRelativeDriveCommand(
                    () -> applyShiftModeToPower(DRIVER_CONTROLLER.getLeftY() / OperatorConstants.STICKS_SPEED_DIVIDER, OperatorConstants.MINIMUM_TRANSLATION_SHIFT_POWER),
                    () -> applyShiftModeToPower(DRIVER_CONTROLLER.getLeftX() / OperatorConstants.STICKS_SPEED_DIVIDER, OperatorConstants.MINIMUM_TRANSLATION_SHIFT_POWER),
                    () -> applyShiftModeToPower(DRIVER_CONTROLLER.getRightX() / OperatorConstants.STICKS_SPEED_DIVIDER, OperatorConstants.MINIMUM_ROTATION_SHIFT_POWER),
                    true
            ),
            RESET_HEADING_COMMAND = new InstantCommand(() -> POSE_ESTIMATOR.resetPose(changeRotation(POSE_ESTIMATOR.getCurrentPose(), new Rotation2d()))),
            SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND = SwerveCommands.getOpenLoopSelfRelativeDriveCommand(
                    () -> applyShiftModeToPower(getXPowerFromPov(DRIVER_CONTROLLER.getPov()) / OperatorConstants.POV_DIVIDER, OperatorConstants.MINIMUM_TRANSLATION_SHIFT_POWER),
                    () -> applyShiftModeToPower(getYPowerFromPov(DRIVER_CONTROLLER.getPov()) / OperatorConstants.POV_DIVIDER, OperatorConstants.MINIMUM_TRANSLATION_SHIFT_POWER),
                    () -> 0,
                    true
            ),
            BRAKE_SWERVE_COMMAND = new InstantCommand(() -> SWERVE.setBrake(true));

    /**
     * The shift mode is a mode of the robot that slows down the robot relative to how much the right trigger axis is pressed.
     * This method will take the given power, and slow it down relative to how much the right trigger is pressed.
     *
     * @param power        the power to slow down
     * @param minimumPower the minimum amount of power the shift mode can limit (as an absolute number)
     * @return the power to apply to the robot
     */
    public static double applyShiftModeToPower(double power, double minimumPower) {
        if (Math.abs(power) < minimumPower)
            return power;

        final double powerSign = Math.signum(power);
        final double slope = -(1 - minimumPower * powerSign - (1 - power));
        final double shiftModeValue = DRIVER_CONTROLLER.getRightTriggerAxis();

        return slope * shiftModeValue + power;
    }

    private static double getXPowerFromPov(double pov) {
        final double povRadians = Units.degreesToRadians(pov);
        return Math.cos(povRadians);
    }

    private static double getYPowerFromPov(double pov) {
        final double povRadians = Units.degreesToRadians(pov);
        return Math.sin(-povRadians);
    }

    private static Pose2d changeRotation(Pose2d pose2d, Rotation2d newRotation) {
        return new Pose2d(
                pose2d.getTranslation(),
                newRotation
        );
    }
}
