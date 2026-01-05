package frc.trigon.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.lib.commands.CameraPositionCalculationCommand;
import frc.trigon.lib.commands.WheelRadiusCharacterizationCommand;
import frc.trigon.lib.hardware.misc.XboxController;
import frc.trigon.lib.utilities.flippable.FlippableRotation2d;

/**
 * A class that contains commands that only use parameters and don't require logic.
 * These are different from {@link GeneralCommands} and command factories because they don't contain any logic, and only run an existing command with parameters.
 * For example, the static LED command always changes all the LED strips to the same LED mode with the same settings.
 */
public class CommandConstants {
    private static final XboxController DRIVER_CONTROLLER = OperatorConstants.DRIVER_CONTROLLER;
    private static final double
            MINIMUM_TRANSLATION_SHIFT_POWER = 0.30,
            MINIMUM_ROTATION_SHIFT_POWER = 0.4;
    private static final double JOYSTICK_ORIENTED_ROTATION_DEADBAND = 0.07;

    public static final Command
            RESET_HEADING_COMMAND = new InstantCommand(RobotContainer.ROBOT_POSE_ESTIMATOR::resetHeading),
            SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND = SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                    () -> getXPowerFromPov(DRIVER_CONTROLLER.getPov()) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(MINIMUM_TRANSLATION_SHIFT_POWER),
                    () -> getYPowerFromPov(DRIVER_CONTROLLER.getPov()) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(MINIMUM_TRANSLATION_SHIFT_POWER),
                    () -> 0
            ),
            FIELD_RELATIVE_DRIVE_WITH_JOYSTICK_ORIENTED_ROTATION_COMMAND = SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                    () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftY()),
                    () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftX()),
                    CommandConstants::calculateTargetHeadingFromJoystickAngle
            ),
            SELF_RELATIVE_DRIVE_COMMAND = SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                    () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftY()),
                    () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftX()),
                    () -> calculateRotationStickAxisValue(DRIVER_CONTROLLER.getRightX())
            ),
            WHEEL_RADIUS_CHARACTERIZATION_COMMAND = new WheelRadiusCharacterizationCommand(
                    AutonomousConstants.ROBOT_CONFIG.moduleLocations,
                    RobotContainer.SWERVE::getDriveWheelPositionsRadians,
                    () -> RobotContainer.SWERVE.getHeading().getRadians(),
                    (omegaRadiansPerSecond) -> RobotContainer.SWERVE.selfRelativeDrive(new ChassisSpeeds(0, 0, omegaRadiansPerSecond)),
                    RobotContainer.SWERVE
            ),
            CALCULATE_CAMERA_POSITION_COMMAND = new CameraPositionCalculationCommand(
                    RobotContainer.ROBOT_POSE_ESTIMATOR::getEstimatedRobotPose,
                    Rotation2d.fromDegrees(0),
                    (omegaRadiansPerSecond) -> RobotContainer.SWERVE.selfRelativeDrive(new ChassisSpeeds(0, 0, omegaRadiansPerSecond)),
                    RobotContainer.SWERVE
            );

    /**
     * Calculates the target drive power from an axis value by dividing it by the shift mode value.
     *
     * @param axisValue the stick's value
     * @return the drive power
     */
    public static double calculateDriveStickAxisValue(double axisValue) {
        return axisValue / OperatorConstants.TRANSLATION_STICK_SPEED_DIVIDER / calculateShiftModeValue(MINIMUM_TRANSLATION_SHIFT_POWER);
    }

    /**
     * Calculates the target rotation power from an axis value by dividing it by the shift mode value.
     *
     * @param axisValue the stick's value
     * @return the rotation power
     */
    public static double calculateRotationStickAxisValue(double axisValue) {
        return axisValue / OperatorConstants.ROTATION_STICK_SPEED_DIVIDER / calculateShiftModeValue(MINIMUM_ROTATION_SHIFT_POWER);
    }

    /**
     * The shift mode is a mode of the robot that slows down the robot relative to how much the right trigger axis is pressed.
     * This method will take the given power, and slow it down relative to how much the right trigger is pressed.
     *
     * @param minimumPower the minimum amount of power the shift mode can limit (as an absolute number)
     * @return the power to apply to the robot
     */
    public static double calculateShiftModeValue(double minimumPower) {
        final double squaredShiftModeValue = Math.sqrt(DRIVER_CONTROLLER.getRightTriggerAxis());
        final double minimumShiftValueCoefficient = 1 - (1 / minimumPower);

        return 1 - squaredShiftModeValue * minimumShiftValueCoefficient;
    }

    /**
     * Calculates the target rotation value from the joystick's angle. Used for joystick oriented rotation.
     * Joystick oriented rotation is when the robot rotates directly to the joystick's angle.
     *
     * @return the target heading
     */
    private static FlippableRotation2d calculateTargetHeadingFromJoystickAngle() {
        final double
                xPower = DRIVER_CONTROLLER.getRightX(),
                yPower = DRIVER_CONTROLLER.getRightY();

        final double joystickPower = Math.hypot(xPower, yPower);
        if (joystickPower < JOYSTICK_ORIENTED_ROTATION_DEADBAND)
            return null;

        return new FlippableRotation2d(Math.atan2(xPower, yPower), true);
    }

    private static double getXPowerFromPov(double pov) {
        final double povRadians = Units.degreesToRadians(pov);
        return Math.cos(povRadians);
    }

    private static double getYPowerFromPov(double pov) {
        final double povRadians = Units.degreesToRadians(pov);
        return Math.sin(-povRadians);
    }
}