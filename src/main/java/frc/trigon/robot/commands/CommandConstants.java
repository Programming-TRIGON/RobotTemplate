package frc.trigon.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.constants.PathPlannerConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.trigon.commands.WheelRadiusCharacterizationCommand;
import org.trigon.hardware.misc.XboxController;
import org.trigon.hardware.misc.leds.LEDCommands;
import org.trigon.hardware.misc.leds.LEDStrip;
import org.trigon.utilities.mirrorable.MirrorablePose2d;
import org.trigon.utilities.mirrorable.MirrorableRotation2d;

/**
 * A class that contains commands that only use parameters and don't require logic.
 * These are different from {@link GeneralCommands} and command factories because they don't contain any logic, and only run an existing command with parameters.
 * For example, the static LED command always changes all the LED strips to the same LED mode with the same settings.
 */
public class CommandConstants {
    private static final XboxController DRIVER_CONTROLLER = OperatorConstants.DRIVER_CONTROLLER;
    private static final double
            MINIMUM_TRANSLATION_SHIFT_POWER = 0.18,
            MINIMUM_ROTATION_SHIFT_POWER = 0.3;
    private static final double JOYSTICK_ORIENTED_ROTATION_DEADBAND = 0.4;

    public static final Command
            FIELD_RELATIVE_DRIVE_COMMAND = SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
            () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftY()),
            () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftX()),
            () -> calculateRotationStickAxisValue(DRIVER_CONTROLLER.getRightX())
    ),
            FIELD_RELATIVE_DRIVE_WITH_JOYSTICK_ORIENTED_ROTATION_COMMAND = SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                    () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftY()),
                    () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftX()),
                    CommandConstants::calculateJoystickOrientedRotationValue
            ),
            SELF_RELATIVE_DRIVE_COMMAND = SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                    () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftY()),
                    () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftX()),
                    () -> calculateRotationStickAxisValue(DRIVER_CONTROLLER.getRightX())
            ),
            RESET_HEADING_COMMAND = new InstantCommand(() -> RobotContainer.POSE_ESTIMATOR.resetPose(changeRotation(new MirrorablePose2d(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose(), false), new Rotation2d()).get())),
            SET_GYRO_HEADING_TO_SOLVE_PNP_HEADING_COMMAND = new InstantCommand(RobotContainer.POSE_ESTIMATOR::setGyroHeadingToBestSolvePNPHeading).ignoringDisable(true),
            SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND = SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                    () -> getXPowerFromPov(DRIVER_CONTROLLER.getPov()) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(MINIMUM_TRANSLATION_SHIFT_POWER),
                    () -> getYPowerFromPov(DRIVER_CONTROLLER.getPov()) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(MINIMUM_TRANSLATION_SHIFT_POWER),
                    () -> 0
            ),
            STATIC_WHITE_LED_COLOR_COMMAND = LEDCommands.getStaticColorCommand(Color.kWhite, LEDStrip.LED_STRIPS),
            WHEEL_RADIUS_CHARACTERIZATION_COMMAND = new WheelRadiusCharacterizationCommand(
                    PathPlannerConstants.getRobotConfig().moduleLocations,
                    RobotContainer.SWERVE::getDriveWheelPositionsRadians,
                    () -> RobotContainer.SWERVE.getHeading().getRadians(),
                    RobotContainer.SWERVE::runWheelRadiusCharacterization,
                    RobotContainer.SWERVE
            );


    /**
     * Calculates the target drive power from an axis value by dividing it by the shift mode value.
     *
     * @param axisValue the stick's value
     * @return the drive power
     */
    public static double calculateDriveStickAxisValue(double axisValue) {
        return axisValue / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(MINIMUM_TRANSLATION_SHIFT_POWER);
    }

    /**
     * Calculates the target rotation power from an axis value by dividing it by the shift mode value.
     *
     * @param axisValue the stick's value
     * @return the rotation power
     */
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

    /**
     * Calculates the target rotation value from the joystick's angle. Used for joystick oriented rotation.
     *
     * @return the rotation value
     */
    private static MirrorableRotation2d calculateJoystickOrientedRotationValue() {
        if (1 - Math.abs(Math.hypot(DRIVER_CONTROLLER.getRightX(), DRIVER_CONTROLLER.getRightY())) > JOYSTICK_ORIENTED_ROTATION_DEADBAND)
            return null;

        final double targetAngleRadians = Math.atan2(DRIVER_CONTROLLER.getRightX(), DRIVER_CONTROLLER.getRightY());
        return MirrorableRotation2d.fromRadians(targetAngleRadians, false);
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