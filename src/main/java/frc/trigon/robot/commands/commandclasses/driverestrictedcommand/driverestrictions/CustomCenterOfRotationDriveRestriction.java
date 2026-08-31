package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A drive restriction that changes the robot's center of rotation.
 * Rotates the robot around a robot relative point instead of the chassis center.
 * Useful for rotating around an extended mechanism such as an intake.
 */
public class CustomCenterOfRotationDriveRestriction implements DriveRestriction {
    private final Translation2d centerOfRotation;

    /**
     * A drive restriction that changes the robot's center of rotation.
     *
     * @param centerOfRotation the robot's desired center of rotation as a robot relative point
     */
    public CustomCenterOfRotationDriveRestriction(Translation2d centerOfRotation) {
        this.centerOfRotation = centerOfRotation;
    }

    @Override
    public Translation2d applyCenterOfRotationRestriction(Translation2d targetCenterOfRotation) {
        return centerOfRotation;
    }
}
