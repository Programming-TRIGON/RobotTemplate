package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.zonerestrictions;

import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.robot.constants.FieldConstants;

public class ZoneRestrictedDriveConstants {
    private static final double
            ROBOT_X_WIDTH_METERS = 0.836,
            ROBOT_Y_WIDTH_METERS = 0.902,
            INTAKE_LENGTH = 0.198845;
    public static final BoundingBox ROBOT_RELATIVE_BOUNDING_BOX = new BoundingBox(
            new Translation2d(ROBOT_X_WIDTH_METERS / 2 + INTAKE_LENGTH, -ROBOT_Y_WIDTH_METERS / 2),
            new Translation2d(-ROBOT_X_WIDTH_METERS / 2, ROBOT_Y_WIDTH_METERS / 2)
    );
    private static final double
            FIELD_BOUNDARY_MINIMUM_DISTANCE_METERS = 0.1,
            FIELD_BOUNDARY_BRAKING_ZONE_DISTANCE_METERS = 0.3;
    public static final ContainmentZone FIELD_BOUNDARY_ZONE = new ContainmentZone(
            FieldConstants.FIELD_BOUNDING_BOX,
            FIELD_BOUNDARY_MINIMUM_DISTANCE_METERS,
            FIELD_BOUNDARY_BRAKING_ZONE_DISTANCE_METERS
    );
}
