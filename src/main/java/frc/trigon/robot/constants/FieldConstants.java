package frc.trigon.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.io.IOException;
import java.util.HashMap;

public class FieldConstants {
    public static final double
            FIELD_LENGTH_METERS = 16.54175,
            FIELD_WIDTH_METERS = 8.02;

    private static final boolean SHOULD_USE_HOME_TAG_LAYOUT = false;
    private static final Transform3d TAG_OFFSET = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

    static {
        try {
            APRIL_TAG_FIELD_LAYOUT = SHOULD_USE_HOME_TAG_LAYOUT ?
                    AprilTagFieldLayout.loadFromResource("path/to/layout.json") :
                    AprilTagFields.kDefaultField.loadAprilTagLayoutField();//TODO:Switch for year
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = fieldLayoutToTagIdToPoseMap();

    private static HashMap<Integer, Pose3d> fieldLayoutToTagIdToPoseMap() {
        final HashMap<Integer, Pose3d> tagIdToPose = new HashMap<>();
        for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags())
            tagIdToPose.put(aprilTag.ID, aprilTag.pose.transformBy(TAG_OFFSET));
        return tagIdToPose;
    }
}
