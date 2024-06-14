package frc.trigon.robot.utilities.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/**
 * A Mechanism2d object to display the current angle and the target angle of a single jointed arm.
 */
public class SingleJointedArmMechanism2d {
    private final String key;
    private final Mechanism2d mechanism;
    private final MechanismLigament2d
            currentPositionLigament,
            targetPositionLigament;

    public SingleJointedArmMechanism2d(String key) {
        this(key, MechanismConstants.BLUE);
    }

    public SingleJointedArmMechanism2d(String key, Color8Bit mechanismColor) {
        this.key = key;
        this.mechanism = new Mechanism2d(2 * MechanismConstants.MECHANISM_LINE_LENGTH, 2 * MechanismConstants.MECHANISM_LINE_LENGTH);
        MechanismRoot2d root = mechanism.getRoot("AngleRoot", MechanismConstants.MECHANISM_LINE_LENGTH, MechanismConstants.MECHANISM_LINE_LENGTH);
        this.currentPositionLigament = root.append(new MechanismLigament2d("ZCurrentPositionLigament", MechanismConstants.MECHANISM_LINE_LENGTH, 0, MechanismConstants.MECHANISM_LINE_WIDTH, mechanismColor));
        this.targetPositionLigament = root.append(new MechanismLigament2d("TargetPositionLigament", MechanismConstants.MECHANISM_LINE_LENGTH, 0, MechanismConstants.MECHANISM_LINE_WIDTH, MechanismConstants.GRAY));
    }

    /**
     * Updates the mechanism's angle and target angle, then logs the Mechanism2d object.
     *
     * @param currentAngle the current angle
     * @param targetAngle  the target angle
     */
    public void updateMechanism(Rotation2d currentAngle, Rotation2d targetAngle) {
        updateMechanism(currentAngle.getDegrees(), targetAngle.getDegrees());
    }

    /**
     * Updates the mechanism's angle and target angle, then logs the Mechanism2d object.
     *
     * @param currentAngleDegrees the current angle in degrees
     * @param targetAngleDegrees  the target angle in degrees
     */
    public void updateMechanism(double currentAngleDegrees, double targetAngleDegrees) {
        setTargetAngle(targetAngleDegrees);
        updateMechanism(currentAngleDegrees);
    }

    /**
     * Updates the mechanism's angle, then logs the Mechanism2d object.
     *
     * @param currentAngle the current angle
     */
    public void updateMechanism(Rotation2d currentAngle) {
        updateMechanism(currentAngle.getDegrees());
    }

    /**
     * Updates the mechanism's angle, then logs the Mechanism2d object.
     *
     * @param currentAngleDegrees the current angle in degrees
     */
    public void updateMechanism(double currentAngleDegrees) {
        currentPositionLigament.setAngle(currentAngleDegrees);
        Logger.recordOutput(key, mechanism);
    }

    /**
     * Sets the target angle of the mechanism.
     *
     * @param targetAngle the target angle
     */
    public void setTargetAngle(Rotation2d targetAngle) {
        setTargetAngle(targetAngle.getDegrees());
    }

    /**
     * Sets the target angle of the mechanism.
     *
     * @param targetAngleDegrees the target angle in degrees
     */
    public void setTargetAngle(double targetAngleDegrees) {
        targetPositionLigament.setAngle(targetAngleDegrees);
    }
}
