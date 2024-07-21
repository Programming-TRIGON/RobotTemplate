package frc.trigon.robot.utilities.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/**
 * A Mechanism2d object to display the current angle and the target angle of a double jointed arm.
 */
public class DoubleJointedArmMechanism2d {
    private final String key;
    private final Mechanism2d mechanism;
    private final MechanismLigament2d
            currentPositionFirstLigament,
            currentPositionSecondLigament,
            targetPositionFirstLigament,
            targetPositionSecondLigament;

    public DoubleJointedArmMechanism2d(String key, Color8Bit mechanismColor) {
        this(key, MechanismConstants.MECHANISM_LINE_LENGTH, MechanismConstants.MECHANISM_LINE_LENGTH, mechanismColor);
    }

    public DoubleJointedArmMechanism2d(String key, double firstJointLength, double secondJointLength, Color8Bit mechanismColor) {
        this.key = key;
        this.mechanism = new Mechanism2d(2 * MechanismConstants.MECHANISM_WIDTH_RATIO * (firstJointLength + firstJointLength), 2 * MechanismConstants.MECHANISM_WIDTH_RATIO * (firstJointLength + secondJointLength));
        MechanismRoot2d root = mechanism.getRoot("AngleRoot", MechanismConstants.MECHANISM_WIDTH_RATIO * (firstJointLength + firstJointLength), MechanismConstants.MECHANISM_WIDTH_RATIO * (firstJointLength + secondJointLength));
        this.currentPositionFirstLigament = root.append(new MechanismLigament2d("ZCurrentPositionFirstLigament", firstJointLength, 0, MechanismConstants.MECHANISM_LINE_WIDTH, mechanismColor));
        this.currentPositionSecondLigament = currentPositionFirstLigament.append(new MechanismLigament2d("ZCurrentPositionSecondLigament", secondJointLength, 0, MechanismConstants.MECHANISM_LINE_WIDTH, mechanismColor));
        this.targetPositionFirstLigament = root.append(new MechanismLigament2d("TargetPositionFirstLigament", firstJointLength, 0, MechanismConstants.MECHANISM_LINE_WIDTH, MechanismConstants.GRAY));
        this.targetPositionSecondLigament = targetPositionFirstLigament.append(new MechanismLigament2d("TargetPositionSecondLigament", secondJointLength, 0, MechanismConstants.MECHANISM_LINE_WIDTH, MechanismConstants.GRAY));
    }

    /**
     * Updates the angle and target angle of both joints, then logs the Mechanism2d object.
     *
     * @param firstJointCurrentAngle  the current angle of the first joint
     * @param firstJointTargetAngle   the target angle of the first joint
     * @param secondJointCurrentAngle the current angle of the second joint
     * @param secondJointTargetAngle  the target angle of the second joint
     */
    public void update(Rotation2d firstJointCurrentAngle, Rotation2d firstJointTargetAngle, Rotation2d secondJointCurrentAngle, Rotation2d secondJointTargetAngle) {
        setTargetAngle(firstJointTargetAngle, secondJointTargetAngle);
        update(firstJointCurrentAngle, secondJointCurrentAngle);
    }

    /**
     * Updates the angle of both joints, then logs the Mechanism2d object.
     *
     * @param firstJointCurrentAngle  the current angle of the first joint
     * @param secondJointCurrentAngle the current angle of the second joint
     */
    public void update(Rotation2d firstJointCurrentAngle, Rotation2d secondJointCurrentAngle) {
        setCurrentAngle(firstJointCurrentAngle, secondJointCurrentAngle);
        update();
    }

    /**
     * Updates the angle and target angle of the first joint, then logs the Mechanism2d object.
     *
     * @param firstJointCurrentAngle the current angle of the first joint
     * @param firstJointTargetAngle  the target angle of the first joint
     */
    public void updateFirstJoint(Rotation2d firstJointCurrentAngle, Rotation2d firstJointTargetAngle) {
        setFirstJoint(firstJointCurrentAngle, firstJointTargetAngle);
        update();
    }

    /**
     * Updates the angle and target angle of the second joint, then logs the Mechanism2d object.
     *
     * @param secondJointCurrentAngle the current angle of the second joint
     * @param secondJointTargetAngle  the target angle of the second joint
     */
    public void updateSecondJoint(Rotation2d secondJointCurrentAngle, Rotation2d secondJointTargetAngle) {
        setSecondJoint(secondJointCurrentAngle, secondJointTargetAngle);
        update();
    }

    /**
     * Logs the Mechanism2d object.
     */
    public void update() {
        Logger.recordOutput(key, mechanism);
    }

    /**
     * Updates the angle of both joints, but doesn't log the Mechanism2d object.
     *
     * @param firstJointCurrentAngle  the current angle of the first joint
     * @param secondJointCurrentAngle the current angle of the second joint
     */
    public void setCurrentAngle(Rotation2d firstJointCurrentAngle, Rotation2d secondJointCurrentAngle) {
        setFirstJointCurrentAngle(firstJointCurrentAngle);
        setSecondJointCurrentAngle(secondJointCurrentAngle);
    }

    /**
     * Sets the target angle of both joints, but doesn't log the mechanism.
     *
     * @param firstJointTargetAngle  the target angle of the first joint
     * @param secondJointTargetAngle the target angle of the second joint
     */
    public void setTargetAngle(Rotation2d firstJointTargetAngle, Rotation2d secondJointTargetAngle) {
        setFirstJointTargetAngle(firstJointTargetAngle);
        setSecondJointTargetAngle(secondJointTargetAngle);
    }

    /**
     * Updates the angle and target angle of the first joint, but doesn't log the Mechanism2d object.
     *
     * @param firstJointCurrentAngle the current angle of the first joint
     * @param firstJointTargetAngle  the target angle of the first joint
     */
    public void setFirstJoint(Rotation2d firstJointCurrentAngle, Rotation2d firstJointTargetAngle) {
        setFirstJointCurrentAngle(firstJointCurrentAngle);
        setFirstJointTargetAngle(firstJointTargetAngle);
    }

    /**
     * Updates the angle and target angle of the second joint, but doesn't log the Mechanism2d object.
     *
     * @param secondJointCurrentAngle the current angle of the second joint
     * @param secondJointTargetAngle  the target angle of the second joint
     */
    public void setSecondJoint(Rotation2d secondJointCurrentAngle, Rotation2d secondJointTargetAngle) {
        setSecondJointCurrentAngle(secondJointCurrentAngle);
        setSecondJointTargetAngle(secondJointTargetAngle);
    }

    /**
     * Updates the angle of the first joint, but doesn't log the Mechanism2d object.
     *
     * @param firstJointCurrentAngle the current angle of the first joint
     */
    public void setFirstJointCurrentAngle(Rotation2d firstJointCurrentAngle) {
        currentPositionFirstLigament.setAngle(firstJointCurrentAngle);
    }

    /**
     * Sets the target angle of the first joint.
     *
     * @param firstJointTargetAngle the target angle of the first joint
     */
    public void setFirstJointTargetAngle(Rotation2d firstJointTargetAngle) {
        targetPositionFirstLigament.setAngle(firstJointTargetAngle);
    }

    /**
     * Updates the angle of the second joint but doesn't log the Mechanism2d object.
     *
     * @param secondJointCurrentAngle the current angle of the second joint
     */
    public void setSecondJointCurrentAngle(Rotation2d secondJointCurrentAngle) {
        currentPositionSecondLigament.setAngle(secondJointCurrentAngle);
    }

    /**
     * Sets the target angle of the second joint.
     *
     * @param secondJointTargetAngle the target angle of the second joint
     */
    public void setSecondJointTargetAngle(Rotation2d secondJointTargetAngle) {
        targetPositionSecondLigament.setAngle(secondJointTargetAngle);
    }
}
