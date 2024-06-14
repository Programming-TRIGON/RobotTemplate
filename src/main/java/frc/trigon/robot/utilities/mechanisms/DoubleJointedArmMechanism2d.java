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

    public DoubleJointedArmMechanism2d(String key) {
        this(key, MechanismConstants.BLUE);
    }

    private DoubleJointedArmMechanism2d(String key, Color8Bit mechanismColor) {
        this.key = key;
        this.mechanism = new Mechanism2d(4 * MechanismConstants.MECHANISM_LINE_LENGTH, 4 * MechanismConstants.MECHANISM_LINE_LENGTH);
        MechanismRoot2d root = mechanism.getRoot("AngleRoot", 2 * MechanismConstants.MECHANISM_LINE_LENGTH, 2 * MechanismConstants.MECHANISM_LINE_LENGTH);
        this.currentPositionFirstLigament = root.append(new MechanismLigament2d("ZCurrentPositionFirstLigament", MechanismConstants.MECHANISM_LINE_LENGTH, 0, MechanismConstants.MECHANISM_LINE_WIDTH, mechanismColor));
        this.currentPositionSecondLigament = currentPositionFirstLigament.append(new MechanismLigament2d("ZCurrentPositionSecondLigament", MechanismConstants.MECHANISM_LINE_LENGTH, 0, MechanismConstants.MECHANISM_LINE_WIDTH, mechanismColor));
        this.targetPositionFirstLigament = root.append(new MechanismLigament2d("TargetPositionFirstLigament", MechanismConstants.MECHANISM_LINE_LENGTH, 0, MechanismConstants.MECHANISM_LINE_WIDTH, MechanismConstants.GRAY));
        this.targetPositionSecondLigament = targetPositionFirstLigament.append(new MechanismLigament2d("TargetPositionSecondLigament", MechanismConstants.MECHANISM_LINE_LENGTH, 0, MechanismConstants.MECHANISM_LINE_WIDTH, MechanismConstants.GRAY));
    }

    /**
     * Updates the angle and target angle of both joints, then logs the Mechanism2d object.
     *
     * @param firstJointCurrentAngle  the current angle of the first joint
     * @param firstJointTargetAngle   the target angle of the first joint
     * @param secondJointCurrentAngle the current angle of the second joint
     * @param secondJointTargetAngle  the target angle of the second joint
     */
    public void updateMechanism(Rotation2d firstJointCurrentAngle, Rotation2d firstJointTargetAngle, Rotation2d secondJointCurrentAngle, Rotation2d secondJointTargetAngle) {
        updateMechanism(firstJointCurrentAngle.getDegrees(), firstJointTargetAngle.getDegrees(), secondJointCurrentAngle.getDegrees(), secondJointTargetAngle.getDegrees());
    }

    /**
     * Updates the angle and target angle of both joints, then logs the Mechanism2d object.
     *
     * @param firstJointCurrentAngleDegrees  the current angle of the first joint in degrees
     * @param firstJointTargetAngleDegrees   the target angle of the first joint in degrees
     * @param secondJointCurrentAngleDegrees the current angle of the second joint in degrees
     * @param secondJointTargetAngleDegrees  the target angle of the second joint in degrees
     */
    public void updateMechanism(double firstJointCurrentAngleDegrees, double firstJointTargetAngleDegrees, double secondJointCurrentAngleDegrees, double secondJointTargetAngleDegrees) {
        setTargetAngle(firstJointTargetAngleDegrees, secondJointTargetAngleDegrees);
        updateMechanism(firstJointCurrentAngleDegrees, secondJointCurrentAngleDegrees);
    }

    /**
     * Updates the angle of both joints, then logs the Mechanism2d object.
     *
     * @param firstJointCurrentAngle  the current angle of the first joint
     * @param secondJointCurrentAngle the current angle of the second joint
     */
    public void updateMechanism(Rotation2d firstJointCurrentAngle, Rotation2d secondJointCurrentAngle) {
        updateMechanism(firstJointCurrentAngle.getDegrees(), secondJointCurrentAngle.getDegrees());
    }

    /**
     * Updates the angle of both joints, then logs the Mechanism2d object.
     *
     * @param firstJointCurrentAngleDegrees  the current angle of the first joint in degrees
     * @param secondJointCurrentAngleDegrees the current angle of the second joint in degrees
     */
    public void updateMechanism(double firstJointCurrentAngleDegrees, double secondJointCurrentAngleDegrees) {
        updateFirstJoint(firstJointCurrentAngleDegrees);
        updateSecondJoint(secondJointCurrentAngleDegrees);
        Logger.recordOutput(key, mechanism);
    }

    /**
     * Sets the target angle of both joints.
     *
     * @param firstJointTargetAngle  the target angle of the first joint
     * @param secondJointTargetAngle the target angle of the second joint
     */
    public void setTargetAngle(Rotation2d firstJointTargetAngle, Rotation2d secondJointTargetAngle) {
        setTargetAngle(firstJointTargetAngle.getDegrees(), secondJointTargetAngle.getDegrees());
    }

    /**
     * Sets the target angle of both joints.
     *
     * @param firstJointTargetAngleDegrees  the target angle of the first joint in degrees
     * @param secondJointTargetAngleDegrees the target angle of the second joint in degrees
     */
    public void setTargetAngle(double firstJointTargetAngleDegrees, double secondJointTargetAngleDegrees) {
        setFirstJointTargetAngle(firstJointTargetAngleDegrees);
        setSecondJointTargetAngle(secondJointTargetAngleDegrees);
    }

    /**
     * Updates the angle and target angle of the first joint, then logs the Mechanism2d object.
     *
     * @param firstJointCurrentAngle the current angle of the first joint
     * @param firstJointTargetAngle  the target angle of the first joint
     */
    public void updateFirstJoint(Rotation2d firstJointCurrentAngle, Rotation2d firstJointTargetAngle) {
        updateFirstJoint(firstJointCurrentAngle.getDegrees(), firstJointTargetAngle.getDegrees());
    }

    /**
     * Updates the angle and target angle of the first joint, then logs the Mechanism2d object.
     *
     * @param firstJointCurrentAngleDegrees the current angle of the first joint in degrees
     * @param firstJointTargetAngleDegrees  the target angle of the first joint in degrees
     */
    public void updateFirstJoint(double firstJointCurrentAngleDegrees, double firstJointTargetAngleDegrees) {
        updateFirstJoint(firstJointCurrentAngleDegrees);
        setFirstJointTargetAngle(firstJointTargetAngleDegrees);
    }

    /**
     * Updates the angle of the first joint, then logs the Mechanism2d object.
     *
     * @param firstJointCurrentAngle the current angle of the first joint
     */
    public void updateFirstJoint(Rotation2d firstJointCurrentAngle) {
        updateFirstJoint(firstJointCurrentAngle.getDegrees());
    }

    /**
     * Updates the angle of the first joint, then logs the Mechanism2d object.
     *
     * @param firstJointCurrentAngleDegrees the current angle of the first joint in degrees
     */
    public void updateFirstJoint(double firstJointCurrentAngleDegrees) {
        currentPositionFirstLigament.setAngle(firstJointCurrentAngleDegrees);
        Logger.recordOutput(key, mechanism);
    }

    /**
     * Sets the target angle of the first joint.
     *
     * @param firstJointTargetAngle the target angle of the first joint
     */
    public void setFirstJointTargetAngle(Rotation2d firstJointTargetAngle) {
        setFirstJointTargetAngle(firstJointTargetAngle.getDegrees());
    }

    /**
     * Sets the target angle of the first joint.
     *
     * @param firstJointTargetAngleDegrees the target angle of the first joint in degrees
     */
    public void setFirstJointTargetAngle(double firstJointTargetAngleDegrees) {
        targetPositionFirstLigament.setAngle(firstJointTargetAngleDegrees);
    }

    /**
     * Updates the angle and target angle of the second joint, then logs the Mechanism2d object.
     *
     * @param secondJointCurrentAngle the current angle of the second joint
     * @param secondJointTargetAngle  the target angle of the second joint
     */
    public void updateSecondJoint(Rotation2d secondJointCurrentAngle, Rotation2d secondJointTargetAngle) {
        updateSecondJoint(secondJointCurrentAngle.getDegrees(), secondJointTargetAngle.getDegrees());
    }

    /**
     * Updates the angle and target angle of the second joint, then logs the Mechanism2d object.
     *
     * @param secondJointCurrentAngleDegrees the current angle of the second joint in degrees
     * @param secondJointTargetAngleDegrees  the target angle of the second joint in degrees
     */
    public void updateSecondJoint(double secondJointCurrentAngleDegrees, double secondJointTargetAngleDegrees) {
        setSecondJointTargetAngle(secondJointTargetAngleDegrees);
        updateSecondJoint(secondJointCurrentAngleDegrees);
    }

    /**
     * Updates the angle of the second joint, then logs the Mechanism2d object.
     *
     * @param secondJointCurrentAngle the current angle of the second joint
     */
    private void updateSecondJoint(Rotation2d secondJointCurrentAngle) {
        updateSecondJoint(secondJointCurrentAngle.getDegrees());
    }

    /**
     * Updates the angle of the second joint, then logs the Mechanism2d object.
     *
     * @param secondJointCurrentAngleDegrees the current angle of the second joint in degrees
     */
    private void updateSecondJoint(double secondJointCurrentAngleDegrees) {
        currentPositionSecondLigament.setAngle(secondJointCurrentAngleDegrees);
        Logger.recordOutput(key, mechanism);
    }

    /**
     * Sets the target angle of the second joint.
     *
     * @param secondJointTargetAngle the target angle of the second joint
     */
    public void setSecondJointTargetAngle(Rotation2d secondJointTargetAngle) {
        setSecondJointTargetAngle(secondJointTargetAngle.getDegrees());
    }

    /**
     * Sets the target angle of the second joint.
     *
     * @param secondJointTargetAngleDegrees the target angle of the second joint in degrees
     */
    public void setSecondJointTargetAngle(double secondJointTargetAngleDegrees) {
        targetPositionSecondLigament.setAngle(secondJointTargetAngleDegrees);
    }
}
