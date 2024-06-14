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

    public void updateMechanism(Rotation2d firstJointCurrentAngle, Rotation2d firstJointTargetAngle, Rotation2d secondJointCurrentAngle, Rotation2d secondJointTargetAngle) {
        updateMechanism(firstJointCurrentAngle.getDegrees(), firstJointTargetAngle.getDegrees(), secondJointCurrentAngle.getDegrees(), secondJointTargetAngle.getDegrees());
    }

    public void updateMechanism(double firstJointCurrentAngleDegrees, double firstJointTargetAngleDegrees, double secondJointCurrentAngleDegrees, double secondJointTargetAngleDegrees) {
        setTargetAngle(firstJointTargetAngleDegrees, secondJointTargetAngleDegrees);
        updateMechanism(firstJointCurrentAngleDegrees, secondJointCurrentAngleDegrees);
    }

    public void updateMechanism(Rotation2d firstJointCurrentAngle, Rotation2d secondJointCurrentAngle) {
        updateMechanism(firstJointCurrentAngle.getDegrees(), secondJointCurrentAngle.getDegrees());
    }

    public void updateMechanism(double firstJointCurrentAngleDegrees, double secondJointCurrentAngleDegrees) {
        updateFirstJoint(firstJointCurrentAngleDegrees);
        updateSecondJoint(secondJointCurrentAngleDegrees);
        Logger.recordOutput(key, mechanism);
    }

    public void setTargetAngle(Rotation2d firstJointTargetAngle, Rotation2d secondJointTargetAngle) {
        setTargetAngle(firstJointTargetAngle.getDegrees(), secondJointTargetAngle.getDegrees());
    }

    public void setTargetAngle(double firstJointTargetAngleDegrees, double secondJointTargetAngleDegrees) {
        setFirstJointTargetAngle(firstJointTargetAngleDegrees);
        setSecondJointTargetAngle(secondJointTargetAngleDegrees);
    }

    public void updateFirstJoint(Rotation2d firstJointCurrentAngle, Rotation2d firstJointTargetAngle) {
        updateFirstJoint(firstJointCurrentAngle.getDegrees(), firstJointTargetAngle.getDegrees());
    }

    public void updateFirstJoint(double firstJointCurrentAngleDegrees, double firstJointTargetAngleDegrees) {
        updateFirstJoint(firstJointCurrentAngleDegrees);
        setFirstJointTargetAngle(firstJointTargetAngleDegrees);
    }

    public void updateFirstJoint(Rotation2d firstJointCurrentAngle) {
        updateFirstJoint(firstJointCurrentAngle.getDegrees());
    }

    public void updateFirstJoint(double firstJointCurrentAngleDegrees) {
        currentPositionFirstLigament.setAngle(firstJointCurrentAngleDegrees);
        Logger.recordOutput(key, mechanism);
    }

    public void setFirstJointTargetAngle(Rotation2d firstJointTargetAngle) {
        setFirstJointTargetAngle(firstJointTargetAngle.getDegrees());
    }

    public void setFirstJointTargetAngle(double firstJointTargetAngleDegrees) {
        targetPositionFirstLigament.setAngle(firstJointTargetAngleDegrees);
    }

    public void updateSecondJoint(Rotation2d secondJointCurrentAngle, Rotation2d secondJointTargetAngle) {
        updateSecondJoint(secondJointCurrentAngle.getDegrees(), secondJointTargetAngle.getDegrees());
    }

    public void updateSecondJoint(double secondJointCurrentAngleDegrees, double secondJointTargetAngleDegrees) {
        setSecondJointTargetAngle(secondJointTargetAngleDegrees);
        updateSecondJoint(secondJointCurrentAngleDegrees);
    }

    private void updateSecondJoint(Rotation2d secondJointCurrentAngle) {
        updateSecondJoint(secondJointCurrentAngle.getDegrees());
    }

    private void updateSecondJoint(double secondJointCurrentAngleDegrees) {
        currentPositionSecondLigament.setAngle(secondJointCurrentAngleDegrees);
        Logger.recordOutput(key, mechanism);
    }

    public void setSecondJointTargetAngle(Rotation2d secondJointTargetAngle) {
        setSecondJointTargetAngle(secondJointTargetAngle.getDegrees());
    }

    public void setSecondJointTargetAngle(double secondJointTargetAngleDegrees) {
        targetPositionSecondLigament.setAngle(secondJointTargetAngleDegrees);
    }
}
