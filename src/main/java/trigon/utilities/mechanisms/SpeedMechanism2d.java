package trigon.utilities.mechanisms;

import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import trigon.utilities.mechanisms.MechanismConstants;

/**
 * A Mechanism2d object to display the current velocity and target velocity of a mechanism.
 */
public class SpeedMechanism2d {
    private final String key;
    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d
            currentVelocityLigament,
            currentVelocityTopArrowLigament,
            currentVelocityBottomArrowLigament,
            targetVelocityLigament,
            targetVelocityTopArrowLigament,
            targetVelocityBottomArrowLigament;
    private final double deadband;

    /**
     * Constructs a SpeedMechanism2d object.
     *
     * @param key                        the key of the mechanism
     * @param maximumDisplayableVelocity the maximum displayable velocity
     */
    public SpeedMechanism2d(String key, double maximumDisplayableVelocity) {
        this(key, maximumDisplayableVelocity, 0.001);
    }

    /**
     * Constructs a SpeedMechanism2d object.
     *
     * @param name                       the name of the mechanism
     * @param maximumDisplayableVelocity the maximum displayable velocity
     */
    public SpeedMechanism2d(String name, double maximumDisplayableVelocity, double deadband) {
        this.deadband = deadband;
        this.key = "Mechanisms/" + name;
        this.mechanism = new LoggedMechanism2d(2 * maximumDisplayableVelocity, 2 * maximumDisplayableVelocity);
        final LoggedMechanismRoot2d root = mechanism.getRoot("Root", maximumDisplayableVelocity, maximumDisplayableVelocity);
        this.currentVelocityLigament = root.append(new LoggedMechanismLigament2d("ZCurrentVelocityLigament", 0, 0, trigon.utilities.mechanisms.MechanismConstants.MECHANISM_LINE_WIDTH, trigon.utilities.mechanisms.MechanismConstants.BLUE));
        this.currentVelocityTopArrowLigament = currentVelocityLigament.append(new LoggedMechanismLigament2d("ZCurrentVelocityTopArrowLigament", trigon.utilities.mechanisms.MechanismConstants.ARROW_LENGTH_SCALE * maximumDisplayableVelocity, trigon.utilities.mechanisms.MechanismConstants.ZERO_TOP_ANGLE, trigon.utilities.mechanisms.MechanismConstants.MECHANISM_LINE_WIDTH, trigon.utilities.mechanisms.MechanismConstants.BLUE));
        this.currentVelocityBottomArrowLigament = currentVelocityLigament.append(new LoggedMechanismLigament2d("ZCurrentVelocityBottomArrowLigament", trigon.utilities.mechanisms.MechanismConstants.ARROW_LENGTH_SCALE * maximumDisplayableVelocity, trigon.utilities.mechanisms.MechanismConstants.ZERO_BOTTOM_ANGLE, trigon.utilities.mechanisms.MechanismConstants.MECHANISM_LINE_WIDTH, trigon.utilities.mechanisms.MechanismConstants.BLUE));

        this.targetVelocityLigament = root.append(new LoggedMechanismLigament2d("TargetVelocityLigament", 0, 0, trigon.utilities.mechanisms.MechanismConstants.MECHANISM_LINE_WIDTH, trigon.utilities.mechanisms.MechanismConstants.GRAY));
        this.targetVelocityTopArrowLigament = targetVelocityLigament.append(new LoggedMechanismLigament2d("TargetVelocityTopArrowLigament", trigon.utilities.mechanisms.MechanismConstants.ARROW_LENGTH_SCALE * maximumDisplayableVelocity, trigon.utilities.mechanisms.MechanismConstants.ZERO_TOP_ANGLE, trigon.utilities.mechanisms.MechanismConstants.MECHANISM_LINE_WIDTH, trigon.utilities.mechanisms.MechanismConstants.GRAY));
        this.targetVelocityBottomArrowLigament = targetVelocityLigament.append(new LoggedMechanismLigament2d("TargetVelocityBottomArrowLigament", trigon.utilities.mechanisms.MechanismConstants.ARROW_LENGTH_SCALE * maximumDisplayableVelocity, trigon.utilities.mechanisms.MechanismConstants.ZERO_BOTTOM_ANGLE, trigon.utilities.mechanisms.MechanismConstants.MECHANISM_LINE_WIDTH, trigon.utilities.mechanisms.MechanismConstants.GRAY));
    }

    /**
     * Updates the mechanism's velocity and target velocity and logs the Mechanism2d object.
     *
     * @param currentVelocity the current velocity
     * @param targetVelocity  the target velocity
     */
    public void update(double currentVelocity, double targetVelocity) {
        setTargetVelocity(targetVelocity);
        update(currentVelocity);
    }

    /**
     * Updates the mechanism's velocity and logs the Mechanism2d object.
     *
     * @param currentVelocity the current velocity
     */
    public void update(double currentVelocity) {
        setCurrentVelocity(currentVelocity);
        update();
    }

    /**
     * Logs the Mechanism2d object.
     */
    public void update() {
        Logger.recordOutput(key, mechanism);
    }

    /**
     * Sets the current velocity of the mechanism but doesn't log the Mechanism2d object.
     *
     * @param currentVelocity the current velocity
     */
    public void setCurrentVelocity(double currentVelocity) {
        setArrowAngle(currentVelocity, currentVelocityTopArrowLigament, currentVelocityBottomArrowLigament);
        currentVelocityLigament.setLength(currentVelocity);
        setCurrentLigamentColor(velocityToColor(currentVelocity));
    }

    /**
     * Sets the target velocity but doesn't log the Mechanism2d object.
     *
     * @param targetVelocity the target velocity
     */
    public void setTargetVelocity(double targetVelocity) {
        setArrowAngle(targetVelocity, targetVelocityTopArrowLigament, targetVelocityBottomArrowLigament);
        targetVelocityLigament.setLength(targetVelocity);
    }

    private void setCurrentLigamentColor(Color8Bit color) {
        currentVelocityLigament.setColor(color);
        currentVelocityTopArrowLigament.setColor(color);
        currentVelocityBottomArrowLigament.setColor(color);
    }

    private Color8Bit velocityToColor(double velocity) {
        if (velocity > deadband)
            return trigon.utilities.mechanisms.MechanismConstants.GREEN;
        else if (velocity < -deadband)
            return trigon.utilities.mechanisms.MechanismConstants.RED;
        return trigon.utilities.mechanisms.MechanismConstants.BLUE;
    }

    private void setArrowAngle(double velocity, LoggedMechanismLigament2d topLigament, LoggedMechanismLigament2d bottomLigament) {
        if (velocity > deadband) {
            topLigament.setAngle(trigon.utilities.mechanisms.MechanismConstants.POSITIVE_TOP_ANGLE);
            bottomLigament.setAngle(trigon.utilities.mechanisms.MechanismConstants.POSITIVE_BOTTOM_ANGLE);
        } else if (velocity < -deadband) {
            topLigament.setAngle(trigon.utilities.mechanisms.MechanismConstants.NEGATIVE_TOP_ANGLE);
            bottomLigament.setAngle(trigon.utilities.mechanisms.MechanismConstants.NEGATIVE_BOTTOM_ANGLE);
        } else {
            topLigament.setAngle(trigon.utilities.mechanisms.MechanismConstants.ZERO_TOP_ANGLE);
            bottomLigament.setAngle(MechanismConstants.ZERO_BOTTOM_ANGLE);
        }
    }
}