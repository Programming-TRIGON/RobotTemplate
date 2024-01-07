package frc.trigon.robot.motorsimulation;

public class MotorSimulationConfiguration {
    public double conversionFactor = 1;
    public double voltageCompensationSaturation = 12;
    public PIDConfigs pidConfigs = new PIDConfigs();
    public FeedForwardConfigs feedForwardConfigs = new FeedForwardConfigs();
    public MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

    public static class PIDConfigs {
        public double
                kP = 0,
                kI = 0,
                kD = 0;
        public boolean enableContinuousInput = false;
    }

    public static class FeedForwardConfigs {
        public double
                kS = 0,
                kG = 0,
                kV = 0,
                kA = 0;
    }

    public static class MotionMagicConfigs {
        public double
                maxVelocity = 0,
                maxAcceleration = 0;
    }
}
