package frc.trigon.robot.hardware.rev.sparkecnoder;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkAbsoluteEncoder;

public abstract class SparkEncoder {
    public static SparkEncoder createEncoder(CANSparkBase spark) {
        final SparkAbsoluteEncoder absoluteEncoder = spark.getAbsoluteEncoder();
        if (absoluteEncoder != null)
            return new AbsoluteSparkEncoder(absoluteEncoder);
        else
            return new RelativeSparkEncoder(spark.getEncoder());
    }

    protected SparkEncoder() {
        setConversionsFactor(1);
    }

    public abstract double getPositionRevolutions();

    public abstract double getVelocityRevolutionsPerSecond();

    public abstract void setConversionsFactor(double conversionsFactor);
}
