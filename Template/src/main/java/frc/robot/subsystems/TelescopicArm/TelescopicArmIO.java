package frc.robot.subsystems.TelescopicArm;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;

public interface TelescopicArmIO {
    @AutoLog
    public class TelescopicArmData {

        public Rotation2d angle = Rotation2d.fromDegrees(0);
        public double angularVelocityRadPS = 0.0;
        public double angularAccelRadPSS = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void setVoltage(double volts) {
    };

    public default void setMotorIdleMode(IdleMode idleMode) {
    };

    public default void updateData() {
    };
}