package frc.robot.subsystems.TelescopicArm;

import org.littletonrobotics.junction.AutoLog;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.config.TelescopicArmConfig;

public interface TelescopicArmIO {
    @AutoLog
    public class TelescopicArmData {

        public Rotation2d angle = Rotation2d.fromDegrees(0);
        public double angularVelocityRadPS = 0.0;
        public double angularAccelRadPSS = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    @AutoLog
    public class ElevatorData {
        public Translation2d position = TelescopicArmConfig.ElevatorSpecs.MOUNT_OFFSET;
        public double velocityMPS = 0;
        public double accelMPSS = 0;
        public double leftCurrentAmps = 0;
        public double rightCurrentAmps = 0;
        public double leftAppliedVolts = 0;
        public double rightAppliedVolts = 0;
    }

    public default void setElevatorVoltage(double volts) {
    };

    public default void setArmVoltage(double volts) {
    };

    public default void setMotorIdleMode(IdleMode idleMode) {
    };

    public default void updateData() {
    };
}