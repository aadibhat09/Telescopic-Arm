package frc.robot.subsystems.TelescopicArm;

import org.littletonrobotics.junction.AutoLog;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.config.TelescopicArmConfig;

public interface TelescopicArmIO {
    @AutoLog
    public class TelescopicArmData {

        // ARM DATA
        // public double arm_angle = 0;
        public Rotation2d arm_angle = TelescopicArmConfig.ArmSpecs.START_ANGLE;
        public double arm_angularVelocityRadPS = 0.0;
        public double arm_angularAccelRadPSS = 0.0;
        public double arm_appliedVolts = 0.0;
        public double arm_currentAmps = 0.0;


        // ELEVATOR DATA
        public Translation2d elevator_position = TelescopicArmConfig.ElevatorSpecs.MOUNT_OFFSET;
        public double elevator_velocityMPS = 0;
        public double elevator_accelMPSS = 0;
        public double elevator_currentAmps = 0;
        public double elevator_appliedVolts = 0;
    }

    public default void setElevatorVoltage(double volts) {
    };

    public default void setArmVoltage(double volts) {
    };

    public default void setMotorIdleMode(IdleMode idleMode) {
    };

    public default void updateData(TelescopicArmDataAutoLogged telescopicArmData) {
    };
}