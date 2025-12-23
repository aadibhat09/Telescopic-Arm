package frc.robot.subsystems.TelescopicArm;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.config.TelescopicArmConfig;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.GENERAL;
import frc.robot.subsystems.TelescopicArm.TelescopicArmIO;
import frc.robot.utils.MiscUtils;

public class TelescopicArmSim implements TelescopicArmIO {
    double previousArmVelocity;
    double previousElevatorVelocity;
    double armVelocity;
    double elevatorVelocity;

    private final SingleJointedArmSim armSimSystem = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            TelescopicArmConfig.ArmSpecs.GEARING,
            0.05,
            TelescopicArmConfig.ArmSpecs.ARM_LENGTH_M,
            TelescopicArmConfig.ArmSpecs.MIN_ANGLE.getRadians(),
            TelescopicArmConfig.ArmSpecs.MAX_ANGLE.getRadians(),
            TelescopicArmConfig.ArmSpecs.SIMULATE_GRAVITY,
            TelescopicArmConfig.ArmSpecs.START_ANGLE.getRadians());
            
    private final ElevatorSim elevatorSimSystem = new ElevatorSim(
            DCMotor.getNEO(2),
            TelescopicArmConfig.ElevatorSpecs.GEARING,
            TelescopicArmConfig.ElevatorSpecs.CARRIAGE_MASS_KG,
            TelescopicArmConfig.ElevatorSpecs.DRUM_RADIUS_M,
            TelescopicArmConfig.ElevatorSpecs.MIN_HEIGHT_M,
            TelescopicArmConfig.ElevatorSpecs.MAX_HEIGHT_M,
            true,
            TelescopicArmConfig.ElevatorSpecs.STARTING_HEIGHT_M);

    private TelescopicArmDataAutoLogged data;

    // static double prevUpdateS = 0;

    public TelescopicArmSim(TelescopicArmDataAutoLogged telescopicArmData) {

        data = telescopicArmData;
    }

    @Override
    public void setElevatorVoltage(double volts) {

        double clampedVolts = MiscUtils.voltageClamp(volts);

        data.elevator_appliedVolts = clampedVolts;

        elevatorSimSystem.setInputVoltage(clampedVolts);
    }

    @Override
    public void setArmVoltage(double volts) {

        double clampedVolts = MiscUtils.voltageClamp(volts);

        data.arm_appliedVolts = clampedVolts;

        armSimSystem.setInputVoltage(clampedVolts);
    }

    @Override
    public void setMotorIdleMode(IdleMode idleMode) {
        System.out.println("[ElevatorSim] setMotorIdleMode called with " + idleMode
                + ". Note: Idle mode has no effect in simulation.");
    }

    @Override
    public void updateData() {

        // ARM
        armSimSystem.update(RobotConfig.GENERAL.NOMINAL_LOOP_TIME_S);

        previousArmVelocity = armVelocity;
        armVelocity = armSimSystem.getVelocityRadPerSec();
        
        data.arm_angle = armSimSystem.getAngleRads();
        data.arm_angularVelocityRadPS = armVelocity;
        data.arm_angularAccelRadPSS = (armVelocity - previousArmVelocity) / RobotConfig.GENERAL.NOMINAL_LOOP_TIME_S;
        // data.arm_appliedVolts = armSimSystem.;
        data.arm_currentAmps = armSimSystem.getCurrentDrawAmps();


        // ELEVATOR
        elevatorSimSystem.update(RobotConfig.GENERAL.NOMINAL_LOOP_TIME_S);

        previousElevatorVelocity = elevatorVelocity;
        elevatorVelocity = armSimSystem.getVelocityRadPerSec();

        data.elevator_position = new Translation2d(0, elevatorSimSystem.getPositionMeters() + TelescopicArmConfig.ElevatorSpecs.MOUNT_OFFSET.getY());
        data.elevator_velocityMPS = elevatorVelocity;
        data.elevator_accelMPSS = (elevatorVelocity - previousElevatorVelocity) / RobotConfig.GENERAL.NOMINAL_LOOP_TIME_S;
        data.elevator_currentAmps = elevatorSimSystem.getCurrentDrawAmps();
        // data.elevator_appliedVolts = ;


    }
}