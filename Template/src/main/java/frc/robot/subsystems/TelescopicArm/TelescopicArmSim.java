package frc.robot.subsystems.TelescopicArm;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.config.TelescopicArmConfig;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.GENERAL;
import frc.robot.subsystems.TelescopicArm.ElevatorDataAutoLogged;
import frc.robot.subsystems.TelescopicArm.TelescopicArmIO;
import frc.robot.utils.MiscUtils;

/**
 * The reason this class is named weirdly is because "ElevatorSim" is already
 * taken by WPILib to give us the {@link ElevatorSim} utility class
 */
public class TelescopicArmSim implements TelescopicArmIO {

    private final SingleJointedArmSim armSimSystem = new SingleJointedArmSim(DCMotor.getNEO(1),
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

    /**
     * This is our {@link ElevatorData} instance that we will update with data
     * We have to use the "AutoLogged" variation because of the @AutoLog annotation,
     * which will create a new class for the data object
     */
    private TelescopicArmDataAutoLogged data;

    static double prevUpdateS = 0;

    public TelescopicArmSim(TelescopicArmDataAutoLogged telescopicData) {

        data = telescopicData;
    }

    @Override
    public void setElevatorVoltage(double volts) {

        double clampedVolts = MiscUtils.voltageClamp(volts);

        data.leftAppliedVolts = clampedVolts;
        data.rightAppliedVolts = clampedVolts;

        elevatorSimSystem.setInputVoltage(clampedVolts);
    }

    @Override
    public void setArmVoltage(double volts) {

        double clampedVolts = MiscUtils.voltageClamp(volts);

        data.appliedVolts = clampedVolts;

        armSimSystem.setInputVoltage(clampedVolts);
    }

    @Override
    public void setMotorIdleMode(IdleMode idleMode) {
        System.out.println("[ElevatorSim] setMotorIdleMode called with " + idleMode
                + ". Note: Idle mode has no effect in simulation.");
    }

    @Override
    public void updateData() {
        elevatorSimSystem.update(RobotConfig.GENERAL.NOMINAL_LOOP_TIME_S);
        armSimSystem.update(RobotConfig.GENERAL.NOMINAL_LOOP_TIME_S);

        data.position = new Translation2d(0, elevatorSimSystem.getPositionMeters() +
                TelescopicArmConfig.ElevatorSpecs.MOUNT_OFFSET.getY());
        data.velocityMPS = elevatorSimSystem.getVelocityMetersPerSecond();

        // The following data is not useful in simulation:
        data.leftCurrentAmps = elevatorSimSystem.getCurrentDrawAmps();
        data.rightCurrentAmps = elevatorSimSystem.getCurrentDrawAmps();
        // this *could* be calculated by using ((currentVelocity -
        // prevVelocity)/deltaTimeS), but once again, not worth it for sim
        data.accelMPSS = 0;
    }
}