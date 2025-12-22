package frc.robot.subsystems.TelescopicArm;
import frc.robot.config.TelescopicArmConfig.ArmControl.ControlConfig;


public class TelescopicArm extends {
    TelescopicArmIO io;
    TelescopicArmAutoLogged data = new TelescopicArmAutoLogged();

    // elevator and arm config seperate
    ControlConfig armConfig = ElevatorControl.CONTROL_CONFIG;

    public TelescopicArm(TelescopicArmIO io) {
        this.io = io;
    }

    perio
}
