package frc.robot.subsystems.TelescopicArm;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.TelescopicArmConfig.ArmControl;
import frc.robot.config.TelescopicArmConfig.ElevatorControl;
import frc.robot.config.TelescopicArmConfig.ElevatorSpecs;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class TelescopicArm extends SubsystemBase {
    TelescopicArmIO io;
    TelescopicArmAutoLogged data = new TelescopicArmAutoLogged();

    // elevator and arm config seperate
    ArmControl.ControlConfig armConfig = ArmControl.CONTROL_CONFIG;
    ElevatorControl.ControlConfig elevatorConfig = ElevatorControl.CONTROL_CONFIG;

    ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(elevatorConfig.kS, elevatorConfig.kG,
            elevatorConfig.kV,
            elevatorConfig.kA);
    ProfiledPIDController elevatorPid = new ProfiledPIDController(elevatorConfig.kP, elevatorConfig.kI,
            elevatorConfig.kD, new Constraints(elevatorConfig.MAX_VELOCITY_MPS,
                    elevatorConfig.MAX_ACCEL_MPSS));

    ArmFeedforward armFeedforward = new ArmFeedforward(armConfig.kS, armConfig.kG, armConfig.kV, armConfig.kA);
    ProfiledPIDController armPID = new ProfiledPIDController(armConfig.kP, armConfig.kI, armConfig.kD,
            new Constraints(armConfig.MAX_VELOCITY_RadPS, armConfig.MAX_VELOCITY_RadPS));

    private LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
    private LoggedMechanismRoot2d root = mech.getRoot("", 1, 0);
    private LoggedMechanismLigament2d telescopingMech = root
            .append(new LoggedMechanismLigament2d("elevator", ElevatorSpecs.baseHeight, 90)); // should be base height

    public TelescopicArm() {
        io = new TelescopicArmSim(data);
    }

    public void goToTranslation(Translation2d point) {

        double x = point.getX();
        double y = point.getY();

        double setPointAngle = Math.atan2(y, x);
        double setPointDistance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        // TODO: CHANGE MECHS TO GO HERE
    }

    public Translation2d getTranslation() {

        // CHANGE THIS
        return new Translation2d();
    }

    public void updateMech() {
        telescopingMech.setAngle(data.armAngle);
        telescopingMech.setLength(data.elevatorHeight); // add base height
    }

    @Override
    public void periodic() {
        io.updateData();
    }
}
