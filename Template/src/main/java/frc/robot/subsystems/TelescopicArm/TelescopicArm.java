package frc.robot.subsystems.TelescopicArm;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.TelescopicArmConfig.ArmControl;
import frc.robot.config.TelescopicArmConfig.ElevatorControl;
import frc.robot.config.TelescopicArmConfig.ElevatorSpecs;
import frc.robot.config.TelescopicArmConfig.TelescopicArmStates;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class TelescopicArm extends SubsystemBase {
    TelescopicArmIO io;
    TelescopicArmDataAutoLogged data = new TelescopicArmDataAutoLogged();

    // elevator and arm config seperate
    ArmControl.ControlConfig armConfig = ArmControl.CONTROL_CONFIG;
    ElevatorControl.ControlConfig elevatorConfig = ElevatorControl.CONTROL_CONFIG;

    ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(elevatorConfig.kS, elevatorConfig.kG,
            elevatorConfig.kV,
            elevatorConfig.kA);
    ProfiledPIDController elevatorProfile = new ProfiledPIDController(elevatorConfig.kP, elevatorConfig.kI,
            elevatorConfig.kD, new Constraints(elevatorConfig.MAX_VELOCITY_MPS,
                    elevatorConfig.MAX_ACCEL_MPSS));

    ArmFeedforward armFeedforward = new ArmFeedforward(armConfig.kS, armConfig.kG, armConfig.kV, armConfig.kA);
    ProfiledPIDController armProfile = new ProfiledPIDController(armConfig.kP, armConfig.kI, armConfig.kD,
            new Constraints(armConfig.MAX_VELOCITY_RadPS, armConfig.MAX_VELOCITY_RadPS));

    private LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
    private LoggedMechanismRoot2d root = mech.getRoot("", 1, 0);
    private LoggedMechanismLigament2d telescopingMech = root
            .append(new LoggedMechanismLigament2d("elevator", ElevatorSpecs.BASE_HEIGHT, 90)); // should be base height

    TelescopicArmStates state;

    public TelescopicArm() {
        io = new TelescopicArmSim();
    }

    public double getArmAngle() {
        return data.arm_angle.getRadians();
    }

    public double getHeight() {
        return data.elevator_position.getY();
    }

    public void setState(TelescopicArmStates state) {
        this.state = state;
        if(state==TelescopicArmStates.STOP)
        {
            stop();
            return;
        }
        goToTranslation(state.position);
    }

    public void stop() {
        io.setArmVoltage(0);
        io.setElevatorVoltage(0);
    }

    public void setGoal(double height, double angleRads) {
        elevatorProfile.setGoal(height);
        armProfile.setGoal(angleRads);
    }

    private void moveToGoal() {
        State firstArmState = armProfile.getSetpoint();
        double armPID = armProfile.calculate(getArmAngle());

        State nextArmState = armProfile.getSetpoint();
        double ffArmVoltage = armFeedforward.calculateWithVelocities(getArmAngle(), firstArmState.velocity,
                nextArmState.velocity);

        State firstElevatorstate = elevatorProfile.getSetpoint();
        double elevatorPID = elevatorProfile.calculate(getHeight());

        State nextElevatorState = elevatorProfile.getSetpoint();
        double ffElevatorVoltage = elevatorFeedforward.calculateWithVelocities(firstElevatorstate.velocity,
                nextElevatorState.velocity);
                
        System.out.println(firstArmState.position);
        System.out.println(data.arm_angle.getRadians());
        io.setArmVoltage(armPID + ffArmVoltage);
        io.setElevatorVoltage(elevatorPID + ffElevatorVoltage);
    }

    public void goToTranslation(Translation2d point) {

        double x = point.getX();
        double y = point.getY();

        double setPointAngle = Math.atan2(y, x);
        double setPointDistance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        SmartDashboard.putNumber("SETPOINT DISTANCE", setPointDistance);
        SmartDashboard.putNumber("SETPOINT ANGLE", Math.toDegrees(setPointAngle));

        setGoal(setPointDistance, setPointAngle);
    }

    public Translation2d getTranslation() {

        // CHANGE THIS
        return new Translation2d();
    }

    public void updateMechanism() {
        telescopingMech.setAngle(getArmAngle());
        telescopingMech.setLength(getHeight());

        Logger.recordOutput("Telescopic/telescopicMechanism", mech);
    }

    @Override
    public void periodic() {
        io.updateData(data);
        Logger.processInputs("TelescopicArm", data);
        moveToGoal();
        updateMechanism();
        SmartDashboard.putNumber("ARM ANGLE" , data.arm_angle.getDegrees());
        SmartDashboard.putNumber("ELEVATOR VOLTAGE" , data.elevator_appliedVolts);
    }
}
