package frc.robot.commands.telescopicArm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.config.TelescopicArmConfig.TelescopicArmStates;

public class GoToLevel extends Command {
    TelescopicArmStates state;

    public GoToLevel(TelescopicArmStates state) {
        addRequirements(Robot.telescopicArm);
        this.state = state;
    }

    @Override
    public void initialize() {
        Robot.telescopicArm.setState(state);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}