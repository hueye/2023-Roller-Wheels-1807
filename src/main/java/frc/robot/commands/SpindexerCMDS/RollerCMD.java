package frc.robot.commands.SpindexerCMDS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RollerCMD extends CommandBase {
    public RollerCMD() {
        addRequirements(RobotContainer.rollSub);
    }

@Override
public void initialize() {}

@Override
public void execute() {
RobotContainer.rollSub.Roll();
}

@Override
public void end(boolean interrupted) {}

@Override
public boolean isFinished() {
    return false;
}
}
