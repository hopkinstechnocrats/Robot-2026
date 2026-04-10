package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LauncherSubsystem;

public class NewLauncherCommands extends Command {

    LauncherSubsystem m_launcherSubsystem;

    public NewLauncherCommands(LauncherSubsystem launcherSubsystem){
        m_launcherSubsystem = launcherSubsystem;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        m_launcherSubsystem.launcher(Constants.LauncherConstants.k_launchSpeedRPS);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
