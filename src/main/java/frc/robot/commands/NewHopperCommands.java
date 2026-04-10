package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;

public class NewHopperCommands extends Command {

    HopperSubsystem m_hopperSubsystem;

    public NewHopperCommands(HopperSubsystem hopperSubsystem){
        m_hopperSubsystem = hopperSubsystem;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        m_hopperSubsystem.hopper(Constants.HopperConstants.k_hopperSpeedRPS);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}

