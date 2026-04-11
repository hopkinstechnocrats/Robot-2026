package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;

public class FeederCommand extends Command {

    private FeederSubsystem m_feeder;

    public FeederCommand(FeederSubsystem feederSubsystem){
        m_feeder = feederSubsystem;
    }
    
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        m_feeder.feeder(Constants.FeederConstants.k_feederSpeedRPS);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
