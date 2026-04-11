package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;

public class FeederCommand extends Command {

    private FeederSubsystem m_feeder;
private final Timer m_timer = new Timer();

    public FeederCommand(FeederSubsystem feederSubsystem){
        m_feeder = feederSubsystem;
    }
    
    
    
    @Override
    public void initialize(){
       m_timer.reset(); //makes the timer start at 0
    m_timer.start(); //Starts the clock counting up
    }

    @Override
    public void execute(){
        m_feeder.feeder(Constants.FeederConstants.k_feederSpeedRPS);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        // Returns true when the timer that started in initialize reaches 5.0 seconds
    return m_timer.get() >= 5.0;
    
    }
}
