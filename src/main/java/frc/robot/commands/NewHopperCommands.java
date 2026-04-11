package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;

public class NewHopperCommands extends Command {

    private final Timer m_timer = new Timer();
    HopperSubsystem m_hopperSubsystem;

    public NewHopperCommands(HopperSubsystem hopperSubsystem){
        m_hopperSubsystem = hopperSubsystem;

    
        
    }

    @Override
    public void initialize(){ 
m_timer.reset(); //makes the timer start at 0
    m_timer.start(); //Starts the clock counting up

    }

    @Override
    public void execute(){
        m_hopperSubsystem.hopper(Constants.HopperConstants.k_hopperSpeedRPS);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        // Returns true when the timer that started in initialize reaches 5.0 seconds
    return m_timer.get() >= 5.0;
    }
}

