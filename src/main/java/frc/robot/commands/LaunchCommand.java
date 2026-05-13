package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class LaunchCommand extends Command{
    LauncherSubsystem m_launcher;
    HopperSubsystem m_hopper;
    FeederSubsystem m_feeder;
    
    public LaunchCommand(HopperSubsystem hopper, LauncherSubsystem launcher, FeederSubsystem feeder){
        m_launcher = launcher;
        m_feeder = feeder;
        m_hopper = hopper;
        addRequirements(hopper, launcher, feeder);
        
    }
    @Override
    public void execute(){
        m_launcher.launcher(LauncherConstants.k_launchSpeedRPS);
        if(m_launcher.atSpeed()){
        m_feeder.feeder(FeederConstants.k_feederSpeedRPS);
        m_hopper.hopper(HopperConstants.k_hopperSpeedRPS);
        }
        else{
            m_feeder.feeder(0);
            m_hopper.hopper(0);
        }
    }
}
