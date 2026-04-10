package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

    private IntakeSubsystem m_intake;
    private Boolean m_deploy;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, Boolean deploy){
        m_intake = intakeSubsystem;
        m_deploy = deploy;
    }
    
    @Override
    public void initialize(){}

    @Override
    public void execute(){
        if(m_deploy){
            m_intake.intakeDeploy(Constants.IntakeConstants.k_intakeSetpointDeploy);
        } else {
            m_intake.intake(Constants.IntakeConstants.k_intakeSpeedRPS);
        }
    }  

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
