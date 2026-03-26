package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.swerve.Swervedrive;
import java.util.function.DoubleSupplier;


public class TeleopDrive extends Command{
    private Swervedrive m_swerve;
    private DoubleSupplier m_x;
    private DoubleSupplier m_y;
    private DoubleSupplier m_omega;
    private DoubleSupplier m_fastMode;
    private DoubleSupplier m_bumpMode;

    private double m_xOut;
    private double m_yOut;
    private double m_omegaOut;

    private double invert = 1;

    public TeleopDrive(Swervedrive swervedrive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omega, DoubleSupplier fastMode, DoubleSupplier bumpMode){
        m_swerve = swervedrive;
        m_x = xSupplier;
        m_y = ySupplier;
        m_omega = omega;
        m_fastMode = fastMode;
        m_bumpMode = bumpMode;

        addRequirements(swervedrive);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
            invert = -1;
        }
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
            invert = 1;
        }
        m_xOut = MathUtil.applyDeadband(-m_x.getAsDouble(), Constants.ControlConstants.k_driveControllerDeadband);
        m_yOut = MathUtil.applyDeadband(-m_y.getAsDouble(), Constants.ControlConstants.k_driveControllerDeadband);
        m_omegaOut = MathUtil.applyDeadband(Constants.SwerveConstants.k_blaireMode*m_omega.getAsDouble(), Constants.ControlConstants.k_driveControllerDeadband);

        if(m_fastMode.getAsDouble() > 0.5){
            m_xOut *= 19; // right
            m_yOut *= 19;
        }else if(m_bumpMode.getAsDouble() > 0.5){
            m_xOut *= 50; // left
            m_yOut *= 50;

        }else{
            m_xOut *= 27; // none
            m_yOut *= 27;
        }
        
        
        m_omegaOut *= Constants.SwerveConstants.k_maxAngularSpeedRadPerSec;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(m_xOut * invert, m_yOut * invert, m_omegaOut, m_swerve.getRotation());
        //ChassisSpeeds speeds = new ChassisSpeeds(m_xOut, m_yOut, m_omegaOut);

        m_swerve.Drive(speeds);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
