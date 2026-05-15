package frc.robot.autos;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.swerve.Swervedrive;

public class Autos {

    // Forward Auto
    public Command oneSecond(Swervedrive swerveDrive, double speed) {
        return new SequentialCommandGroup(
            drive(swerveDrive, speed, 0, 0).withTimeout(1)
        );
    }

    public Command timeBased(Swervedrive swerveDrive, double time) {
        return new SequentialCommandGroup(
            drive(swerveDrive, 4, 0, 0).withTimeout(time)
        );
    }

    public Command threeSecond(Swervedrive swerveDrive, double speed) {
        return new SequentialCommandGroup(
            drive(swerveDrive, speed, 0, 0).withTimeout(3)
        );
    }





    // Forward + Turn Auto
    public Command complexAuto(Swervedrive swerveDrive, double speed) {
        return new SequentialCommandGroup(
            drive(swerveDrive, speed, 0, 0).withTimeout(1),
            drive(swerveDrive, 0, speed, 0).withTimeout(2)
        );
    }

    // Drive command
    public Command drive(Swervedrive swerveDrive, double speedX, double speedY, double speedO){
        return new RunCommand(
            () -> {
                swerveDrive.Drive(new ChassisSpeeds(speedX, speedY, speedO));
            }, swerveDrive);
    }/* 
    public void driveForward(){
        double m_omegaOut = 0;
        double invert = -1;
        double m_xOut = 1;
        double m_yOut = 0;
        DoubleSupplier m_x = ()=>1;
        DoubleSupplier m_y;
        DoubleSupplier m_omega;
        Swervedrive m_swerve;

        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
            invert = -1;
        }
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
            invert = 1;
        }
        m_xOut = MathUtil.applyDeadband(-m_x.getAsDouble(), Constants.ControlConstants.k_driveControllerDeadband);
        m_yOut = MathUtil.applyDeadband(-m_y.getAsDouble(), Constants.ControlConstants.k_driveControllerDeadband);
        m_omegaOut = MathUtil.applyDeadband(Constants.SwerveConstants.k_blaireMode*m_omega.getAsDouble(), Constants.ControlConstants.k_driveControllerDeadband);

        m_xOut *= 27;
        m_yOut *= 27;
        
        
        m_omegaOut *= Constants.SwerveConstants.k_maxAngularSpeedRadPerSec;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(m_xOut * invert, m_yOut * invert, m_omegaOut, m_swerve.getRotation());
        //ChassisSpeeds speeds = new ChassisSpeeds(m_xOut, m_yOut, m_omegaOut);

        m_swerve.Drive(speeds);
    }*/
}
