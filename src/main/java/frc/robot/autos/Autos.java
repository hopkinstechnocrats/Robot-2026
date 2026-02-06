package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.swerve.Swervedrive;

public class Autos {
/* 
    // Forward Auto
    public Command forwardAuto(Swervedrive swerveDrive, double speed) {
        System.out.println("2nd auto cmd ran (forwardAuto)");
        return new SequentialCommandGroup(
            drive(swerveDrive, speed).withTimeout(10)
        );
    }

    // Turn Auto
    public Command turnAuto(SwerveModule swerveModule, double speed) {
        System.out.println("2nd auto cmd ran (forwardAuto)");
        return new SequentialCommandGroup(
            turn(swerveModule, speed).withTimeout(10)
            m_turnMotor.setControl(m_turnRequest.withPosition(m_moduleState.angle.getRotations() * Constants.SwerveConstants.k_turnGearRatio));

        );
    }

    // Drive command -- speed from -1 to 1
    public Command drive(Swervedrive swerveDrive, double speed){
        return new RunCommand(
            () -> {
                swerveDrive.Drive(new ChassisSpeeds(0, speed, 0));
            }, swerveDrive);
    }

    // Turn command -- idk man
    public Command turn(SwerveModule swerveModule, double speed){
        return new RunCommand(
            () -> {
                swerveModule.Drive(new SwerveModuleState(speed, new Rotation2d(3, 3)));
            }, swerveModule);
    }
*/
}