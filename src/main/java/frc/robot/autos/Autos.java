package frc.robot.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.swerve.Swervedrive;

public class Autos {

    // Forward Auto
    public Command forwardAuto(Swervedrive swerveDrive, double speed) {
        System.out.println("2nd auto cmd ran (forwardAuto)");
        return new SequentialCommandGroup(
            drive(swerveDrive, speed).withTimeout(10) 
        );
    }

    // Drive command -- speed from -1 to 1
    public Command drive(Swervedrive swerveDrive, double speed){
        System.out.println("1st auto cmd ran (drive)");
        return new RunCommand(
            () -> {
                swerveDrive.Drive(new ChassisSpeeds(0, speed, 0));
            }, swerveDrive);
    }

}