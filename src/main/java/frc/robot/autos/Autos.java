package frc.robot.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.swerve.Swervedrive;

public class Autos {

    // Forward Auto
    public Command forwardAuto(Swervedrive swerveDrive, double speed) {
        return new SequentialCommandGroup(
            drive(swerveDrive, speed, 0, 0).withTimeout(2)
        );
    }

    // Forward + Turn Auto
    public Command complexAuto(Swervedrive swerveDrive, double speed) {
        return new SequentialCommandGroup(
            drive(swerveDrive, speed, 0, 0).withTimeout(2),
            drive(swerveDrive, 0, speed, 0).withTimeout(2)
        );
    }

    // Drive command
    public Command drive(Swervedrive swerveDrive, double speedX, double speedY, double speedO){
        return new RunCommand(
            () -> {
                swerveDrive.Drive(new ChassisSpeeds(speedX, speedY, speedO));
            }, swerveDrive);
    }

}