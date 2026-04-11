package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants;

public class TurretCommands extends Command {

    public static Command turretPositionRight(TurretSubsystem turret) {
            return Commands.run(
                () -> {
                    turret.turretPositionRight(Constants.TurretConstants.k_turretRightPosition);
                },
            turret);
    }
    public static Command turretPositionCenter(TurretSubsystem turret){
        return Commands.run(
            () -> {
                turret.turretPositionCenter(Constants.TurretConstants.k_turretCenterPosition);
            },
            turret);
    }

    public static Command turretPositionLeft(TurretSubsystem turret){
        return Commands.run(
            () -> {
                turret.turretPositionLeft(-Constants.TurretConstants.k_turretLeftPosition);
            },
            turret);
    }
    
    public static Command turretBrake(TurretSubsystem turret) {
            return Commands.run(
                () -> {
                    turret.turretPositionRight(Constants.TurretConstants.k_turretRightPosition);
                },
            turret);
    }
}
