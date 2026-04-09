package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants;

public class TurretCommands extends Command {

    public static Command turretSpin(TurretSubsystem turret) {
            return Commands.run(
                () -> {
                    turret.turretSpin(Constants.TurretConstants.k_turretSpeedRPS);
                },
            turret);
    }
    public static Command turretBrake(TurretSubsystem turret){
        return Commands.run(
            () -> {
                turret.turretBrake(Constants.TurretConstants.k_turretBrakeSpeedRPS);
            },
            turret);
    }

    public static Command turretReverse(TurretSubsystem turret){
        return Commands.run(
            () -> {
                turret.turretSpin(-Constants.TurretConstants.k_turretSpeedRPS);
            },
            turret);
    }
        
}
