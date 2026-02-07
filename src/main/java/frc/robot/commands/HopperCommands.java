package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;

public class HopperCommands {
       
    public static Command hopper(HopperSubsystem wheel) {
        return Commands.run(
            () -> {
                wheel.hopper(Constants.HopperConstants.k_hopperSpeed);
            },
        wheel);
    }
    public static Command reverseHopper(HopperSubsystem wheel) {
        return Commands.run(
            () -> {
                wheel.hopper(Constants.HopperConstants.k_reverseHopperSpeed);
            },
        wheel);
    }
}
