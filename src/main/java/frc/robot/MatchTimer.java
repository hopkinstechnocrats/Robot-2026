package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class MatchTimer {
    NetworkTableInstance inst;
    NetworkTable table;
    String gameData;
    StringEntry hubIsEnabled;
    DoubleEntry gameTime;
    double matchTime = DriverStation.getMatchTime();
    CommandXboxController driveController = new CommandXboxController(Constants.ControlConstants.k_driverPort);
    CommandXboxController operatorController = new CommandXboxController(Constants.ControlConstants.k_operatorXboxControllerPort);

    public static void main(String[] args) {
        System.out.println(DriverStation.getMatchTime());
    }

    MatchTimer(){
        gameData = DriverStation.getGameSpecificMessage();
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Game Phase");

        hubIsEnabled = table.getStringTopic("/Game Phase/Testing Phases").getEntry("default");
        gameTime = table.getDoubleTopic("/Game Phase/Aprox. Game Time").getEntry(0);
    }

    public boolean allianceWin() {
        if(gameData.length() > 0){
            switch (gameData.charAt(0)){
            case 'B' :
                return true;
            case 'R' :
                return false;
            default:
                return true;
            }
        }else {
            return true;
        }
    }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (shift1Active == false){
                if (matchTime > 130) {
                // Transition shift, hub is active.
                return true;
            } else if (matchTime > 105) {
                // Shift 1
                return shift1Active;
            } else if (matchTime > 80) {
                // Shift 2
                return !shift1Active;
            } else if (matchTime > 55) {
                // Shift 3
                return shift1Active;
            } else if (matchTime > 30) {
                // Shift 4
                return !shift1Active;
            } else {
                // End game, hub always active.
                return true;
            }
        } else if (shift1Active == true){
            if (matchTime > 130) {
                // Transition shift, hub is active.
                return true;
            } else if (matchTime > 105) {
                // Shift 1
                return !shift1Active;
            } else if (matchTime > 80) {
                // Shift 2
                return shift1Active;
            } else if (matchTime > 55) {
                // Shift 3
                return !shift1Active;
            } else if (matchTime > 30) {
                // Shift 4
                return shift1Active;
            } else {
                // End game, hub always active.
                return true;
            }
        } else {
            return true;
        }
    }
    
    public void update(){
        matchTime = DriverStation.getMatchTime();
        if (isHubActive() == true){
            hubIsEnabled.set("Active!");
        } else if (isHubActive() == false){
            hubIsEnabled.set("Inactive!");
        } else {
            hubIsEnabled.set("Auto");
        }
        gameTime.set(matchTime);    
    }

    public void rumble(){
        if (matchTime > 125 && matchTime < 130 && !DriverStation.isAutonomousEnabled()){
            driveController.setRumble(GenericHID.RumbleType.kBothRumble, .1);
            operatorController.setRumble(GenericHID.RumbleType.kBothRumble, .1);
                // transition shift
            } else if (matchTime > 100 && matchTime < 105 && !DriverStation.isAutonomousEnabled()) {
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, .1);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, .1);
                // Shift 1
            } else if (matchTime > 75 && matchTime < 80 && !DriverStation.isAutonomousEnabled()) {
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, .1);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, .1);
                // Shift 2
            } else if (matchTime > 50 && matchTime < 55 && !DriverStation.isAutonomousEnabled()) {
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, .1);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, .1);
                // Shift 3
            } else if (matchTime > 25 && matchTime < 30 && !DriverStation.isAutonomousEnabled()) {
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, .1);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, .1);
                // Shift 4                
            } else {       
            }
    }

}
    

