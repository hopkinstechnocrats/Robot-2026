package frc.robot;

import java.util.Optional;

import java.text.DecimalFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class MatchTimer {
    NetworkTableInstance inst;
    NetworkTable table;
    String gameData;
    StringEntry hubIsEnabled;
    StringEntry shift;
    StringEntry gameTime;
    double matchTime = DriverStation.getMatchTime();
    DecimalFormat time = new DecimalFormat("#.##");
    CommandXboxController driveController = new CommandXboxController(Constants.ControlConstants.k_driverPort);
    CommandXboxController operatorController = new CommandXboxController(Constants.ControlConstants.k_operatorXboxControllerPort);


    MatchTimer(){
        
        gameData = DriverStation.getGameSpecificMessage();
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Game Phase");

        hubIsEnabled = table.getStringTopic("Hub Status").getEntry("default");
        gameTime = table.getStringTopic("Game Time").getEntry("0");
        shift = table.getStringTopic("Active Shift").getEntry("Not Enabled");

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

        if (DriverStation.isAutonomousEnabled()){
            shift.set("Auto");
        }

        gameTime.set(time.format(matchTime));

        if (matchTime > 139 && matchTime < 140 && !DriverStation.isAutonomousEnabled()){
            shift.set("Transition Shift");
            //start transition shift
            } else if (matchTime > 130 && matchTime < 135 && !DriverStation.isAutonomousEnabled()) {
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
                // transition shift rumble
            } else if (matchTime > 129 && matchTime < 130 && !DriverStation.isAutonomousEnabled()) {
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, .0);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, .0);
                shift.set("Shift 1");
                //rumble stop, start shift 1
            } else if (matchTime > 105 && matchTime < 110 && !DriverStation.isAutonomousEnabled()) {
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
                // Shift 1 rumble
            } else if (matchTime > 104 && matchTime < 105 && !DriverStation.isAutonomousEnabled()) {
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, .0);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, .0);
                shift.set("Shift 2");
                //rumble stop, start shift 2
            } else if (matchTime > 80 && matchTime < 85 && !DriverStation.isAutonomousEnabled()) {
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
                // Shift 2 rumble
            } else if (matchTime > 79 && matchTime < 80 && !DriverStation.isAutonomousEnabled()) {
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, .0);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, .0);
                shift.set("Shift 3");
                //rumble stop, start shift 3
            } else if (matchTime > 55 && matchTime < 60 && !DriverStation.isAutonomousEnabled()) {
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
                // Shift 3 rumble
            } else if (matchTime > 54 && matchTime < 55 && !DriverStation.isAutonomousEnabled()) {
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, .0);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, .0);
                shift.set("Shift 4");
                //rumble stop, start shift 4
            } else if (matchTime > 30 && matchTime < 35 && !DriverStation.isAutonomousEnabled()) {
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
                // Shift 4 rumble
            } else if (matchTime > 29 && matchTime < 30 && !DriverStation.isAutonomousEnabled()) {
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, .0);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, .0);
                shift.set("Endgame");
                //rumble stop, start endgame
            } else if (matchTime > 1 && matchTime < 5 && !DriverStation.isAutonomousEnabled()) {
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
                // Endgame rumble
            } else if (matchTime > 0 && matchTime < 1 && !DriverStation.isAutonomousEnabled()) {
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, .0);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, .0);
                //rumble stop                
            } else {       
            }
    }

}