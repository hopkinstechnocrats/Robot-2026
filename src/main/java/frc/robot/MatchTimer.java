package frc.robot;

import java.util.Optional;

import java.text.DecimalFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class MatchTimer {
    NetworkTableInstance inst;
    NetworkTable table;
    String gameData;
    StringEntry hubIsEnabled;
    StringEntry shift;
    IntegerEntry gameTime;
    IntegerEntry timeLeftInShift;
    int matchTime = (int)DriverStation.getMatchTime();
    int timeDifference = 0;
    CommandXboxController driveController = new CommandXboxController(Constants.ControlConstants.k_driverPort);
    CommandXboxController operatorController = new CommandXboxController(Constants.ControlConstants.k_operatorXboxControllerPort);


    MatchTimer(){
        
        gameData = DriverStation.getGameSpecificMessage();
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Game Phase");

        hubIsEnabled = table.getStringTopic("Hub Status").getEntry("default");
        gameTime = table.getIntegerTopic("Game Time").getEntry(0);
        shift = table.getStringTopic("Active Shift").getEntry("Not Enabled");
        timeLeftInShift = table.getIntegerTopic("Time Left In Shift").getEntry(0);

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
        matchTime = (int)DriverStation.getMatchTime();
        if (isHubActive() == true){
            hubIsEnabled.set("Active!");
        } else if (isHubActive() == false){
            hubIsEnabled.set("Inactive!");
        } else {
            hubIsEnabled.set("Auto");
        }

        if (DriverStation.isAutonomousEnabled()){
            shift.set("Auto");
            timeLeftInShift.set(matchTime);
        }

        if (matchTime-timeDifference == 5 && !DriverStation.isAutonomousEnabled()){
            driveController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
            operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
        }

        if (matchTime == timeDifference && !DriverStation.isAutonomousEnabled()){
            driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
            operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        }

        gameTime.set(matchTime);
        timeLeftInShift.set(matchTime - timeDifference);//to get the difference


        if (matchTime == 139 && !DriverStation.isAutonomousEnabled()){
            shift.set("Transition Shift");
            timeDifference = 139-10;
            //start transition shift
            } else if (matchTime == 130 && !DriverStation.isAutonomousEnabled()) {
                shift.set("Shift 1");
                timeDifference = 130-25;
                //start shift 1
            } else if (matchTime == 105 && !DriverStation.isAutonomousEnabled()) {
                shift.set("Shift 2");
                timeDifference = 105-25;
                //start shift 2
            } else if (matchTime == 80 && !DriverStation.isAutonomousEnabled()) {
                shift.set("Shift 3");
                timeDifference = 80-25;
                //start shift 3
            } else if (matchTime == 55 && !DriverStation.isAutonomousEnabled()) {
                shift.set("Shift 4");
                timeDifference = 55-25;
                //start shift 4
            } else if (matchTime == 30 && !DriverStation.isAutonomousEnabled()) {
                shift.set("Endgame");
                timeDifference = 30-30;
                //start endgame
            } else if (matchTime == 0 && !DriverStation.isAutonomousEnabled()) {
                timeDifference = 0;
                //End          
            } else {       
            }
    }

}