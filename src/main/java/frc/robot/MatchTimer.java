package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.BooleanEntry;
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
    BooleanEntry hubColors;
    int matchTime = (int)DriverStation.getMatchTime();
    int timeDifference = 0;
    public boolean firstShift;
    boolean powerRumble = false;
    CommandXboxController driveController = new CommandXboxController(Constants.ControlConstants.k_driverPort);
    CommandXboxController operatorController = new CommandXboxController(Constants.ControlConstants.k_operatorXboxControllerPort);
    

    MatchTimer(){


        gameData = DriverStation.getGameSpecificMessage();
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Game Data For Drive Team");

        hubIsEnabled = table.getStringTopic("Hub Status").getEntry("default");
        gameTime = table.getIntegerTopic("Total Game Time Remaining").getEntry(0);
        shift = table.getStringTopic("Current Shift").getEntry("default");
        timeLeftInShift = table.getIntegerTopic("Time Left In Shift").getEntry(0);
        hubColors = table.getBooleanTopic("Hub Status Color").getEntry(false);

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

        firstShift = shift1Active;
        // to bypass the variavble scope. couldn't think of another way to do it :p

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
        }
    
    public void update(){        
            
        matchTime = (int)DriverStation.getMatchTime();//pulls live match

        if (isHubActive() == true){
            hubIsEnabled.set("Active!");
            hubColors.set(true);
        } else if (isHubActive() == false){
            hubIsEnabled.set("Inactive!"); 
            hubColors.set(false);
        } else {
            hubIsEnabled.set("Auto");
            hubColors.set(true);
        }//sets whether or not the hub is active

        if (DriverStation.isAutonomousEnabled()){
            shift.set("Auto");
            gameTime.set(matchTime + 140);
            timeDifference = 0;
        }//sets auto time, resets the time left in shift

        if(powerRumble == true){
            if (timeLeftInShift.get() % 2 == 1 && timeLeftInShift.get() < 6  && !DriverStation.isAutonomousEnabled()){
                driveController.setRumble(GenericHID.RumbleType.kBothRumble, 1.5);
                operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 1.5);
            }

            if(timeLeftInShift.get() % 2 == 0 && timeLeftInShift.get() < 5 && !DriverStation.isAutonomousEnabled()){
            driveController.setRumble(GenericHID.RumbleType.kLeftRumble, .3);
            operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, .3);
            //Pulsing rumble when active

        }else{
            if (timeLeftInShift.get() == 5 && !DriverStation.isAutonomousEnabled()){
                driveController.setRumble(GenericHID.RumbleType.kLeftRumble, .3);
                operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, .3);
            }
        }//constant low rumble when inactive
        
        if (timeLeftInShift.get() == 0 && !DriverStation.isAutonomousEnabled()){
            driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
            operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        }//rumble stop

        if(!DriverStation.isAutonomousEnabled()){
            gameTime.set(matchTime);
        }//updating match time

        timeLeftInShift.set(matchTime - timeDifference);//to get the difference for time left in the shift

        if (matchTime == 139 && !DriverStation.isAutonomousEnabled()){
            shift.set("Transition Shift");
            timeDifference = 140 - 10;// current match time - lenght of shift
            if (firstShift == true){
                    powerRumble = true;
                }else{
                    powerRumble = false;
                }
            //start transition shift, set correct rumble type, sets the time left in shift
        } else if (matchTime == 130 && !DriverStation.isAutonomousEnabled()) {
            shift.set("Shift 1");
            timeDifference = 130-25;
            if (isHubActive() == false){
                powerRumble = true;
            }else{
                powerRumble = false;
            }
            //start shift 1, set correct rumble type, sets the time left in shift
        } else if (matchTime == 105 && !DriverStation.isAutonomousEnabled()) {
            shift.set("Shift 2");
            timeDifference = 105-25;
            if (isHubActive() == false){
                powerRumble = true;
            }else{
                powerRumble = false;
            }
                //start shift 2, set correct rumble type, sets the time left in shift
        } else if (matchTime == 80 && !DriverStation.isAutonomousEnabled()) {
            shift.set("Shift 3");
            timeDifference = 80-25;
            if (isHubActive() == false){
                powerRumble = true;
            }else{
                powerRumble = false;
            }
            //start shift 3, set correct rumble type, sets the time left in shift
        } else if (matchTime == 55 && !DriverStation.isAutonomousEnabled()) {
            shift.set("Shift 4");
            timeDifference = 55-25;
            if (isHubActive() == false){
                powerRumble = true;
            }else{
                powerRumble = false;
            }
            //start shift 4, set correct rumble type, sets the time left in shift
        } else if (matchTime == 30 && !DriverStation.isAutonomousEnabled()) {
            shift.set("Endgame");
            timeDifference = 30-30;
            powerRumble = true;
            //start endgame, set correct rumble type, sets the time left in shift
        } else if (matchTime == 0 && !DriverStation.isAutonomousEnabled()) {
            timeDifference = 0;
            shift.set("Game Over");
             //End of game      
            } else {       
            }
        }
    }
}