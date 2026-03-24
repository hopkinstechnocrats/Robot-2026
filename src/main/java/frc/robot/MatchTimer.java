package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;


public class MatchTimer {
    NetworkTableInstance inst;
    NetworkTable table;
    String gameData;
    StringEntry hubIsEnabled;
    
    MatchTimer(){
        gameData = DriverStation.getGameSpecificMessage();
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Game Phase");
        if (isHubActive() == true){
            hubIsEnabled = table.getStringTopic("Game phase").getEntry("Active!");
        } else if (isHubActive() == false){
            hubIsEnabled = table.getStringTopic("Game phase").getEntry("Inactive!");
        } else {
            hubIsEnabled = table.getStringTopic("Game phase").getEntry("Auto");
        }
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
        double matchTime = DriverStation.getMatchTime();
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

        if (redInactiveFirst == false){
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
        } else if (redInactiveFirst == true){
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

}
    

