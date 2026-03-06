package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants{

    public static final class SwerveConstants{
        public static final int k_frontLeftDriveCANID = 13;
        public static final int k_frontRightDriveCANID = 12;
        public static final int k_backLeftDriveCANID = 11;
        public static final int k_backRightDriveCANID = 10;

        public static final int k_frontLeftTurnCANID = 6;
        public static final int k_frontRightTurnCANID = 7;
        public static final int k_backLeftTurnCANID = 8;
        public static final int k_backRightTurnCANID = 9;

        public static final int k_flAbsEncoderPort = 22;
        public static final int k_frAbsEncoderPort = 21;
        public static final int k_blAbsEncoderPort = 23;
        public static final int k_brAbsEncoderPort = 20;

        public static final double k_driveKP = 0.3 ;
        public static final double k_driveKI = 0.0;
        public static final double k_driveKD = 0.0;

        public static final double k_turnKP = 12.0;
        public static final double k_turnKI = 5;
        public static final double k_turnKD = 0.0;

        public static final double frontLeftX = 0.21761;
        public static final double frontLeftY = 0.31921;
        public static final double frontRightX = 0.21761;
        public static final double frontRightY = -0.31921;
        public static final double backLeftX = -0.21761;
        public static final double backLeftY = 0.31921;
        public static final double backRightX = -0.21761;
        public static final double backRightY = -0.31921;

        public static final double k_flAbsEncoderOffset = -0.171;
        public static final double k_frAbsEncoderOffset = -0.989;
        public static final double k_blAbsEncoderOffset = 0.325;
        public static final double k_brAbsEncoderOffset = 0.221;

        public static final double k_maxLinearSpeedMeterPerSecond = 14;
        public static final double k_slowMaxLinearSpeenMetersPerSecond = 5;
        public static final double k_bumpMaxLinearSpeenMetersPerSecond = 8;
        public static final double k_maxAngularSpeedRadPerSec = 8.0 * Math.PI;

        public static final double k_driveGearRatio = 5.27;
        public static final double k_turnGearRatio = 287/11;
        public static final double k_wheelCircumferenceMeters = 0.1016 * Math.PI;

        public static final Pose2d k_startPose = new Pose2d(0, 0, new Rotation2d(0));
        //1 for inverted turning, -1 for non inverted
        public static final double k_blaireMode = 1;
    }
 
 
    public static final class ControlConstants{
        public static final double k_driveControllerDeadband = 0.1;
        public static final double k_operatorControllerDeadband = 0.1;
        public static final int k_driverPort = 0;
        public static final int k_operatorXboxControllerPort = 1;
    }
    
    public static final class GyroConstants{
        public static final int k_gyroID = 15;
    }
//TODO: test and change all of these values
    public static final class FeederConstants{
        public static final int k_feederMotorCANID = 24;
        public static final double k_feederSpeedRPS = -10;
        public static final double k_reverseFeederSpeedRPS = 20;
        public static final double k_feederBreakSpeedRPS = 0;
        public static final double k_feederP = 0.3; 
        public static final double k_feederI = 0;
        public static final double k_feederD = 0;
        public static final double k_feederFeedForward = 0; 
    }
    //TODO: test to find numbers for this whole block
    public static final class HopperConstants{
        public static final int k_hopperMotorCANID = 17;
        public static final double k_hopperSpeedRPS = 20;
        public static final double k_reverseHopperSpeedRPS = -20;   
        public static final double k_hopperBrakeSpeedRPS = 0;
        public static final double k_hopperP = 0.3; 
        public static final double k_hopperI = 0;
        public static final double k_hopperD = 0;
        public static final double k_feedForward = 0.1; 
    }
//TODO: change all of these numbers
    public static final class IntakeConstants{
        public static final int k_intakeMotorCANID = 14;
        public static final double k_intakeSpeedRPS = -50;//TODO: change this speed
        public static final double k_reverseIntakeSpeedRPS = 65;   
        public static final double k_intakeBrakeSpeedRPS = 0;
        public static final double k_intakeP = 0.3; 
        public static final double k_intakeI = 0;
        public static final double k_intakeD = 0;
        public static final double k_intakeFeedForward = 0.1;
        
        public static final int k_intakeDeployMotorCANID = 18;
        public static final int k_intakeDeployMotorFollowerCANID = 16;
        public static final double k_intakeDeployP = 20;
        public static final double k_intakeDeployI = 0;
        public static final double k_intakeDeployD = 0;
        public static final double k_intakeDeployS = 0;
        public static final double k_intakeDeployFeedForeward = 0;
        public static final double k_intakeSetpointDeploy = 0.0;
        public static final double k_intakeSetpointRetract = -0.1694;
        public static final double k_deployGearRatio = 20*(84/14);
    }

    public static final class LauncherConstants{
        // TODO test numbers 73-78
        public static final int k_launcherMotorCANID = 25;//TODO: Change CANID
        public static final int k_launcherMotorSecondCANID = 26;//TODO: Change CANID
        public static final double k_launchSpeedRPS = 78; //rotations per second
        public static final double k_launcherBrakeSpeedRPS = 0;
        public static final double k_launcherP = 0.3; 
        public static final double k_launcherI = 0.1;
        public static final double k_launcherD = 0;
        public static final double k_launcherFeedForward = 1.2/10; 
    }


}
