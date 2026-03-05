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

        public static final double k_maxLinearSpeedMeterPerSecond = 7.5;
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
        public static final int operatorXboxControllerPort = 1;
    }
    
    public static final class GyroConstants{
        public static final int k_gyroID = 15;
    }
//TODO: change all of these numbers
    public static final class IntakeConstants{
        public static final int k_intakeMotorCANID = 14;
        public static final double k_intakeSpeedRPS = 50;//TODO: change this speed
        public static final double k_reverseIntakeSpeedRPS = -1000;   
        public static final double k_intakeBrakeSpeedRPS = 0;
        public static final double k_intakeP = 0.3; 
        public static final double k_intakeI = 0;
        public static final double k_intakeD = 0;
        public static final double k_intakeFeedForward = 0.1;
        
        public static final int k_intakeDeployMotorCANID = 18;
        public static final int k_intakeDeployMotorFollowerCANID = 16;
        public static final double k_intakeDeployP = 10;
        public static final double k_intakeDeployI = 0;
        public static final double k_intakeDeployD = 0;
        public static final double k_intakeDeployS = 0;
        public static final double k_intakeDeployFeedForeward = 0;
        public static final double k_intakeSetpointDeploy = 0.25;
        public static final double k_intakeSetpointBob = 0.2;
        public static final double k_intakeSetpointRetract = 0;
        public static final double k_intakeDeployPosition = 0;       
        public static final double k_deployGearRatio = 20*(84/14);
    }


}
