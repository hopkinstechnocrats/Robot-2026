package frc.robot.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.DoubleEntry;
import frc.robot.Constants;

public class Swervedrive extends SubsystemBase{
    
    SwerveDriveKinematics m_swerveKinematics;
    Translation2d m_frontLeftPosition;
    Translation2d m_frontRightPosition;
    Translation2d m_backLeftPosition;
    Translation2d m_backRightPosition;

    SwerveDriveOdometry swerveOdometry;
    Pose2d m_pose;
    SwerveDrivePoseEstimator m_poseEstimator;
    
    NetworkTableInstance inst;
    NetworkTable table;

    StructArrayPublisher<SwerveModuleState> desiredStatePublisher;
    StructArrayPublisher<SwerveModuleState> actualStatePublisher;

    StructPublisher<Pose2d> robotPosition;

    DoubleEntry flAnalog;
    DoubleEntry frAnalog;
    DoubleEntry blAnalog;
    DoubleEntry brAnalog;

    SwerveModule fL;
    SwerveModule fR;
    SwerveModule bL;
    SwerveModule bR;

    Gyro gyro;

    SwerveModuleState[] desiredModuleStates;
    SwerveModuleState[] actualModuleState = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    ChassisSpeeds m_speeds;

    public Swervedrive(){
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Swerve");

        desiredStatePublisher = table.getStructArrayTopic("Desired Module States", SwerveModuleState.struct).publish();
        actualStatePublisher = table.getStructArrayTopic("Actual Module States", SwerveModuleState.struct).publish();

        m_frontLeftPosition = new Translation2d(Constants.SwerveConstants.frontLeftX, Constants.SwerveConstants.frontLeftY);
        m_frontRightPosition = new Translation2d(Constants.SwerveConstants.frontRightX, Constants.SwerveConstants.frontRightY);
        m_backLeftPosition = new Translation2d(Constants.SwerveConstants.backLeftX, Constants.SwerveConstants.backLeftY);
        m_backRightPosition = new Translation2d(Constants.SwerveConstants.backRightX, Constants.SwerveConstants.backRightY);

        fL = new SwerveModule(Constants.SwerveConstants.k_frontLeftDriveCANID, Constants.SwerveConstants.k_frontLeftTurnCANID, 
                Constants.SwerveConstants.k_flAbsEncoderPort, Constants.SwerveConstants.k_flAbsEncoderOffset);
        fR = new SwerveModule(Constants.SwerveConstants.k_frontRightDriveCANID, Constants.SwerveConstants.k_frontRightTurnCANID, 
                Constants.SwerveConstants.k_frAbsEncoderPort, Constants.SwerveConstants.k_frAbsEncoderOffset);
        bL = new SwerveModule(Constants.SwerveConstants.k_backLeftDriveCANID, Constants.SwerveConstants.k_backLeftTurnCANID,
                Constants.SwerveConstants.k_blAbsEncoderPort, Constants.SwerveConstants.k_blAbsEncoderOffset);
        bR = new SwerveModule(Constants.SwerveConstants.k_backRightDriveCANID, Constants.SwerveConstants.k_backRightTurnCANID,
                Constants.SwerveConstants.k_brAbsEncoderPort, Constants.SwerveConstants.k_brAbsEncoderOffset);

        m_swerveKinematics = new SwerveDriveKinematics(m_frontLeftPosition, m_frontRightPosition, m_backLeftPosition, m_backRightPosition);

        gyro = new Gyro(Constants.GyroConstants.k_gyroID);
        m_poseEstimator  = new SwerveDrivePoseEstimator(m_swerveKinematics, gyro.getRotation(), new SwerveModulePosition[]{
            fL.getModulePosition(), fR.getModulePosition(), bL.getModulePosition(), bR.getModulePosition()
        }, Constants.SwerveConstants.k_startPose);

        flAnalog = table.getDoubleTopic("FL Absolute Encoder").getEntry(0);
        frAnalog = table.getDoubleTopic("FR Absolute Encoder").getEntry(0);
        blAnalog = table.getDoubleTopic("BL Absolute Encoder").getEntry(0);
        brAnalog = table.getDoubleTopic("BR Absolute Encoder").getEntry(0);

        robotPosition = table.getStructTopic("Robot Position", Pose2d.struct).publish();


        // Configure AutoBuilder last
        AutoBuilder.configure(
            m_poseEstimator, // Robot pose supplier
            resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            m_speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> Drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
            config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
            },
            this
            );

    }


    @Override
    public void periodic(){
        m_pose = m_poseEstimator.update(gyro.getRotation(), new SwerveModulePosition[]{
             fL.getModulePosition(), fR.getModulePosition(), bL.getModulePosition(), bR.getModulePosition()
        });

        desiredStatePublisher.set(desiredModuleStates);

        updateActualStates();

        actualStatePublisher.set(actualModuleState);

        flAnalog.set(fL.getAbsEncoderPositionRot());
        frAnalog.set(fR.getAbsEncoderPositionRot());
        blAnalog.set(bL.getAbsEncoderPositionRot());
        brAnalog.set(bR.getAbsEncoderPositionRot());
        robotPosition.set(m_pose);
    }

    public void Drive(ChassisSpeeds desiredState){
        desiredModuleStates = m_swerveKinematics.toSwerveModuleStates(desiredState);
        fL.Drive(desiredModuleStates[0]);
        fR.Drive(desiredModuleStates[1]);
        bL.Drive(desiredModuleStates[2]);
        bR.Drive(desiredModuleStates[3]);
    }

    private void updateActualStates(){
        actualModuleState[0] = new SwerveModuleState(fL.getDriveVelocityMeterPerSec(), fL.getAngleRotation2d());
        actualModuleState[1] = new SwerveModuleState(fR.getDriveVelocityMeterPerSec(), fR.getAngleRotation2d());
        actualModuleState[2] = new SwerveModuleState(bL.getDriveVelocityMeterPerSec(), bL.getAngleRotation2d());
        actualModuleState[3] = new SwerveModuleState(bR.getDriveVelocityMeterPerSec(), bR.getAngleRotation2d());

    }
    
    public Rotation2d getRotation(){
        return gyro.getRotation();
    }
        
    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(gyro.getRotation(), this.getModulePositions(), pose);
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0, 0), gyro.getRotation());
        m_swerveKinematics.toSwerveModuleStates(robotRelativeSpeeds);
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[]{
            fL.getModulePosition(),
            fR.getModulePosition(),
            bL.getModulePosition(),
            bR.getModulePosition()
        };
        
        return positions;
    }




}
