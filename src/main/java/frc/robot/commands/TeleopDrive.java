package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.swerve.Swervedrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class TeleopDrive extends Command{
    private Swervedrive m_swerve;
    private DoubleSupplier m_x;
    private DoubleSupplier m_y;
    private DoubleSupplier m_omega;
    private DoubleSupplier m_slowMode;
    private DoubleSupplier m_fastMode;
    private BooleanSupplier m_bumpAngle;
    
    private Rotation2d desiredRotation;

    NetworkTableInstance inst;
    NetworkTable table;
    DoubleEntry calculatedOmega;

    private double m_xOut;
    private double m_yOut;
    private double m_omegaOut;

    private double invert = 1;

    public TeleopDrive(Swervedrive swervedrive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omega, DoubleSupplier slowMode, DoubleSupplier fastMode, BooleanSupplier bumpAngle){
        m_swerve = swervedrive;
        m_x = xSupplier;
        m_y = ySupplier;
        m_omega = omega;
        m_slowMode = slowMode;
        m_fastMode = fastMode;
        m_bumpAngle = bumpAngle;
        desiredRotation = new Rotation2d(Math.PI/4);

        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Drive Command");
        calculatedOmega = table.getDoubleTopic("Calculated Omega").getEntry(0);

        addRequirements(swervedrive);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
            invert = -1;
        }
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
            invert = 1;
        }

        if(m_bumpAngle.getAsBoolean()){


            // Create PID controller
            ProfiledPIDController angleController = 
                new ProfiledPIDController(
                    5.0, 0.0, 0.0, 
                    new TrapezoidProfile.Constraints(8, 20)); 
            angleController.enableContinuousInput(-Math.PI, Math.PI);

            // Calculate angular speed
            double omega =
                angleController.calculate(
                    m_swerve.getRotation().getRadians(), desiredRotation.getRadians());

            calculatedOmega.set(omega);

            // Convert to field relative speeds & send command

            m_xOut = MathUtil.applyDeadband(-m_x.getAsDouble(), Constants.ControlConstants.k_driveControllerDeadband);
            m_yOut = MathUtil.applyDeadband(-m_y.getAsDouble(), Constants.ControlConstants.k_driveControllerDeadband);

            ChassisSpeeds speeds =
                new ChassisSpeeds(
                    m_xOut * Constants.SwerveConstants.k_maxLinearSpeedMeterPerSecond,
                    m_yOut * Constants.SwerveConstants.k_maxLinearSpeedMeterPerSecond,
                    omega);
            m_swerve.Drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_swerve.getRotation()));

        } else {
            m_xOut = MathUtil.applyDeadband(-m_x.getAsDouble(), Constants.ControlConstants.k_driveControllerDeadband);
            m_yOut = MathUtil.applyDeadband(-m_y.getAsDouble(), Constants.ControlConstants.k_driveControllerDeadband);
            m_omegaOut = MathUtil.applyDeadband(Constants.SwerveConstants.k_blaireMode*m_omega.getAsDouble(), Constants.ControlConstants.k_driveControllerDeadband);

            if(m_slowMode.getAsDouble() > 0.5){
                m_xOut *= Constants.SwerveConstants.k_slowMaxLinearSpeenMetersPerSecond;
                m_yOut *= Constants.SwerveConstants.k_slowMaxLinearSpeenMetersPerSecond;
            }else if(m_fastMode.getAsDouble() > 0.5){
                m_xOut *= Constants.SwerveConstants.k_maxLinearSpeedMeterPerSecond;
                m_yOut *= Constants.SwerveConstants.k_maxLinearSpeedMeterPerSecond;

            }else{
                m_xOut *= Constants.SwerveConstants.k_midLinearSpeenMetersPerSecond;
                m_yOut *= Constants.SwerveConstants.k_midLinearSpeenMetersPerSecond;
            }
            
            m_omegaOut *= Constants.SwerveConstants.k_maxAngularSpeedRadPerSec;

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(m_xOut * invert, m_yOut * invert, m_omegaOut, m_swerve.getRotation());

            //ChassisSpeeds speeds = new ChassisSpeeds(m_xOut, m_yOut, m_omegaOut);

            m_swerve.Drive(speeds);
        }
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
