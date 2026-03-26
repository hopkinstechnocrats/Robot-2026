package frc.robot.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TunableNumber;

public class SwerveModule extends SubsystemBase{

    TalonFX m_driveMotor;
    TalonFX m_turnMotor;

    TalonFXConfiguration m_driveConfig;
    TalonFXConfiguration m_turnConfig;

    CANcoder m_absoluteEncoder;
    CANcoderConfiguration m_canCoderConfig;

    NetworkTableInstance inst;
    NetworkTable table;

    TunableNumber kPInputTurn;
    TunableNumber kIInputTurn;
    TunableNumber kDInputTurn;

    TunableNumber kPInputDrive;
    TunableNumber kIInputDrive;
    TunableNumber kDInputDrive;

    final PositionVoltage m_turnRequest = new PositionVoltage(0).withSlot(0);
    final VelocityVoltage m_driveRequest = new VelocityVoltage(0).withSlot(0);

    SwerveModuleState m_moduleState;

    SwerveModule(int driveID, int turnID, int absEncoderPort, double absEncoderOffset){
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Tunable Numbers");

        kPInputTurn = new TunableNumber("/Tunable Numbers/kPInput Turn", Constants.SwerveConstants.k_turnKP);
        kIInputTurn = new TunableNumber("/Tunable Numbers/kIInput Turn", Constants.SwerveConstants.k_turnKI);
        kDInputTurn = new TunableNumber("/Tunable Numbers/kDInput Turn", Constants.SwerveConstants.k_turnKD);

        kPInputDrive = new TunableNumber("/Tunable Numbers/kPInput Drive", Constants.SwerveConstants.k_driveKP);
        kIInputDrive = new TunableNumber("/Tunable Numbers/kIInput Drive", Constants.SwerveConstants.k_driveKI);
        kDInputDrive = new TunableNumber("/Tunable Numbers/kDInput Drive", Constants.SwerveConstants.k_driveKD);


        m_driveMotor = new TalonFX(driveID, new CANBus("GertrudeGreyser"));
        m_turnMotor = new TalonFX(turnID, new CANBus("GertrudeGreyser"));

        m_absoluteEncoder = new CANcoder(absEncoderPort, new CANBus("GertrudeGreyser"));
        m_canCoderConfig = new CANcoderConfiguration();

        m_canCoderConfig.MagnetSensor.MagnetOffset = absEncoderOffset;
        m_absoluteEncoder.getConfigurator().apply(m_canCoderConfig);

        m_driveConfig = new TalonFXConfiguration(); 
        m_turnConfig = new TalonFXConfiguration();

        //TODO: tune all drive PID values

        m_driveConfig.Slot0.kP = Constants.SwerveConstants.k_driveKP;
        m_driveConfig.Slot0.kI = Constants.SwerveConstants.k_driveKI;
        m_driveConfig.Slot0.kD = Constants.SwerveConstants.k_driveKD;

        m_turnConfig.Slot0.kP = Constants.SwerveConstants.k_turnKP;
        m_turnConfig.Slot0.kI = Constants.SwerveConstants.k_turnKI;
        m_turnConfig.Slot0.kD = Constants.SwerveConstants.k_turnKD;

        m_turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        m_turnConfig.Feedback.RotorToSensorRatio = Constants.SwerveConstants.k_turnGearRatio; 
        m_turnConfig.Feedback.FeedbackRemoteSensorID = m_absoluteEncoder.getDeviceID();

        m_driveConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.k_driveGearRatio;

        m_turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

        m_turnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        m_driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


//TODO: play with current limit
        m_driveConfig.CurrentLimits.StatorCurrentLimit = 120;

        m_driveMotor.getConfigurator().apply(m_driveConfig);
        m_turnMotor.getConfigurator().apply(m_turnConfig);
    }

     @Override
    public void periodic(){
        if(DriverStation.isTestEnabled() && kPInputTurn.hasChanged(hashCode())){
            m_turnConfig.Slot0.kP = kPInputTurn.getAsDouble();
            m_turnMotor.getConfigurator().apply(m_turnConfig);
        }

        if(DriverStation.isTestEnabled() && kIInputTurn.hasChanged(hashCode())){
            m_turnConfig.Slot0.kI = kIInputTurn.getAsDouble();
            m_turnMotor.getConfigurator().apply(m_turnConfig);
        }

        if(DriverStation.isTestEnabled() && kDInputTurn.hasChanged(hashCode())){
            m_turnConfig.Slot0.kD = kDInputTurn.getAsDouble();
            m_turnMotor.getConfigurator().apply(m_turnConfig);            
        }

        if(DriverStation.isTestEnabled() && kPInputDrive.hasChanged(hashCode())){
            m_driveConfig.Slot0.kP = kPInputDrive.getAsDouble();
            m_driveMotor.getConfigurator().apply(m_driveConfig);
        }

        if(DriverStation.isTestEnabled() && kIInputDrive.hasChanged(hashCode())){
            m_driveConfig.Slot0.kI = kIInputDrive.getAsDouble();
            m_driveMotor.getConfigurator().apply(m_driveConfig);
        }

        if(DriverStation.isTestEnabled() && kDInputDrive.hasChanged(hashCode())){
            m_driveConfig.Slot0.kD = kDInputDrive.getAsDouble();
            m_driveMotor.getConfigurator().apply(m_driveConfig);            
        }
    }

    public void Drive(SwerveModuleState moduleState){
        m_moduleState = moduleState;
        m_moduleState.optimize(this.getAngleRotation2d());
        m_moduleState.speedMetersPerSecond *= m_moduleState.angle.minus(this.getAngleRotation2d()).getCos();
        m_driveMotor.setControl(m_driveRequest.withVelocity(m_moduleState.speedMetersPerSecond));
        m_turnMotor.setControl(m_turnRequest.withPosition(m_moduleState.angle.getRotations()));
    }

    public double getAnglePositionRot(){
        return m_absoluteEncoder.getPosition().getValueAsDouble();
    }

    public double getDrivePositionRot(){
        return m_driveMotor.getPosition().getValueAsDouble();
    }

    public double getDriveDistanceMeters(){
        return this.getDrivePositionRot()*Constants.SwerveConstants.k_wheelCircumferenceMeters;
    }

    public Rotation2d getAngleRotation2d(){
        return new Rotation2d(m_absoluteEncoder.getPosition().getValueAsDouble() * 2 * Math.PI); 
    }

    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(this.getDriveDistanceMeters(), this.getAngleRotation2d());
    }

    public double getAbsEncoderPositionRot(){
        return m_absoluteEncoder.getPosition().getValueAsDouble();
    }
//TODO: pick which of thse 3 to uses
    public double getDriveVelocityMeterPerSec(){
        return m_driveMotor.getVelocity().getValueAsDouble() * Constants.SwerveConstants.k_wheelCircumferenceMeters;
    }

    public double getDriveVelocityMeterPerSec2(){
        return m_driveMotor.getRotorVelocity().getValueAsDouble() * Constants.SwerveConstants.k_wheelCircumferenceMeters*Constants.SwerveConstants.k_driveGearRatio;
    }

    public double getDriveVelocityMeterPerSec3(){
        return m_driveMotor.getRotorVelocity().getValueAsDouble() * Constants.SwerveConstants.k_wheelCircumferenceMeters/Constants.SwerveConstants.k_driveGearRatio;
    }
}
