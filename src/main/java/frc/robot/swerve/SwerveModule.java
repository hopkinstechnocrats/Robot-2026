package frc.robot.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TunableNumber;

public class SwerveModule extends SubsystemBase{

    TalonFX m_driveMotor;
    TalonFX m_turnMotor;

    Slot0Configs m_driveConfig;
    Slot0Configs m_turnConfig;

    MotorOutputConfigs m_driveOutputConfigs;
    MotorOutputConfigs m_turnOutputConfigs;
    
    AnalogEncoder m_absoluteEncoder;

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

    SwerveModule(int driveID, int turnID, int absEncoderPort, double absEcoderOffset){
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

        m_absoluteEncoder = new AnalogEncoder(absEncoderPort);

        m_driveConfig = new Slot0Configs();
        m_turnConfig = new Slot0Configs();

        m_driveConfig.kP = Constants.SwerveConstants.k_driveKP;
        m_driveConfig.kI = Constants.SwerveConstants.k_driveKI;
        m_driveConfig.kD = Constants.SwerveConstants.k_driveKD;

        m_turnConfig.kP = Constants.SwerveConstants.k_turnKP;
        m_turnConfig.kI = Constants.SwerveConstants.k_turnKI;
        m_turnConfig.kD = Constants.SwerveConstants.k_turnKD;

        

        m_turnOutputConfigs = new MotorOutputConfigs();
        m_driveOutputConfigs = new MotorOutputConfigs();

        //TODO I have no clue something with inversion
        m_turnOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        m_driveOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        m_driveOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        m_turnOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        m_driveMotor.getConfigurator().apply(m_driveConfig);
        m_driveMotor.getConfigurator().apply(m_driveOutputConfigs);
        m_turnMotor.getConfigurator().apply(m_turnConfig);
        m_turnMotor.getConfigurator().apply(m_turnOutputConfigs);

        m_turnMotor.getConfigurator().setPosition((m_absoluteEncoder.get()-absEcoderOffset)*Constants.SwerveConstants.k_turnGearRatio);
    }

     @Override
    public void periodic(){
        if(DriverStation.isTestEnabled() && kPInputTurn.hasChanged(hashCode())){
            m_turnConfig.kP = kPInputTurn.getAsDouble();
            m_turnMotor.getConfigurator().apply(m_turnConfig);
        }

        if(DriverStation.isTestEnabled() && kIInputTurn.hasChanged(hashCode())){
            m_turnConfig.kI = kIInputTurn.getAsDouble();
            m_turnMotor.getConfigurator().apply(m_turnConfig);
        }

        if(DriverStation.isTestEnabled() && kDInputTurn.hasChanged(hashCode())){
            m_turnConfig.kD = kDInputTurn.getAsDouble();
            m_turnMotor.getConfigurator().apply(m_turnConfig);            
        }

        if(DriverStation.isTestEnabled() && kPInputDrive.hasChanged(hashCode())){
            m_driveConfig.kP = kPInputDrive.getAsDouble();
            m_driveMotor.getConfigurator().apply(m_driveConfig);
        }

        if(DriverStation.isTestEnabled() && kIInputDrive.hasChanged(hashCode())){
            m_driveConfig.kI = kIInputDrive.getAsDouble();
            m_driveMotor.getConfigurator().apply(m_driveConfig);
        }

        if(DriverStation.isTestEnabled() && kDInputDrive.hasChanged(hashCode())){
            m_driveConfig.kD = kDInputDrive.getAsDouble();
            m_driveMotor.getConfigurator().apply(m_driveConfig);            
        }
    }

    public void Drive(SwerveModuleState moduleState){
        m_moduleState = moduleState;
        m_moduleState.optimize(this.getAngleRotation2d());
        m_moduleState.speedMetersPerSecond *= m_moduleState.angle.minus(this.getAngleRotation2d()).getCos();
        m_driveMotor.setControl(m_driveRequest.withVelocity(m_moduleState.speedMetersPerSecond * Constants.SwerveConstants.k_driveGearRatio));
        m_turnMotor.setControl(m_turnRequest.withPosition(m_moduleState.angle.getRotations() * Constants.SwerveConstants.k_turnGearRatio));
    }

    public double getAnglePositionRot(){
        return m_turnMotor.getPosition().getValueAsDouble()/Constants.SwerveConstants.k_turnGearRatio;
    }

    public double getDrivePositionRot(){
        return m_driveMotor.getPosition().getValueAsDouble()/Constants.SwerveConstants.k_driveGearRatio;
    }

    public double getDriveDistanceMeters(){
        return this.getDrivePositionRot()*Constants.SwerveConstants.k_wheelCircumferenceMeters;
    }

    public Rotation2d getAngleRotation2d(){
        return new Rotation2d((m_turnMotor.getPosition().getValueAsDouble()/Constants.SwerveConstants.k_turnGearRatio) * Math.PI * 2 ); 
    }

    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(this.getDriveDistanceMeters(), this.getAngleRotation2d());
    }

    public double getAbsEncoderPositionRot(){
        return m_absoluteEncoder.get();
    }

    public double getDriveVelocityMeterPerSec(){
        return (m_driveMotor.getVelocity().getValueAsDouble()/Constants.SwerveConstants.k_driveGearRatio) * Constants.SwerveConstants.k_wheelCircumferenceMeters;
    }
}
