package frc.robot.subsystems;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TunableNumber;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

 	NetworkTableInstance inst;
  	NetworkTable table;
  	//DoubleEntry IntakePIDDifference; 
	//DoubleEntry IntakeMotorVoltage; 
    DoubleEntry DeployPIDDifference;
    DoubleEntry DeployMotorVoltage; 
    DoubleEntry DeployPIDFollowerDifference;
    DoubleEntry DeployMotorFollowerVoltage;
    DoubleEntry deployMotorPosition;
    DoubleEntry DeployCommandedPosition;
    //TunableNumber m_tunableIntakeP;
    //TunableNumber m_tunableIntakeI;
    //TunableNumber m_tunableIntakeD;
    TunableNumber m_tunableDeployIntakeP;
    TunableNumber m_tunableDeployIntakeI;
    TunableNumber m_tunableDeployIntakeD;
    CANcoder m_intakeAbsoluteEncoder;
    CANcoderConfiguration m_intakeCanCoderConfig;
    //TalonFX m_intakeMotor;
    //TalonFX m_intakeFollowerMotor;
    TalonFX m_intakeDeployMotor;
    TalonFX m_intakeDeployMotorFollower;
    //TalonFXConfiguration m_intakeConfig;
    TalonFXConfiguration m_deployConfig;
    //final VelocityVoltage m_intakeRequest = new VelocityVoltage(0).withSlot(0);
    final PositionVoltage m_intakeDeployRequest = new PositionVoltage(0).withSlot(0);
    final DutyCycleOut m_intakeDeployDutyCycle = new DutyCycleOut(0);
    double m_intakeSetpoint;

    PWMTalonFX m_PWMintakeMotor;
    PWMTalonFX m_PWMintakeFollowerMotor;


    TalonFX intakeMotor;

        public IntakeSubsystem(){
            inst = NetworkTableInstance.getDefault();
            table = inst.getTable("Intake Info");

            m_intakeAbsoluteEncoder = new CANcoder(Constants.IntakeConstants.k_absEncoderPortIntake);
            m_intakeCanCoderConfig = new CANcoderConfiguration();

            m_intakeCanCoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.k_intakeDeployEncoderOffset;

            m_PWMintakeMotor = new PWMTalonFX(Constants.IntakeConstants.k_intakeMotorPWMPort); 
            m_PWMintakeFollowerMotor = new PWMTalonFX(Constants.IntakeConstants.k_intakeFollowerPWMPort);

            //m_intakeMotor = new TalonFX(Constants.IntakeConstants.k_intakeMotorCANID); //Need to getCANID
            //m_intakeFollowerMotor = new TalonFX(Constants.IntakeConstants.k_intakeFollowerCANID);
            m_intakeDeployMotor = new TalonFX(Constants.IntakeConstants.k_intakeDeployMotorCANID); 
            m_intakeDeployMotorFollower = new TalonFX(Constants.IntakeConstants.k_intakeDeployMotorFollowerCANID);
            
            deployMotorPosition = table.getDoubleTopic("Deploy Position").getEntry(0);
            DeployCommandedPosition = table.getDoubleTopic("Deploy Commanded Position").getEntry(m_intakeSetpoint);

            //m_tunableIntakeP = new TunableNumber("IntakeTuning/IntakeP", Constants.IntakeConstants.k_intakeP);
            //m_tunableIntakeI = new TunableNumber("IntakeTuning/IntakeI", Constants.IntakeConstants.k_intakeI);
            //m_tunableIntakeD = new TunableNumber("IntakeTuning/IntakeD", Constants.IntakeConstants.k_intakeD);
            m_tunableDeployIntakeP = new TunableNumber("IntakeTuning/IntakeDeployP", Constants.IntakeConstants.k_intakeDeployP);
            m_tunableDeployIntakeI = new TunableNumber("IntakeTuning/IntakeDeployI", Constants.IntakeConstants.k_intakeDeployI);
            m_tunableDeployIntakeD = new TunableNumber("IntakeTuning/IntakeDeployD", Constants.IntakeConstants.k_intakeDeployD);

            //m_intakeConfig = new TalonFXConfiguration();
            m_deployConfig = new TalonFXConfiguration();

            m_deployConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            m_deployConfig.Feedback.RotorToSensorRatio = Constants.IntakeConstants.k_deployGearRatio; 
            m_deployConfig.Feedback.FeedbackRemoteSensorID = m_intakeAbsoluteEncoder.getDeviceID();
            m_deployConfig.ClosedLoopGeneral.ContinuousWrap = false;

            //m_intakeConfig.Slot0.kP = Constants.IntakeConstants.k_intakeP;
            //m_intakeConfig.Slot0.kI = Constants.IntakeConstants.k_intakeI;
            //m_intakeConfig.Slot0.kD = Constants.IntakeConstants.k_intakeD;
            //m_intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            //m_intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            //m_intakeConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
            //m_intakeConfig.CurrentLimits.StatorCurrentLimit = 80;

            m_deployConfig.Slot0.kP = Constants.IntakeConstants.k_intakeDeployP;
            m_deployConfig.Slot0.kI = Constants.IntakeConstants.k_intakeDeployI;
            m_deployConfig.Slot0.kD = Constants.IntakeConstants.k_intakeDeployD;
            m_deployConfig.Slot0.kG = Constants.IntakeConstants.k_intakeDeployG;
            m_deployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            m_deployConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
            m_deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_deployConfig.Feedback.SensorToMechanismRatio = 1;
            m_deployConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
            m_deployConfig.CurrentLimits.StatorCurrentLimit = 15;

            //m_intakeMotor.getConfigurator().apply(m_intakeConfig);
            //m_intakeFollowerMotor.getConfigurator().apply(m_intakeConfig);
            m_intakeDeployMotor.getConfigurator().apply(m_deployConfig);
            m_intakeDeployMotorFollower.getConfigurator().apply(m_deployConfig);
            m_intakeAbsoluteEncoder.getConfigurator().apply(m_intakeCanCoderConfig);

            //IntakeMotorVoltage = table.getDoubleTopic("Intake Motor Volated").getEntry(0);
            //IntakePIDDifference = table.getDoubleTopic("Intake PID Difference").getEntry(0);
            DeployMotorVoltage = table.getDoubleTopic("Deploy Motor Volated").getEntry(0);
            DeployPIDDifference = table.getDoubleTopic("Deploy PID Difference").getEntry(0);
            DeployMotorFollowerVoltage = table.getDoubleTopic("Deploy Follower Motor Volated").getEntry(0);
            DeployPIDFollowerDifference = table.getDoubleTopic("Deploy Follower PID Difference").getEntry(0);

            m_intakeSetpoint = m_intakeDeployMotor.getPosition().getValueAsDouble();
        }
    
		@Override
    	public void periodic(){
      		//IntakePIDDifference.set(m_intakeMotor.getClosedLoopError().getValueAsDouble()); 
			//IntakeMotorVoltage.set(m_intakeMotor.getMotorVoltage().getValueAsDouble());

            DeployPIDDifference.set(m_intakeDeployMotor.getClosedLoopError().getValueAsDouble()); 
			DeployMotorVoltage.set(m_intakeDeployMotor.getMotorVoltage().getValueAsDouble());

            DeployPIDFollowerDifference.set(m_intakeDeployMotorFollower.getClosedLoopError().getValueAsDouble()); 
			DeployMotorFollowerVoltage.set(m_intakeDeployMotorFollower.getMotorVoltage().getValueAsDouble());

            deployMotorPosition.set(m_intakeDeployMotor.getPosition().getValueAsDouble());
            DeployCommandedPosition.set(m_intakeSetpoint);

            /*
            if(m_tunableIntakeP.hasChanged(hashCode()) && DriverStation.isTest()){
                m_intakeConfig.Slot0.kP = m_tunableIntakeP.getAsDouble();
                m_intakeMotor.getConfigurator().apply(m_intakeConfig);
            }

            if(m_tunableIntakeI.hasChanged(hashCode()) && DriverStation.isTest()){
                m_intakeConfig.Slot0.kI = m_tunableIntakeI.getAsDouble();
                m_intakeMotor.getConfigurator().apply(m_intakeConfig);
            }

            if(m_tunableIntakeD.hasChanged(hashCode()) && DriverStation.isTest()){
                m_intakeConfig.Slot0.kD = m_tunableIntakeD.getAsDouble();
                m_intakeMotor.getConfigurator().apply(m_intakeConfig);
            }
            */
    	}

        
        public void intake(double intakeSpeed){ // spins the rollers at a speed
        	//m_intakeMotor.setControl(m_intakeRequest.withVelocity(intakeSpeed));
            //m_intakeFollowerMotor.setControl(new Follower(m_intakeMotor.getDeviceID(), MotorAlignmentValue.Opposed)); 
    	    m_PWMintakeMotor.set(intakeSpeed);
            m_PWMintakeFollowerMotor.set(-intakeSpeed); // this speed value should always be the speed of the above times -1
        }

        public void intakeDeploy(double position){ // moves arm to specific pos
            m_intakeSetpoint = position;
            m_intakeDeployMotor.setControl(m_intakeDeployRequest.withPosition(m_intakeSetpoint));
            m_intakeDeployMotorFollower.setControl(new Follower(m_intakeDeployMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }
        public void intakeBrake(){ // holds arm in place?
        	//m_intakeMotor.setControl(m_intakeRequest.withVelocity(Constants.IntakeConstants.k_intakeBrakeSpeedRPS));
            //m_intakeMotor.setControl(new Follower(m_intakeMotor.getDeviceID(), MotorAlignmentValue.Opposed));
            m_intakeDeployMotor.setControl(m_intakeDeployRequest.withPosition(m_intakeSetpoint));
            m_intakeDeployMotorFollower.setControl(new Follower(m_intakeDeployMotor.getDeviceID(), MotorAlignmentValue.Opposed));
            m_PWMintakeMotor.set(0);
            m_PWMintakeFollowerMotor.set(0);

        }

        public void intakeUp(){ // moves arm up?
            m_intakeDeployMotor.setControl(m_intakeDeployDutyCycle.withOutput(0.12));
            m_intakeDeployMotorFollower.setControl(new Follower(m_intakeDeployMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }

        public void intakeDown(){
            m_intakeDeployMotor.setControl(m_intakeDeployDutyCycle.withOutput(-0.07));
            m_intakeDeployMotorFollower.setControl(new Follower(m_intakeDeployMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }

}
