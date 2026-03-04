package frc.robot.subsystems;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TunableNumber;

public class IntakeSubsystem extends SubsystemBase{

 	NetworkTableInstance inst;
  	NetworkTable table;
  	DoubleEntry IntakePIDDifference; 
	DoubleEntry IntakeMotorVoltage; 
    DoubleEntry DeployPIDDifference;
    DoubleEntry DeployMotorVoltage; 
    DoubleEntry DeployPIDFollowerDifference;
    DoubleEntry DeployMotorFollowerVoltage;
  	TalonFX m_intakeMotor;
    TalonFX m_intakeDeployMotor;
    TalonFX m_intakeDeployMotorFollower;
    TalonFXConfiguration m_intakeConfig;
    TalonFXConfiguration m_deployConfig;
    final VelocityVoltage m_intakeRequest = new VelocityVoltage(0).withSlot(0);
    final PositionVoltage m_intakeDeployRequest = new PositionVoltage(0).withSlot(0);


    TalonFX intakeMotor;

        public IntakeSubsystem(){
            inst = NetworkTableInstance.getDefault();
            table = inst.getTable("Intake Info");

            m_intakeMotor = new TalonFX(Constants.IntakeConstants.k_intakeMotorCANID); //Need to getCANID
            m_intakeDeployMotor = new TalonFX(Constants.IntakeConstants.k_intakeDeployMotorCANID); //TODO:Also needs CANID
            m_intakeDeployMotorFollower = new TalonFX(Constants.IntakeConstants.k_intakeDeployMotorFollowerCANID); //TODO:Also needs CANID

            m_intakeConfig = new TalonFXConfiguration();
            m_deployConfig = new TalonFXConfiguration();

            m_intakeConfig.Slot0.kP = Constants.IntakeConstants.k_intakeP;
            m_intakeConfig.Slot0.kI = Constants.IntakeConstants.k_intakeI;
            m_intakeConfig.Slot0.kD = Constants.IntakeConstants.k_intakeD;
            m_intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            m_deployConfig.Slot0.kP = Constants.IntakeConstants.k_intakeDeployP;
            m_deployConfig.Slot0.kI = Constants.IntakeConstants.k_intakeDeployI;
            m_deployConfig.Slot0.kD = Constants.IntakeConstants.k_intakeDeployD;
            m_deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            m_intakeMotor.getConfigurator().apply(m_intakeConfig);
            m_intakeDeployMotor.getConfigurator().apply(m_deployConfig);
            m_intakeDeployMotorFollower.getConfigurator().apply(m_deployConfig);

            IntakeMotorVoltage = table.getDoubleTopic("Intake Motor Volated").getEntry(0);
            IntakePIDDifference = table.getDoubleTopic("Intake PID Difference").getEntry(0);
            DeployMotorVoltage = table.getDoubleTopic("Deploy Motor Volated").getEntry(0);
            DeployPIDDifference = table.getDoubleTopic("Deploy PID Difference").getEntry(0);
            DeployMotorFollowerVoltage = table.getDoubleTopic("Deploy Follower Motor Volated").getEntry(0);
            DeployPIDFollowerDifference = table.getDoubleTopic("Deploy Follower PID Difference").getEntry(0);
        }
    
		@Override
    	public void periodic(){
      		IntakePIDDifference.set(m_intakeMotor.getClosedLoopError().getValueAsDouble()); 
			IntakeMotorVoltage.set(m_intakeMotor.getMotorVoltage().getValueAsDouble());

            DeployPIDDifference.set(m_intakeDeployMotor.getClosedLoopError().getValueAsDouble()); 
			DeployMotorVoltage.set(m_intakeDeployMotor.getMotorVoltage().getValueAsDouble());

            DeployPIDFollowerDifference.set(m_intakeDeployMotorFollower.getClosedLoopError().getValueAsDouble()); 
			DeployMotorFollowerVoltage.set(m_intakeDeployMotorFollower.getMotorVoltage().getValueAsDouble());
    	}

        public void intake(double intakeSpeed){
        	m_intakeMotor.setControl(m_intakeRequest.withVelocity(intakeSpeed));
        }

        public void intakeDeploy(double position){
            m_intakeDeployMotor.setControl(m_intakeDeployRequest.withPosition(position));
            m_intakeDeployMotorFollower.setControl(new Follower(m_intakeDeployMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }
        public void intakeBrake(){
        	m_intakeMotor.setControl(m_intakeRequest.withVelocity(Constants.IntakeConstants.k_intakeBrakeSpeedRPS));
            m_intakeDeployMotor.setControl(m_intakeDeployRequest.withPosition(m_intakeDeployMotor.getPosition().getValueAsDouble()));
            m_intakeDeployMotorFollower.setControl(new Follower(m_intakeDeployMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }
}
