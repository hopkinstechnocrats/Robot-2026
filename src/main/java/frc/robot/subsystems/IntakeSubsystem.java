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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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
	Slot0Configs m_intakeConfig;
    Slot1Configs m_intakeDeployConfig;
    MotorOutputConfigs m_intakeOutputConfig;
    MotorOutputConfigs m_intakeDeployOutputConfig;
    TunableNumber kIntakeFeedSpeedRPS;
    TunableNumber kIntakeReverseFeedSpeedRPS;
    TunableNumber kPInputIntake;
    TunableNumber kIInputIntake;
    TunableNumber kDInputIntake;
    TunableNumber kPInputDeployIntake;
    TunableNumber kIInputDeployIntake;
    TunableNumber kDInputDeployIntake;
    TunableNumber kSInputDeployIntake;
    TunableNumber kVInputDeployIntake;
    final VelocityVoltage m_intakeRequest = new VelocityVoltage(0).withSlot(0);
    final PositionVoltage m_intakeDeployRequest = new PositionVoltage(0).withSlot(1);
    final PositionVoltage m_intakeUndeployRequest = new PositionVoltage(0).withSlot(1);
    final PositionVoltage m_intakeDeployBobRequest = new PositionVoltage(0).withSlot(1);


    TalonFX intakeMotor;

        public IntakeSubsystem(){
            inst = NetworkTableInstance.getDefault();
            table = inst.getTable("Intake Info");

            kIntakeFeedSpeedRPS = new TunableNumber("/TunableNumbers/Intake Feed Speed RPS", Constants.IntakeConstants.k_intakeSpeedRPS);
            kIntakeReverseFeedSpeedRPS = new TunableNumber("/TunableNumbers/Intake Reverse Feed Speed RPS", Constants.IntakeConstants.k_reverseIntakeSpeedRPS);
            kPInputIntake = new TunableNumber("/Tunable Numbers/k_PInput Intake", Constants.IntakeConstants.k_intakeP);
            kIInputIntake = new TunableNumber("/Tunable Numbers/k_IInput Intake", Constants.IntakeConstants.k_intakeI);
            kDInputIntake = new TunableNumber("/Tunable Numbers/k_DInput Intake", Constants.IntakeConstants.k_intakeD);

            kPInputDeployIntake = new TunableNumber("/Tunable Numbers/kP_Input Deploy Intake", Constants.IntakeConstants.k_intakeDeployP);
            kIInputDeployIntake = new TunableNumber("/Tunable Numbers/kI_Input Deploy Intake", Constants.IntakeConstants.k_intakeDeployI);
            kDInputDeployIntake = new TunableNumber("/Tunable Numbers/kD_Input Deploy Intake", Constants.IntakeConstants.k_intakeDeployD);
            kSInputDeployIntake = new TunableNumber("/Tunable Numbers/kS_Input Deploy Intake", Constants.IntakeConstants.k_intakeDeployS);
            kVInputDeployIntake = new TunableNumber("/Tunable Numbers/kV_Input Deploy Intake", Constants.IntakeConstants.k_intakeDeployFeedForeward);

            m_intakeMotor = new TalonFX(Constants.IntakeConstants.k_intakeMotorCANID); //Need to getCANID
            m_intakeDeployMotor = new TalonFX(Constants.IntakeConstants.k_intakeDeployMotorCANID); //TODO:Also needs CANID
            m_intakeDeployMotorFollower = new TalonFX(Constants.IntakeConstants.k_intakeDeployMotorFollowerCANID); //TODO:Also needs CANID
            m_intakeConfig = new Slot0Configs();
            m_intakeDeployConfig = new Slot1Configs();
            m_intakeOutputConfig = new MotorOutputConfigs();
            m_intakeConfig.kP = Constants.IntakeConstants.k_intakeP;
            m_intakeConfig.kI = Constants.IntakeConstants.k_intakeI;
            m_intakeConfig.kD = Constants.IntakeConstants.k_intakeD;
			m_intakeConfig.kV = Constants.IntakeConstants.k_intakeFeedForward;
            m_intakeDeployOutputConfig = new MotorOutputConfigs();
            m_intakeDeployConfig.kP = Constants.IntakeConstants.k_intakeDeployP;
            m_intakeDeployConfig.kI = Constants.IntakeConstants.k_intakeDeployI;
            m_intakeDeployConfig.kD = Constants.IntakeConstants.k_intakeDeployD;
            m_intakeDeployConfig.kS = Constants.IntakeConstants.k_intakeDeployS;
            m_intakeDeployConfig.kV = Constants.IntakeConstants.k_intakeDeployFeedForeward;

            m_intakeMotor.getConfigurator().apply(m_intakeConfig);
            m_intakeDeployMotor.getConfigurator().apply(m_intakeDeployOutputConfig);
            m_intakeDeployMotorFollower.getConfigurator().apply(m_intakeDeployOutputConfig);

            IntakeMotorVoltage = table.getDoubleTopic("Intake Motor Volated").getEntry(0);
            IntakePIDDifference = table.getDoubleTopic("Intake PID Difference").getEntry(0);
            DeployMotorVoltage = table.getDoubleTopic("Deploy Motor Volated").getEntry(0);
            DeployPIDDifference = table.getDoubleTopic("Deploy PID Difference").getEntry(0);
            DeployMotorFollowerVoltage = table.getDoubleTopic("Deploy Follower Motor Volated").getEntry(0);
            DeployPIDFollowerDifference = table.getDoubleTopic("Deploy Follower PID Difference").getEntry(0);

            final TrapezoidProfile m_intakeDeployProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(80, 160));

            TrapezoidProfile.State m_DeployGoal = new TrapezoidProfile.State(Constants.IntakeConstants.k_intakeSetpointDeploy, 0);
            TrapezoidProfile.State m_DeploySetpoint = new TrapezoidProfile.State();

            TrapezoidProfile.State m_UndeployGoal = new TrapezoidProfile.State(Constants.IntakeConstants.k_intakeSetpointRetract, 0);
            TrapezoidProfile.State m_WhileDeployedSetpoint = new TrapezoidProfile.State();

            TrapezoidProfile.State m_DeployBobGoal = new TrapezoidProfile.State(Constants.IntakeConstants.k_intakeSetpointBob, 0);
            TrapezoidProfile.State m_WhileDeployBobSetpoint = new TrapezoidProfile.State();


            m_DeploySetpoint = m_intakeDeployProfile.calculate(0.020, m_DeploySetpoint, m_DeployGoal);
            m_intakeDeployRequest.Position = m_DeploySetpoint.position;
            m_intakeDeployRequest.Velocity = m_DeploySetpoint.velocity;

            m_WhileDeployedSetpoint = m_intakeDeployProfile.calculate(0.020, m_WhileDeployedSetpoint, m_UndeployGoal);
            m_intakeUndeployRequest.Position = m_WhileDeployedSetpoint.position;
            m_intakeUndeployRequest.Velocity = m_WhileDeployedSetpoint.velocity;

            m_WhileDeployBobSetpoint = m_intakeDeployProfile.calculate(0.020, m_WhileDeployBobSetpoint, m_DeployBobGoal);
            m_intakeDeployBobRequest.Position = m_WhileDeployBobSetpoint.position;
            m_intakeDeployBobRequest.Velocity = m_WhileDeployBobSetpoint.velocity;
        }
    
		@Override
    	public void periodic(){
      		IntakePIDDifference.set(m_intakeMotor.getClosedLoopError().getValueAsDouble()); 
			IntakeMotorVoltage.set(m_intakeMotor.getMotorVoltage().getValueAsDouble());

            DeployPIDDifference.set(m_intakeDeployMotor.getClosedLoopError().getValueAsDouble()); 
			DeployMotorVoltage.set(m_intakeDeployMotor.getMotorVoltage().getValueAsDouble());

            DeployPIDFollowerDifference.set(m_intakeDeployMotorFollower.getClosedLoopError().getValueAsDouble()); 
			DeployMotorFollowerVoltage.set(m_intakeDeployMotorFollower.getMotorVoltage().getValueAsDouble());

            if(DriverStation.isTestEnabled() && kIntakeFeedSpeedRPS.hasChanged(hashCode())){
                m_intakeRequest.Velocity = kIntakeFeedSpeedRPS.getAsDouble();
            }

            if(DriverStation.isTestEnabled() && kPInputIntake.hasChanged(hashCode())){
                m_intakeConfig.kP = kPInputIntake.getAsDouble();
                m_intakeMotor.getConfigurator().apply(m_intakeConfig);
            }

            if(DriverStation.isTestEnabled() && kIInputIntake.hasChanged(hashCode())){
                m_intakeConfig.kI = kIInputIntake.getAsDouble();
                m_intakeMotor.getConfigurator().apply(m_intakeConfig);
            }

            if(DriverStation.isTestEnabled() && kDInputIntake.hasChanged(hashCode())){
                m_intakeConfig.kD = kDInputIntake.getAsDouble();
                m_intakeMotor.getConfigurator().apply(m_intakeConfig);            
            }

            if(DriverStation.isTestEnabled() && kPInputDeployIntake.hasChanged(hashCode())){
                m_intakeDeployConfig.kP = kPInputDeployIntake.getAsDouble();
                m_intakeDeployMotor.getConfigurator().apply(m_intakeDeployConfig);
                m_intakeDeployMotorFollower.getConfigurator().apply(m_intakeDeployConfig);            
            }

            if(DriverStation.isTestEnabled() && kIInputDeployIntake.hasChanged(hashCode())){
                m_intakeDeployConfig.kI = kIInputDeployIntake.getAsDouble();
                m_intakeDeployMotor.getConfigurator().apply(m_intakeDeployConfig);
                m_intakeDeployMotorFollower.getConfigurator().apply(m_intakeDeployConfig);            
            }

            if(DriverStation.isTestEnabled() && kDInputDeployIntake.hasChanged(hashCode())){
                m_intakeDeployConfig.kD = kDInputDeployIntake.getAsDouble();
                m_intakeDeployMotor.getConfigurator().apply(m_intakeDeployConfig);
                m_intakeDeployMotorFollower.getConfigurator().apply(m_intakeDeployConfig);            
            }

            if(DriverStation.isTestEnabled() && kSInputDeployIntake.hasChanged(hashCode())){
                m_intakeDeployConfig.kS = kSInputDeployIntake.getAsDouble();
                m_intakeDeployMotor.getConfigurator().apply(m_intakeDeployConfig);
                m_intakeDeployMotorFollower.getConfigurator().apply(m_intakeDeployConfig);            
            }

            if(DriverStation.isTestEnabled() && kVInputDeployIntake.hasChanged(hashCode())){
                m_intakeDeployConfig.kV = kVInputDeployIntake.getAsDouble();
                m_intakeDeployMotor.getConfigurator().apply(m_intakeDeployConfig);
                m_intakeDeployMotorFollower.getConfigurator().apply(m_intakeDeployConfig);            
            }
    	}

        public void intake(double intakeSpeed){
        	m_intakeMotor.setControl(m_intakeRequest.withVelocity(intakeSpeed));
        }

        public void intakeDeploy(){
            m_intakeDeployMotor.setControl(m_intakeDeployRequest.withPosition(m_intakeDeployRequest.Position).withVelocity(m_intakeDeployRequest.Velocity));
            m_intakeDeployMotorFollower.setControl(m_intakeDeployRequest.withPosition(-m_intakeDeployRequest.Position).withVelocity(-m_intakeDeployRequest.Velocity));
        }
        
        public void intakeUndeploy(){
            m_intakeDeployMotor.setControl(m_intakeUndeployRequest.withPosition(m_intakeUndeployRequest.Position).withVelocity(m_intakeUndeployRequest.Velocity));
            m_intakeDeployMotorFollower.setControl(m_intakeUndeployRequest.withPosition(-m_intakeUndeployRequest.Position).withVelocity(-m_intakeUndeployRequest.Velocity));
        }
        
        public void intakeBob(){
            m_intakeDeployMotor.setControl(m_intakeDeployBobRequest.withPosition(m_intakeDeployBobRequest.Position).withVelocity(m_intakeDeployBobRequest.Velocity));
            m_intakeDeployMotorFollower.setControl(m_intakeDeployBobRequest.withPosition(-m_intakeDeployBobRequest.Position).withVelocity(-m_intakeDeployBobRequest.Velocity));
        }

        public void intakeBrake(){
        	m_intakeMotor.setControl(m_intakeRequest.withVelocity(Constants.IntakeConstants.k_intakeBrakeSpeedRPS));
            m_intakeDeployMotor.setControl(m_intakeDeployRequest.withPosition(Constants.IntakeConstants.k_intakeBrakeSpeedRPS).withVelocity(Constants.IntakeConstants.k_intakeBrakeSpeedRPS));
            m_intakeDeployMotorFollower.setControl(m_intakeDeployRequest.withPosition(Constants.IntakeConstants.k_intakeBrakeSpeedRPS).withVelocity(Constants.IntakeConstants.k_intakeBrakeSpeedRPS));
        }
}