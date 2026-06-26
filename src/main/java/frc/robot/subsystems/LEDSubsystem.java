
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase{
    final CANdle ledControler = new CANdle(LEDConstants.k_CANdleCANID, "roborio");
   public LEDSubsystem() {
 // Configure the CANdle for basic use
 CANdleConfiguration configs = new CANdleConfiguration();
 // Set the LED strip type and brightness
 configs.LED.StripType = StripTypeValue.GRB;
 configs.LED.BrightnessScalar = 0.5;
 // Disable status LED when being controlled
 configs.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
 
 // Write these configs to the CANdle
  ledControler.getConfigurator().apply(configs);
    ledControler.setControl(new SolidColor(8, 16).withColor(LEDConstants.k_Green));
   }
   
}
