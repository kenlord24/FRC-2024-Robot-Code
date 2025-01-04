// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;



public class Analog1ShooterPitchEncoderSubsystem extends SubsystemBase {
  /** Creates a new Analog0Subsystem. */

 AnalogInput analogShooterPitch = new AnalogInput(1);
 
  public Analog1ShooterPitchEncoderSubsystem() 
  {
 
  }

public double getShooterPitchJoyAverageVoltage()
{
 
     analogShooterPitch.setAverageBits(32);
     return analogShooterPitch.getAverageVoltage();

}  


public Boolean isAnalogShooterPitchClosed()
{
  if(analogShooterPitch.getVoltage() > 1.5)
  {
    return true;
  }
   return false;
}
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
