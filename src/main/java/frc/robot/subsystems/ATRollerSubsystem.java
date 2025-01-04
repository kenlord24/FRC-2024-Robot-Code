// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;


public class ATRollerSubsystem extends SubsystemBase 
{

  private static final int deviceID = 10;
  private CANSparkMax m_motor;
  private SparkPIDController m_pidController;
  
  public double RPMMultipler;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  /** Creates a new IntakeSubsystem. */
  public ATRollerSubsystem() 
  {
      // initialize motor
      m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
      m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
       
      m_motor.restoreFactoryDefaults();

      m_pidController = m_motor.getPIDController();

      RPMMultipler = .5;
      // PID coefficients
      kP = .4; 
      kI = 0;
      kD = 0; 
      kIz = 0; 
      kFF = 0; 
      kMaxOutput = 1; 
      kMinOutput = -1;
      maxRPM = 5700;

      // set PID coefficients
      m_pidController.setP(kP);
      m_pidController.setI(kI);
      m_pidController.setD(kD);
      m_pidController.setIZone(kIz);
      m_pidController.setFF(kFF);
      m_pidController.setOutputRange(kMinOutput, kMaxOutput);

      // // display PID coefficients on SmartDashboard
      // SmartDashboard.putNumber("AT P Gain", kP);
      // SmartDashboard.putNumber("AT I Gain", kI);
      // SmartDashboard.putNumber("AT D Gain", kD);
      // SmartDashboard.putNumber("AT I Zone", kIz);
      // SmartDashboard.putNumber("AT Feed Forward", kFF);
      // SmartDashboard.putNumber("AT Max Output", kMaxOutput);
      // SmartDashboard.putNumber("AT Min Output", kMinOutput);
      
     

}


public  CANSparkMax getATRollerMotor()
{

   return m_motor;

}



public void setATRollerVoltage()
{

   m_motor.setVoltage(kMaxOutput);

}

public void setATRollerPower(double Power)
{

  m_motor.set(Power);
  SmartDashboard.putNumber("AT Power", Power);
  
}

public void Invert(boolean isInvert)
{
  m_motor.setInverted(isInvert);

}
public void setATRollerCoast()
{
    m_motor.setIdleMode(IdleMode.kCoast);
}

public void setATRollerBrake()
{
    m_motor.setIdleMode(IdleMode.kBrake);
}
 

@Override
  public void periodic() 
  {
        // // read PID coefficients from SmartDashboard
        // double p = SmartDashboard.getNumber("P Gain", 0);
        // double i = SmartDashboard.getNumber("I Gain", 0);
        // double d = SmartDashboard.getNumber("D Gain", 0);
        // double iz = SmartDashboard.getNumber("I Zone", 0);
        // double ff = SmartDashboard.getNumber("Feed Forward", 0);
        // double max = SmartDashboard.getNumber("Max Output", 0);
        // double min = SmartDashboard.getNumber("Min Output", 0);
        // double rpmMult = SmartDashboard.getNumber("Intake RPMMultiplier", 0);
        // SmartDashboard.getNumber("Intake RPM", m_encoder.getVelocity());
        // SmartDashboard.getNumber("Intake RPMMultiplier", RPMMultipler);
    
        // // if PID coefficients on SmartDashboard have changed, write new values to controller
        // if(rpmMult != RPMMultipler) 
        // {
        //   RPMMultipler = rpmMult; 
        // }

        // if((p != kP)) 
        // { 
        //   m_pidController.setP(p); 
        //   kP = p; 
        // }
        // if((i != kI)) 
        // { 
        //   m_pidController.setI(i); 
        //   kI = i; 
        // }
        // if((d != kD)) { m_pidController.setD(d); kD = d; }
        // if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        // if((ff != kFF)) 
        // {
        //    m_pidController.setFF(ff); kFF = ff; 
        // }
        // if((max != kMaxOutput) || (min != kMinOutput)) 
        // { 
        //   m_pidController.setOutputRange(min, max); 
        //   kMinOutput = min; kMaxOutput = max; 
        // }
  }
}