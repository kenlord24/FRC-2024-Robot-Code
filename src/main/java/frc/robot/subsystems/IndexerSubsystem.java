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


public class IndexerSubsystem extends SubsystemBase 
{

  private static final int deviceID = 16;
  private CANSparkMax m_motor;
  private SparkPIDController m_pidController;
  public double RPMMultipler;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  /** Creates a new  IndexerSubsystem. */
  public IndexerSubsystem() 
  {
      // initialize motor
      m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
      m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
      /**
        * The RestoreFactoryDefaults method can be used to reset the configuration parameters
        * in the SPARK MAX to their factory default state. If no argument is passed, these
        * parameters will not persist between power cycles
        */
      m_motor.restoreFactoryDefaults();

      /**
        * In order to use PID functionality for a controller, a SparkPIDController object
        * is constructed by calling the getPIDController() method on an existing
        * CANSparkMax object
        */
      m_pidController = m_motor.getPIDController();

      RPMMultipler = .5;
      // PID coefficients
      kP = .1; 
      kI = 0;
      kD = 0; 
      kIz = 0; 
      kFF = 0; 
      kMaxOutput = 1; 
      kMinOutput = -1;
      maxRPM = 5700;
      setIndexerCoast();
      // set PID coefficients
      m_pidController.setP(kP);
      m_pidController.setI(kI);
      m_pidController.setD(kD);
      m_pidController.setIZone(kIz);
      m_pidController.setFF(kFF);
      m_pidController.setOutputRange(kMinOutput, kMaxOutput);

      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("Indexer P Gain", kP);
      SmartDashboard.putNumber("Indexer I Gain", kI);
      SmartDashboard.putNumber("Indexer D Gain", kD);
      SmartDashboard.putNumber("Indexer I Zone", kIz);
      SmartDashboard.putNumber("Indexer FF", kFF);
      SmartDashboard.putNumber("Indexer Max Output", kMaxOutput);
      SmartDashboard.putNumber("Indexer Min Output", kMinOutput);
      SmartDashboard.putNumber("Indexer RPMMultiplier", RPMMultipler);
     

}



public  CANSparkMax getIndexerMotor()
{

   return m_motor;

}

public void setIndexerSpeed(double percentOutput)
{
  m_motor.set(percentOutput);
  SmartDashboard.putNumber("Indexer Power", percentOutput);
}

 
public void setIndexerCoast()
{
    m_motor.setIdleMode(IdleMode.kCoast);
}
public void setIndexerBrake()
{
    m_motor.setIdleMode(IdleMode.kBrake);
}


public void Invert(boolean isInvert)
{
  m_motor.setInverted(isInvert);

}
@Override
  public void periodic() 
  {
       
  }
}
