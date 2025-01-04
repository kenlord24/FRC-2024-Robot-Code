// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;

public class ShootForwardCommand extends Command {
  /** Creates a new ShootForwardCommand. */

 private final ShooterSubsystem ShooterSubsystem;
 private final BlinkinSubsystem BlinkinSubsystem;
 
  /*********instance variables */
  final double kP = 0.11;
  final double kI = 0.5;
  final double kD = 0.0001;
  final double iLimit = 1;  /*limit 1 meter */
  
  double RPSsetpoint = 0;
  double errorSum = 0;
  double lastTimeStamp = 0;
  double executeTimeStamp = 0;
  double isFinishedTimeStamp = 0;
  double lastError = 0.0;
  Double MotorARPS = 0.0;
  Double MotorBRPS = 0.0;


  public ShootForwardCommand(ShooterSubsystem ShooterSubsystem, 
                            BlinkinSubsystem BlinkinSubsystem,
                            Double MotorARPS, Double MotorBRPS)
                             
  {
      this.ShooterSubsystem = ShooterSubsystem;
      this.BlinkinSubsystem = BlinkinSubsystem;
      this.MotorARPS = MotorARPS;
      this.MotorBRPS = MotorBRPS;
      addRequirements(ShooterSubsystem);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    lastTimeStamp = Timer.getFPGATimestamp();
    errorSum = 0;
    lastError=0;
    RPSsetpoint = this.MotorARPS;
    ShooterSubsystem.setShooterInverted(true,false);
    BlinkinSubsystem.blinkinBlueSet();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

       if(this.MotorARPS > 0 )
       {
          ShooterSubsystem.setShooterRPS(MotorARPS, MotorBRPS);
      
       }
       else
       {
          ShooterSubsystem.setShooterCoast();
       }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(this.MotorARPS > 0)
    {
      return true;
    }
 
    return false;
  }
}
