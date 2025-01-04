// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.BlinkinSubsystem;

import frc.robot.subsystems.ATRollerSubsystem;



public class ATRollerRunTimedCommand extends Command 
{
  /** Creates a new IntakeCommand. */

  private final ATRollerSubsystem ATRollerSubsystem;
  private final BlinkinSubsystem BlinkinSubsystem;

  public final double seconds;
  double ExecuteTimeStamp = 0;
  double CurrentTimeStamp = 0;
  
  public ATRollerRunTimedCommand(ATRollerSubsystem ATRollerSubsystem, 
                         BlinkinSubsystem BlinkinSubsystem, 
                         double seconds)
  {
    // Use addRequirements() here to declare subsystem dependencies.

    this.ATRollerSubsystem  = ATRollerSubsystem;
    this.BlinkinSubsystem   = BlinkinSubsystem;
  
    this.seconds            = seconds;
    addRequirements(ATRollerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    ExecuteTimeStamp = Timer.getFPGATimestamp();
       ATRollerSubsystem.setATRollerCoast();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
 
      ATRollerSubsystem.setATRollerPower(1.0);
      // if(ATRollerSubsystem.isATRolloerSwitchClosed())
      // {
      //     BlinkinSubsystem.blinkinBlueSet();
      //     end(true);
      // }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

      ATRollerSubsystem.setATRollerPower(0.0);
      ATRollerSubsystem.setATRollerCoast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    //returns in seconds.
    if((Timer.getFPGATimestamp() -  ExecuteTimeStamp) > seconds) 
    {
      return true;
    } 
    return false;
  }
}
