// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Analog0Subsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.IntakeSubsystem;



public class IntakeCommand extends Command 
{
  /** Creates a new IntakeCommand. */

  private final IntakeSubsystem IntakeSubsystem;
  private final BlinkinSubsystem BlinkinSubsystem;
  private final Analog0Subsystem Analog0Subsystem;
  public  boolean isOn;
  public IntakeCommand(IntakeSubsystem IntakeSubsystem, 
                       BlinkinSubsystem BlinkinSubsystem, 
                       Analog0Subsystem Analog0Subsystem,
                        boolean isOn) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Analog0Subsystem = Analog0Subsystem;
    this.IntakeSubsystem = IntakeSubsystem;
    this.BlinkinSubsystem = BlinkinSubsystem;
    this.isOn = isOn;
    addRequirements(IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //true 
    if(isOn)
    {
      IntakeSubsystem.setIntakePower(.3);
    }
    else
    {
        //isOn = false 
      IntakeSubsystem.setIntakeCoast();
   
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(isOn == false)
    {
      return true;
    }
    else if(Analog0Subsystem.isAnalogIntakeSwitchClosed())
     {
         BlinkinSubsystem.blinkinGreenSet();              
         IntakeSubsystem.setIntakeCoast();
         return true;
     }
    return false;
  }
}
