// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.IndexerSubsystem;

public class IndexerCommand extends Command 
{
  /** Creates a new IndexerCommand. */

  private final IndexerSubsystem IndexerSubsystem;
  public  final boolean isOn;
  public IndexerCommand(IndexerSubsystem IndexerSubsystem, boolean isOn) 
  {
    // Use addRequirements() here to declare subsystem dependencies.

    this.IndexerSubsystem = IndexerSubsystem;
    this.isOn = isOn;
    addRequirements(IndexerSubsystem);
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
      IndexerSubsystem.setIndexerSpeed(.3);
    }
    else
    {
      //isOn = false 
     IndexerSubsystem.setIndexerCoast();
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    IndexerSubsystem.setIndexerCoast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
