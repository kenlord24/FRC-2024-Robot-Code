// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorUpandDownCommand extends Command 
{


  private final ElevatorSubsystem Elevator;
  private final DoubleSupplier  vY;
  
  double RPSsetpoint = 0;
  double errorSum = 0;
  double lastTimeStamp = 0;
  double executeTimeStamp = 0;
  double isFinishedTimeStamp = 0;
  double lastError = 0;
   double desiredSpeeds = 0;

  /** Creates a new ElevatorUpandDownCommand. */
  public ElevatorUpandDownCommand(ElevatorSubsystem Elevator, DoubleSupplier vY) 
  {
    this.Elevator = Elevator;
    this.vY = vY;
    
    addRequirements(Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
   
    desiredSpeeds = vY.getAsDouble()/10;
  
    Elevator.setElevatorSpeed(desiredSpeeds);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    Elevator.setElevatorSpeed(0);
    Elevator.setElevatorBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    
    return false;
  }
}
