// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorFixedLocationCommand extends Command 
{
  double lastTimeStamp = 0;
  double executeTimeStamp = 0;
  double isFinishedTimeStamp = 0;

  private final ElevatorSubsystem Elevator;
  private final Double  Distance;
  final double iLimitDistance = 1;  /*limit ` distance*/
  double RPSsetpoint = 0;
  double errorSum = 0;
  double lastError = 0;
  double setpointDistance= 0;
  double errorSumDistance = 0;
  double lastErrorDistance = 0;
  double TotalDistance= 0;
  double  startingDistance = 0;
  double   currentDistancePosition = 0;
  double   limitSpeed = 0.5;

  double outputSpeed = 0;
  boolean isForward = true;
  double errorDistance = 7;
  int i = 0;

  /** Creates a new ElevatorUpandDownCommand. */
  public ElevatorFixedLocationCommand(ElevatorSubsystem Elevator, Double Distance) 
  {
    this.Elevator = Elevator;
    this.Distance = Distance;
    
    addRequirements(Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {


    Elevator.setElevatorBrake();

    lastTimeStamp = Timer.getFPGATimestamp();
        //target -180    0      forward     tar < current forward
    //target -220  -180     forward     tar < current  forward
    //        0     -180    backward    tar > current  back 
    //        -180   -220   backward    tar > current  back 

  
    errorSumDistance = 0;
    lastErrorDistance =0;
    setpointDistance = this.Distance*(120/9/25.4);  //motor revolutions per inch

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

      /*************************/
    /*kP                     */
    /*************************/
;
    currentDistancePosition = Elevator.getPosition();

   
    errorDistance = (setpointDistance - currentDistancePosition);  


    if (Math.abs(errorDistance) < 1)  //deadband  
    {
         errorDistance = 0;
   
    }





    /*************************/
    /*kI                    */
    /*************************/
    double dt  = Timer.getFPGATimestamp() - lastTimeStamp;
    //  errorSum = 0;
    // if error is < 1 meter than errorSum is calcuated for kI

    if (Math.abs(errorDistance) < iLimitDistance)   //only executes when error less than 1 meter    
    {
      errorSumDistance += errorDistance * dt;

    }

    /*************************/
    /*kD                     */
    /*************************/
    double errorRateDistance = (errorDistance - lastErrorDistance)/dt;

    /*************************/
    /*kP kI kD Calculations  */
    /*************************/

    double outputSpeed =  Elevator.kP * errorDistance + Elevator.kI * errorSumDistance + Elevator.kD * errorRateDistance;
   
        
    if (outputSpeed > limitSpeed  )     
    {
      outputSpeed = limitSpeed ;
    }
    else if (outputSpeed < -limitSpeed ) 
    {
      outputSpeed = -limitSpeed ;
   
    }

    SmartDashboard.putNumber("Elevator OutputSpeed" ,outputSpeed);
      
    Elevator.setElevatorSpeed(outputSpeed);
    Elevator.setElevatorBrake();



    SmartDashboard.putNumber("Elevator setPoint inches" ,setpointDistance);
    SmartDashboard.putNumber("Elevator our distance motor" , currentDistancePosition);
    SmartDashboard.putNumber("Elevator Error Distance" ,errorDistance);
    SmartDashboard.putNumber("Elevator ErrorDistanceSum" ,errorSumDistance);
    SmartDashboard.putNumber("Elevator ErrorRate",errorRateDistance);

       
    /*setup for next execution */
    lastTimeStamp = Timer.getFPGATimestamp();
    lastErrorDistance= errorDistance;
  }  
    

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    isFinishedTimeStamp = Timer.getFPGATimestamp();
    Elevator.lastPosition = this.currentDistancePosition;
    Elevator.setElevatorSpeed(0);
   Elevator.setElevatorBrake();
    
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
   
      if (Math.abs(errorDistance) < 1.5 )
    {
          Elevator.setElevatorSpeed(0.0);
          Elevator.setElevatorDisable();
          return true;
    }        
    else
    {    
       
      return false;


    }


  
  }
}
