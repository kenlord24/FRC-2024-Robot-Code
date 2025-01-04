// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPitchSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class ShooterPitchFixedLocationCommand extends Command 
{
  double lastTimeStamp = 0;
  double executeTimeStamp = 0;
  double isFinishedTimeStamp = 0;

  private final ShooterPitchSubsystem ShooterPitch;
  private final Double  Distance;
  final double iLimitDistance = 0.02;  /*limit ` distance*/
  double RPSsetpoint = 0;
  double errorSum = 0;
  double lastError = 0;
  double setpointDistance= 0;
  double errorSumDistance = 0;
  double lastErrorDistance = 0;
  double TotalDistance= 0;
  double  startingDistance = 0;
  double   currentDistancePosition = 0;
  double   limitSpeed = 0.2;

  double outputSpeed = 0;
  boolean isForward = true;
  double errorDistance = 0;
  int i = 0;

  /** Creates a new ShooterPitchFixedLocationCommand. */
  public ShooterPitchFixedLocationCommand(ShooterPitchSubsystem ShooterPitch, Double Distance) 
  {
    this.ShooterPitch = ShooterPitch;
    this.Distance = Distance;
    
    addRequirements(ShooterPitch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    ShooterPitch.setShooterPitchBrake();

    lastTimeStamp = Timer.getFPGATimestamp();
        //target -180    0      forward     tar < current forward
    //target -220  -180     forward     tar < current  forward
    //        0     -180    backward    tar > current  back 
    //        -180   -220   backward    tar > current  back 
  
    errorSumDistance = 0;
    lastErrorDistance =0;
    setpointDistance = this.Distance;  //motor revolutions per inch

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

      /*************************/
    /*kP                     */
    /*************************/
;
    currentDistancePosition = ShooterPitch.getPosition();

   
    errorDistance = (setpointDistance - currentDistancePosition);  


    if (Math.abs(errorDistance) < .0001)  //deadband  
    {
       errorDistance = 0;
   
    }





    /*************************/
    /*kI                    */
    /*************************/
    double dt  = Timer.getFPGATimestamp() - lastTimeStamp;
    errorSum = 0;
   
    if (Math.abs(errorDistance) < iLimitDistance)   //.01
    {
      errorSumDistance += errorDistance * dt;

    }

    // /*************************/
    // /*kD                     */
    // /*************************/
    double errorRateDistance = (errorDistance - lastErrorDistance)/dt;

    /*************************/
    /*kP kI kD Calculations  */
    /*************************/

    double outputSpeed =  ShooterPitch.kP * errorDistance + ShooterPitch.kI * errorSumDistance + ShooterPitch.kD * errorRateDistance;
   
        
    if (outputSpeed > limitSpeed  )  
    {
      outputSpeed = limitSpeed ;
    
    }
    if (outputSpeed < -limitSpeed ) 
    {
        outputSpeed = -limitSpeed;
   
    }


    ShooterPitch.setShooterPitchSpeed(outputSpeed);

    // if (Math.abs(errorDistance) < .0001 )
    // {
    //        ShooterPitch.setShooterPitchSpeed(0.0);  //0.001
    // }   
    // else
    // {
    //         ShooterPitch.setShooterPitchSpeed(outputSpeed);
    // } 
    SmartDashboard.putNumber("ShooterPitch setPoint inches" ,setpointDistance);  
    SmartDashboard.putNumber("ShooterPitch OutputSpeed" ,outputSpeed);

       
    /*setup for next execution */
    lastTimeStamp = Timer.getFPGATimestamp();
    lastErrorDistance= errorDistance;
  }  
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    isFinishedTimeStamp = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("ShooterPitch isFinished", isFinishedTimeStamp);
    ShooterPitch.lastPosition = this.currentDistancePosition;
    ShooterPitch.setShooterPitchSpeed(0);
    ShooterPitch.setShooterPitchBrake();
    
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
   
     
    return false;
  }
}
