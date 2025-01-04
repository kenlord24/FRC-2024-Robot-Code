// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.Analog0Subsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.ATRollerSubsystem;
import frc.robot.subsystems.IndexerSubsystem;



public class ATStagingForwardorBackwardTimedCommand extends Command 
{
  private final BlinkinSubsystem  BlinkinSubsystem;
  private final Analog0Subsystem  Analog0Subsystem;  
  private final ATRollerSubsystem   ATRollerSubsystem;
  private final IndexerSubsystem   IndexerSubsystem;
 
   private double IndexerPercentOutput;
     private double  ATRollerPercentOut;


  private boolean isATRollerInverted;
  private boolean isIndexerInverted;

  private boolean isAnalogActivated;
  private double  secondsToRun;
  double ExecuteTimeStamp = 0;
  double CurrentTimeStamp = 0;


  /** Creates a newIntakeATRollerIndexerForwardorBackwardCommand. */
  public ATStagingForwardorBackwardTimedCommand(BlinkinSubsystem  BlinkinSubsystem,
                                                Analog0Subsystem  Analog0Subsystem,  

                                                ATRollerSubsystem    ATRollerSubsystem,
                                                double   ATRollerPercentOut,
                                                boolean isATRollerInverted,

                                                IndexerSubsystem  IndexerSubsystem,
                                                double IndexerPercentOutput,                                                
                                                boolean isIndexerInverted,

                                                boolean isAnalogActivated,
                                                double secondsToRun)
  {                                              
        
    this.BlinkinSubsystem = BlinkinSubsystem;
    this.Analog0Subsystem = Analog0Subsystem;  

    
    this.ATRollerSubsystem  = ATRollerSubsystem;
    this.ATRollerPercentOut = ATRollerPercentOut;
    this.isATRollerInverted = isATRollerInverted ;  
    
    this.IndexerSubsystem   = IndexerSubsystem;
    this.IndexerPercentOutput = IndexerPercentOutput;
    this.isIndexerInverted  = isIndexerInverted;
    
    this.isAnalogActivated  = isAnalogActivated;
    this.secondsToRun = secondsToRun;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ATRollerSubsystem);
    addRequirements(IndexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    IndexerSubsystem.Invert(isIndexerInverted);
    ATRollerSubsystem.Invert(isATRollerInverted);

    IndexerSubsystem.setIndexerCoast();
    ATRollerSubsystem.setATRollerCoast();
    ExecuteTimeStamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 

    {
   
        IndexerSubsystem.setIndexerSpeed(IndexerPercentOutput);
        ATRollerSubsystem.setATRollerPower(ATRollerPercentOut);
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
           
      IndexerSubsystem.setIndexerSpeed(0);
      ATRollerSubsystem.setATRollerPower(.0);
      IndexerSubsystem.setIndexerBrake();
      ATRollerSubsystem.setATRollerBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
     //returns in seconds.
    if((Timer.getFPGATimestamp() -  ExecuteTimeStamp )> secondsToRun) 
    {
      return true;
    } 
    return false;
  }
 
  
}
