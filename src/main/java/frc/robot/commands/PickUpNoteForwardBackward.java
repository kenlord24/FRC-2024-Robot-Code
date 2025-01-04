// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Analog0Subsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ATRollerSubsystem;
import frc.robot.subsystems.IndexerSubsystem;



public class PickUpNoteForwardBackward extends Command 
{
  private final BlinkinSubsystem  BlinkinSubsystem;
  private final Analog0Subsystem  Analog0Subsystem;  
  private final IntakeSubsystem   IntakeSubsystem;
  private final ATRollerSubsystem   ATRollerSubsystem;
  private final IndexerSubsystem   IndexerSubsystem;

  private double IntakePower;  
  private double ATRollerPower;  
  private double IndexerPercentOutput;

  private boolean isIntakeInverted;
  private boolean isATRollerInverted;
  private boolean isIndexerInverted;

  private boolean isAnalogActivated;



  /** Creates a newIntakeATRollerIndexerForwardorBackwardCommand. */
  public PickUpNoteForwardBackward(BlinkinSubsystem  BlinkinSubsystem,
                                                Analog0Subsystem  Analog0Subsystem,  
                                                IntakeSubsystem   IntakeSubsystem,
                                                double IntakePower,
                                                boolean isIntakeInverted,

                                                ATRollerSubsystem    ATRollerSubsystem,
                                                double  ATRollerPower,
                                                boolean isATRollerInverted,

                                                IndexerSubsystem  IndexerSubsystem,
                                                double IndexerPercentOutput,
                                                boolean isIndexerInverted,
                                                boolean isAnalogActivated)
  {                                              
        
    this.BlinkinSubsystem = BlinkinSubsystem;
    this.Analog0Subsystem = Analog0Subsystem;  

    this.IntakeSubsystem    = IntakeSubsystem;  
    this.IntakePower        = IntakePower ;
    this.isIntakeInverted   = isIntakeInverted;
    
    this.ATRollerSubsystem  = ATRollerSubsystem;
    this.ATRollerPower      = ATRollerPower ;
    this.isATRollerInverted = isATRollerInverted ;  
    
    this.IndexerSubsystem   = IndexerSubsystem;
    this.IndexerPercentOutput = IndexerPercentOutput;
    this.isIndexerInverted  = isIndexerInverted;
    
    this.isAnalogActivated  = isAnalogActivated;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(IntakeSubsystem);
    addRequirements(ATRollerSubsystem);
    addRequirements(IndexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    IntakeSubsystem.Invert(isIntakeInverted);
    IndexerSubsystem.Invert(isIndexerInverted);
    ATRollerSubsystem.Invert(isATRollerInverted);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
    {
   
      if(IntakePower > 0 || IndexerPercentOutput > 0)
      {
        IntakeSubsystem.setIntakePower(IntakePower);
        IndexerSubsystem.setIndexerSpeed(IndexerPercentOutput);
        ATRollerSubsystem.setATRollerPower(ATRollerPower);
        
               
      }
      else
      {
          //isOn = false 
        IntakeSubsystem.setIntakeCoast();
        IndexerSubsystem.setIndexerCoast();
        ATRollerSubsystem.setATRollerCoast();
        

      }

      if(Analog0Subsystem.isAnalogIntakeSwitchClosed())
      {
          BlinkinSubsystem.blinkinGreenSet();   
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

          IntakeSubsystem.setIntakePower(0);
          IndexerSubsystem.setIndexerSpeed(0);
          ATRollerSubsystem.setATRollerPower(0);
                  
           IntakeSubsystem.setIntakeCoast();
           IndexerSubsystem.setIndexerBrake();
           ATRollerSubsystem.setATRollerBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
   
   
    return false;
  }
  
}
