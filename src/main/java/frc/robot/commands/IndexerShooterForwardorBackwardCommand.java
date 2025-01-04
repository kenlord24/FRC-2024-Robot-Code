// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ATRollerSubsystem;
import frc.robot.subsystems.Analog0Subsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;



public class IndexerShooterForwardorBackwardCommand  extends Command 
{
  private final BlinkinSubsystem    BlinkinSubsystem;
  private final Analog0Subsystem    Analog0Subsystem;  
  private final IntakeSubsystem     IntakeSubsystem;
  private final ATRollerSubsystem   ATRollerSubsystem;
  private final IndexerSubsystem    IndexerSubsystem;
  private final ShooterSubsystem    ShooterSubsystem;

  private double    IntakePercentOutput;
  private double    IndexerPercentOutput;
  private double    ATRollerPercentOutput;
  
  private double    ShooterA_RPM;
  private double    ShooterB_RPM;
  private boolean   isIntakeInverted;

  private boolean   isATRollerInverted;

  private boolean   isIndexerInverted;
  private boolean   isShooterAInverted;
  private boolean   isShooterBInverted;
  private boolean   isAnalogActivated;



  /** Creates a new IntakeATRollerForwardorBackwardCommand. */
  public IndexerShooterForwardorBackwardCommand(BlinkinSubsystem  BlinkinSubsystem,
                                                Analog0Subsystem  Analog0Subsystem,  
                                                IntakeSubsystem  IntakeSubsystem,
                                                double IntakePercentOutput,
                                                boolean isIntakeInverted,

                                                ATRollerSubsystem  ATRollerSubsystem,
                                                double ATRollerPercentOutput,
                                                boolean isATRollerInverted,
                                                
                                                IndexerSubsystem  IndexerSubsystem,
                                                double IndexerPercentOutput,
                                                boolean isIndexerInverted,

                                                ShooterSubsystem  ShooterSubsystem,
                                                double ShooterA_RPM,
                                                boolean isShooterAInverted,
                                                double ShooterB_RPM,
                                                boolean isShooterBInverted,
                                                boolean isAnalogActivated)
  {                                              
        
    this.BlinkinSubsystem = BlinkinSubsystem;
    this.Analog0Subsystem = Analog0Subsystem;  
 
    this.ATRollerSubsystem   =  ATRollerSubsystem;
    this.ATRollerPercentOutput = ATRollerPercentOutput;
    this.isATRollerInverted      = isATRollerInverted;

    this.IntakeSubsystem   = IntakeSubsystem;
    this.IndexerPercentOutput = IntakePercentOutput;
    this.isIntakeInverted  = isIntakeInverted;
    
    
    this.IndexerSubsystem   = IndexerSubsystem;
    this.IndexerPercentOutput = IndexerPercentOutput;
    this.isIndexerInverted  = isIndexerInverted;

    this.ShooterSubsystem = ShooterSubsystem;
    this.ShooterA_RPM = ShooterA_RPM;
    this.isShooterAInverted  = isShooterAInverted;
    this.ShooterB_RPM = ShooterB_RPM;
    this.isShooterBInverted  = isShooterBInverted;
    this.isAnalogActivated  = isAnalogActivated;
    
    addRequirements(IndexerSubsystem);
    addRequirements(ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
   

    IntakeSubsystem.Invert(isIntakeInverted);
    IndexerSubsystem.Invert(isIndexerInverted);
    ATRollerSubsystem.Invert(isATRollerInverted);
    ShooterSubsystem.setShooterInverted(isShooterAInverted, isShooterBInverted);
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
    {
   
      if(IndexerPercentOutput > 0 || ShooterA_RPM > 0 || ShooterB_RPM> 0)
      {
   
      ShooterSubsystem.setShooterRPS(ShooterA_RPM , ShooterB_RPM );
        
        if(ShooterSubsystem.getShooterVelocity() > 50.0)
        {
            IndexerSubsystem.setIndexerSpeed(IndexerPercentOutput);
            ATRollerSubsystem.setATRollerPower(ATRollerPercentOutput);
            IntakeSubsystem.setIntakePower(IntakePercentOutput);
        
        }
               
      }
      else
      {
          //isOn = false 
        IntakeSubsystem.setIntakeCoast();  
        ATRollerSubsystem.setATRollerCoast();  
        IndexerSubsystem.setIndexerCoast();
        ShooterSubsystem.setShooterCoast();
        

      }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
     IntakeSubsystem.setIntakeCoast();
     ATRollerSubsystem.setATRollerCoast();  
     IndexerSubsystem.setIndexerCoast();
     ShooterSubsystem.setShooterCoast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    if(IndexerPercentOutput== 0 && ShooterA_RPM == 0  && ShooterA_RPM == 0)
    {
      return true;
    }
    else if(isAnalogActivated)
    {
        if(!Analog0Subsystem.isAnalogIntakeSwitchClosed())
        {
            IntakeSubsystem.setIntakeCoast();
            ATRollerSubsystem.setATRollerCoast();  
            IndexerSubsystem.setIndexerCoast();
            ShooterSubsystem.setShooterCoast();
            return true;
        }
    }return false;
  }
  
}
