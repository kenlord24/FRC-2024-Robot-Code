// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ATRollerSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class TurnOffAllMotorCommand extends Command 
{

  private final IntakeSubsystem         IntakeSubsystem;
  private final ATRollerSubsystem       ATRollerSubsystem;
  private final IndexerSubsystem        IndexerSubsystem;

  private final BlinkinSubsystem  BlinkinSubsystem;


  private final ShooterSubsystem        ShooterSubsystem;
  /** Creates a new TurnOffAllMotorCommand. */
  public TurnOffAllMotorCommand(IntakeSubsystem     IntakeSubsystem,
                                ATRollerSubsystem   ATRollerSubsystem,
                                IndexerSubsystem    IndexerSubsystem,
                                ShooterSubsystem        ShooterSubsystem,
                                BlinkinSubsystem  BlinkinSubsystem)

  {                                              
    this.IntakeSubsystem          = IntakeSubsystem;     
    this.ATRollerSubsystem        = ATRollerSubsystem;
    this.IndexerSubsystem         = IndexerSubsystem;
    this.BlinkinSubsystem = BlinkinSubsystem;

    this.ShooterSubsystem         = ShooterSubsystem;
;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(IntakeSubsystem);
    addRequirements(ATRollerSubsystem);
    addRequirements(IndexerSubsystem);

    addRequirements(ShooterSubsystem );
  }                              // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
    IntakeSubsystem.setIntakePower(0);
    IntakeSubsystem.setIntakeCoast();


    IndexerSubsystem.setIndexerCoast();
    IndexerSubsystem.setIndexerSpeed(0);

    ATRollerSubsystem.setATRollerCoast();
    ATRollerSubsystem.setATRollerPower(0);
    
    
    ShooterSubsystem.setShooterRPS(0, 0);
    ShooterSubsystem.setShooterCoast();
    
    BlinkinSubsystem.blinkinYellowSet();   

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return true;
  }
}
