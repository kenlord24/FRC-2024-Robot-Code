/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbJoyStickCommand;
import frc.robot.commands.ElevatorFixedLocationCommand;
import frc.robot.commands.ATRollerRunTimedCommand;
import frc.robot.commands.ATStagingForwardorBackwardTimedCommand;
import frc.robot.commands.TurnOffAllMotorCommand;
import frc.robot.commands.AutoShootToSpeakerTimed;
import frc.robot.commands.AutoShooterPitchFixedLocationTimedCommand;
import frc.robot.commands.AutoPickUpNoteForwardBackwardTimed;
import frc.robot.commands.ShootToSpeakerForwardorBackward;
import frc.robot.commands.PickUpNoteForwardBackward;
import frc.robot.commands.ShooterPitchJoyStickCommand;
import frc.robot.commands.ShooterPitchFixedLocationCommand;
import frc.robot.commands.TurnToAngle;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.Constants.PWMPorts;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ATRollerSubsystem;
import frc.robot.subsystems.ShooterPitchSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Analog0Subsystem;
import frc.robot.subsystems.Analog1ShooterPitchEncoderSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  
    private boolean isAnalogActivated = false;  //keep off we don't want code to control...
     //shut off of motors.
     /******************************************************************/
     //Pick Up Note Constants FORWARD      .leftTrigger()
     /******************************************************************/
     //PickupNoteForwardandBack    variables also use in  //AutoPickupNoteForwardandBackTimed
     //so if you change here --- you change for both....
     //
     double   PickUpNote_IntakePower        = 0.7;     //leftTrigger  Auto  IntakeON
     boolean  PickUpNote_isIntakeInverted   = true;    //leftTrigger  Auto  IntakeON
     double   PickUpNote_ATRollerPower       = 0.4;    //leftTrigger  Auto  IntakeON
     boolean  PickUpNote_isATRollerInverted = false;   //leftTrigger  Auto  IntakeON
     double   PickUpNote_IndexerPercentOut  = 0.0;     //leftTrigger  Auto  IntakeON
     boolean  PickUpNote_isIndexerInverted  = false;   //leftTrigger  Auto  IntakeON

     //AutoPickupNoteForwardandBackTimed   IntakeOn auto command
     double   AutoSecondToRun = 2.0;    // Auto  IntakeON
    
     /******************************************************************/
     //Pick Up Note Constants Backward       .leftBumper()
     /******************************************************************/
     double   PickUpNoteBack_IntakePower        = 1.0;        //leftBumper
     boolean  PickUpNoteBack_isIntakeInverted   = false;      //leftBumper
     double   PickUpNoteBack_ATRollerPower      = 1.0;        //leftBumper
     boolean  PickUpNoteBack_isATRollerInverted = true;       //leftBumper
     double   PickUpNoteBack_IndexerPercentOut  = 1.0;        //leftBumper
     boolean  PickUpNoteBack_isIndexerInverted  = true;       //leftBumper

    //*******************************************************************/
    //                            Shooter Pitch Up
    //.leftStick()   Y does the shooter pitch auto...
    /*******************************************************************/
    
    double ShooterPitchUP_Home = -0.08;    //   .b()    //far away smallest angle  //from podium  
    double ShooterPitchUp_High = -0.145;   //   .y()    //score auto and score close  .35   //center and sides//from woofer

    double ShooterPitchAutoClose = -0.145;       //auto woofer center    auto ShooterPitchClose 
    double ShooterPitchAutoShortClose = -0.145;  //auto woofer short     auto ShooterPitchShortClose
    double ShooterPitchAutoLongClose = -0.145;   //auto woofer long      auto ShooterPitchLongClose 
    double ShooterPitchAutoLongClose2 =  -0.09;  //auto podium long      auto ShooterPitchLongClose2 

    double ShooterPitchClimbPosition = -0.24;     //UpperController right bumper

   //Shooter Pitch auto....seconds to run for pitch 
    
     double AutoSecondToRunClose  = 1.0;     //short and center ....
     double AutoSecondToRunLong   = 1.5;     //X button   auto ShooterPitchShortClose 
      

    /*******************************************************************/
    /*Elevator control Buttons                                         */
    //ElevatorFixedLocationCommand       
    /*******************************************************************/
    double ElevatorControl_High = 205.0;     //POVUP   telop   
    double ElevatorControl_Mid  = 100.0;     //POVLeft  telop
    double ElevatorControl_Home =  1.0;     //POVDOWN  teleop      X button multiple times

   /******************************************************************/
   //Shooting Path 1:   Shoot to Speaker
   //Shooter can be    rightTrigger() 
   //Shoot to speaker   
   /******************************************************************/

   double   ShootToSpeaker_IntakePower          = .7;        //right Trigger  Auto ShootToSpeaker
   boolean  ShootToSpeaker_isIntakeInverted     = true;      //right Trigger  Auto ShootToSpeaker
   double   ShootToSpeaker_IndexerPower         = 1.0;       //right Trigger  Auto ShootToSpeaker
   boolean  ShootToSpeaker_isIndexerInverted    = false;     //right Trigger  Auto ShootToSpeaker
   double   ShootToSpeaker_ATRollerPower        = 1.0;       //right Trigger  Auto ShootToSpeaker
   boolean  ShootToSpeaker_isATRollerInverted  = false;      //right Trigger  Auto ShootToSpeaker
   double   ShootToSpeaker_ShooterRPSA          = 100.0;     //right Trigger  Auto ShootToSpeaker
   boolean  ShootToSpeaker_isShooterInvertedA   = true;      //right Trigger  Auto ShootToSpeaker
   double   ShootToSpeaker_ShooterRPSB          = 100.0;     //right Trigger  Auto ShootToSpeaker 
   boolean  ShootToSpeaker_isShooterInvertedB   = false;     //right Trigger  Auto ShootToSpeaker

  /**************auto shoot to speaker seconds to run */
  double ShootToSpeakerAutoSecondsToRun = 2;   //Auto ShootToSpeaker
 
 
  /******************************************************************/
  //Shooting Path:   AT Staging    Button 3  X button
  //ATStagingForwardorBackwardTimedCommand
  /******************************************************************/

  //forward
   double  ATStagingREV_ElevatorHome       = 0.0;
   boolean ATStagingREV_isATRollerInverted1 = false;       //x button 
   double  ATStagingREV_ATRollerPercentOut1  = 1.0;        //x button 
   double  ATStagingREV_IndexerPercentOut1  = 1.0;         //x button 
   boolean ATStagingREV_isIndexerInverted1  = false;       //x button 
   double ATStagingREV_IndexerSeconds1 = 0.5;              //x button 
  //back 

   double ElevatorControl_ATStaging = 50.0;                //x button 
   boolean  ATStagingREV_isATRollerInverted2 = false;      //x button 
   double ATStagingREV_ATRollerPercentOut2  =  1.0;        //x button 
   double  ATStagingREV_IndexerPercentOut2  = -1.0;        //x button 
   boolean ATStagingREV_isIndexerInverted2  = false;       //x button 
   double ATStagingREV_IndexerSeconds2 =  1.0;              //x button 

  private final SendableChooser<Command> autoChooser;
 
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  "swerve"));

  //Used for intake 
  private final Analog0Subsystem Analog0Subsystem  = new Analog0Subsystem ();
  //used for shooterPitch 
  private final Analog1ShooterPitchEncoderSubsystem  Analog1ShooterPitchEncodeSubsystem = new Analog1ShooterPitchEncoderSubsystem();
  
  
  private final   ShooterSubsystem      ShooterSubsystem        = new ShooterSubsystem();
  private final   IndexerSubsystem      IndexerSubsystem        = new IndexerSubsystem();
  private final   IntakeSubsystem       IntakeSubsystem         = new IntakeSubsystem();
  private final   ClimbSubsystem        ClimbSubsystem          = new ClimbSubsystem();
  private final   BlinkinSubsystem      BlinkinSubsystemPort0   = new BlinkinSubsystem(PWMPorts.kBlinkin);
  public  final   ElevatorSubsystem     ElevatorSubsystem       = new ElevatorSubsystem();
  public final   ShooterPitchSubsystem ShooterPitchSubsystem  = new ShooterPitchSubsystem();
  private final   ATRollerSubsystem     ATRollerSubsystem =       new ATRollerSubsystem();
                                                                        
  private final CommandXboxController DriverController =
        new CommandXboxController(0);
  private final CommandXboxController UpperController =
        new CommandXboxController(1);


  private static final Command ClimbJoyStickCommand = null;
  private static final Command ShooterPitchJoyStickCommand = null;
    
  public RobotContainer()
  {

   CommandScheduler.getInstance().setDefaultCommand(ClimbSubsystem, ClimbJoyStickCommand);
   CommandScheduler.getInstance().setDefaultCommand(ShooterPitchSubsystem, ShooterPitchJoyStickCommand);

    // Configure the trigger bindings
    configureBindings();

    
Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
() -> -MathUtil.applyDeadband(DriverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
() -> -MathUtil.applyDeadband(DriverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
() -> -DriverController.getRawAxis(4) * Math.abs(DriverController.getRawAxis(4)));

/************************************ */

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity: driveFieldOrientedAnglularVelocity);

    

   NamedCommands.registerCommand("ShooterPitchClose",new  AutoShooterPitchFixedLocationTimedCommand(ShooterPitchSubsystem,
                                                          ShooterPitchAutoClose, 
                                                          AutoSecondToRunClose));

   NamedCommands.registerCommand("ShooterPitchLongClose",new  AutoShooterPitchFixedLocationTimedCommand(ShooterPitchSubsystem,
                                                          ShooterPitchAutoLongClose, 
                                                          AutoSecondToRunLong));
   NamedCommands.registerCommand("ShooterPitchLongClose2",new  AutoShooterPitchFixedLocationTimedCommand(ShooterPitchSubsystem,
                                                          ShooterPitchAutoLongClose2, 
                                                          AutoSecondToRunLong));
   NamedCommands.registerCommand("ShooterPitchShortClose",new  AutoShooterPitchFixedLocationTimedCommand(ShooterPitchSubsystem,
                                                          ShooterPitchAutoShortClose, 
                                                          AutoSecondToRunLong));
   NamedCommands.registerCommand("IntakeON",new  AutoPickUpNoteForwardBackwardTimed(BlinkinSubsystemPort0,
                                                  Analog0Subsystem,  
                                                  IntakeSubsystem,
                                                      PickUpNote_IntakePower,
                                                      PickUpNote_isIntakeInverted,
                                                    ATRollerSubsystem,
                                                      PickUpNote_ATRollerPower,
                                                      PickUpNote_isATRollerInverted,
                                                    IndexerSubsystem,
                                                      PickUpNote_IndexerPercentOut,
                                                      PickUpNote_isIndexerInverted,
                                                      isAnalogActivated,
                                                      AutoSecondToRun));

    NamedCommands.registerCommand("ResetGyro", new InstantCommand(drivebase::zeroGyro));
 
    NamedCommands.registerCommand("ShootToSpeaker", new AutoShootToSpeakerTimed(BlinkinSubsystemPort0,
                                                                                     Analog0Subsystem,  
                                                                                     IntakeSubsystem,                         
                                                                                        ShootToSpeaker_IntakePower,
                                                                                        ShootToSpeaker_isIntakeInverted,
                                                                                      ATRollerSubsystem,
                                                                                         ShootToSpeaker_ATRollerPower,
                                                                                         ShootToSpeaker_isATRollerInverted,      
                                                                                      IndexerSubsystem,
                                                                                         ShootToSpeaker_IndexerPower ,
                                                                                         ShootToSpeaker_isIndexerInverted ,
                                                                                      ShooterSubsystem,
                                                                                          ShootToSpeaker_ShooterRPSA,
                                                                                          ShootToSpeaker_isShooterInvertedA,
                                                                                          ShootToSpeaker_ShooterRPSB,                                                              
                                                                                          ShootToSpeaker_isShooterInvertedB,
                                                                                      false, 
                                                                                          ShootToSpeakerAutoSecondsToRun));


                                                                                                          
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }
  
  private void configureBindings()
  {
    
     UpperController
    .leftTrigger()
    .onTrue(new  PickUpNoteForwardBackward(BlinkinSubsystemPort0,
                                                                Analog0Subsystem,  
                                                                IntakeSubsystem,
                                                                    PickUpNote_IntakePower,
                                                                    PickUpNote_isIntakeInverted,
                                                                  ATRollerSubsystem,
                                                                    PickUpNote_ATRollerPower,
                                                                    PickUpNote_isATRollerInverted,
                                                                  IndexerSubsystem,
                                                                    PickUpNote_IndexerPercentOut,
                                                                    PickUpNote_isIndexerInverted ,
                                                                    isAnalogActivated));                                                     


   UpperController
    .leftBumper()
    .whileTrue(new  PickUpNoteForwardBackward(BlinkinSubsystemPort0,
                                                        Analog0Subsystem,  
                                                       IntakeSubsystem,
                                                          PickUpNoteBack_IntakePower,
                                                          PickUpNoteBack_isIntakeInverted,
                                                        ATRollerSubsystem,
                                                          PickUpNoteBack_ATRollerPower,
                                                          PickUpNoteBack_isATRollerInverted,
                                                        IndexerSubsystem,
                                                          PickUpNoteBack_IndexerPercentOut,
                                                           PickUpNoteBack_isIndexerInverted ,
                                                           false)); 


   /******************************************************************/
   //Shooting Path 1:   Shoot to Speaker
   //Shooter can be 
   /******************************************************************/

   UpperController
    .rightTrigger()
    .onTrue(new ShootToSpeakerForwardorBackward(BlinkinSubsystemPort0,
                                                     Analog0Subsystem,  
                                                        IntakeSubsystem,                         
                                                              ShootToSpeaker_IntakePower,
                                                              ShootToSpeaker_isIntakeInverted,
                                                         ATRollerSubsystem,
                                                              ShootToSpeaker_ATRollerPower,
                                                              ShootToSpeaker_isATRollerInverted,
                                                        IndexerSubsystem,
                                                              ShootToSpeaker_IndexerPower ,
                                                              ShootToSpeaker_isIndexerInverted ,
                                                        ShooterSubsystem,
                                                              ShootToSpeaker_ShooterRPSA,
                                                              ShootToSpeaker_isShooterInvertedA,
                                                              ShootToSpeaker_ShooterRPSB,                                                              
                                                              ShootToSpeaker_isShooterInvertedB,
                                                        false));

    //*******************************************************************/
    //                            Shooter Pitch Up
    /*******************************************************************/
    
     UpperController
    .b()
    .onTrue(new ShooterPitchFixedLocationCommand(ShooterPitchSubsystem,
                                              ShooterPitchUP_Home));  


    /*******************************************************************/
    //                           HIGH
    /*******************************************************************/
    UpperController
    .y()
    .onTrue(new ShooterPitchFixedLocationCommand(ShooterPitchSubsystem,
                                             ShooterPitchUp_High ));  
                                                                                      
    /*******************************************************************/
    //                           hangerShooterPosition
    /*******************************************************************/
    UpperController
    .rightBumper()
    .onTrue(new ShooterPitchFixedLocationCommand(ShooterPitchSubsystem,
                                             ShooterPitchClimbPosition));  
                                                                          
                                             
    //*******************************************************************/
    //                            Shooter Pitch JoyStick
    /*******************************************************************/                                       
    UpperController 
    .leftStick()
    .onTrue( new ShooterPitchJoyStickCommand(Analog1ShooterPitchEncodeSubsystem, ShooterPitchSubsystem,
                                         () ->  UpperController .getLeftY() * Math.abs(UpperController .getLeftY())));


  //  /******************************************************************/
  //  //Shooting Path:   AT Staging
  //  // X button which is button3
  //  /******************************************************************/

     UpperController
    .button(3)   //x button
    .onTrue(new SequentialCommandGroup
     (         new AutoShooterPitchFixedLocationTimedCommand(ShooterPitchSubsystem,
                                              ShooterPitchUP_Home,
                                              AutoSecondToRunLong),     
               new  ElevatorFixedLocationCommand(ElevatorSubsystem, ElevatorControl_Home),                                                                       
               new ATStagingForwardorBackwardTimedCommand(BlinkinSubsystemPort0,
                                                       Analog0Subsystem,  
                                                       ATRollerSubsystem,
                                                          ATStagingREV_ATRollerPercentOut1,
                                                          ATStagingREV_isATRollerInverted1,
                                                       IndexerSubsystem,
                                                          ATStagingREV_IndexerPercentOut1,
                                                          ATStagingREV_isIndexerInverted1,
                                                      false, 
                                                      ATStagingREV_IndexerSeconds1 ),
              new ParallelCommandGroup(
                   new ATStagingForwardorBackwardTimedCommand(BlinkinSubsystemPort0,
                                                       Analog0Subsystem,  
                                                       ATRollerSubsystem,
                                                          ATStagingREV_ATRollerPercentOut2,
                                                          ATStagingREV_isATRollerInverted2,
                                                       IndexerSubsystem,
                                                          ATStagingREV_IndexerPercentOut2,
                                                          ATStagingREV_isIndexerInverted2,
                                                      false, 
                                                      ATStagingREV_IndexerSeconds2 ),
                  new SequentialCommandGroup(
         	              new WaitCommand(0.5),
                        new  ElevatorFixedLocationCommand(ElevatorSubsystem, ElevatorControl_ATStaging) )   
                  ),   
                  new  ElevatorFixedLocationCommand(ElevatorSubsystem, ElevatorControl_Home)
                                 
         
               )
             
    );                                                       

   /******************************************************************/
   //Shooting Path:   AT rolller ONLY
   /******************************************************************/
                                        
    UpperController
    .povRight()
    .onTrue(new  ATRollerRunTimedCommand(ATRollerSubsystem, 
                                 BlinkinSubsystemPort0, 
                                 10.0));
    
    /*******************************************************************/
    //                               Elevator High      
    /*******************************************************************/
   
    UpperController
    .povUp()
    .onTrue(new  ElevatorFixedLocationCommand(ElevatorSubsystem, ElevatorControl_High));
    
    /*******************************************************************/
    //                               Elevator Mid      
    /*******************************************************************/    

    UpperController
    .povLeft()
    .onTrue(new  ElevatorFixedLocationCommand(ElevatorSubsystem, ElevatorControl_Mid));

    /*******************************************************************/
    //                               Elevator Home    
    /*******************************************************************/    

    UpperController
    .povDown()
    .onTrue(new  ElevatorFixedLocationCommand(ElevatorSubsystem, ElevatorControl_Home));
       
     /******************************************************************/
     //End of match commands 
    /******************************************************************/
    //                                Climber Joystick
     /*****************************************************************/
     UpperController.rightStick()
     .onTrue(new ClimbJoyStickCommand(ClimbSubsystem,
                                      () -> MathUtil.applyDeadband(UpperController.getRightY(), 0.01) ));
   
    UpperController
    .a()
    .onTrue(new TurnOffAllMotorCommand(IntakeSubsystem,
                                        ATRollerSubsystem,
                                        IndexerSubsystem,
                                        ShooterSubsystem,
                                        BlinkinSubsystemPort0));  

   /*******************************************************************/
    //                          resets gyro     not sure on button 
    /*******************************************************************/
    DriverController
    .button(7)  //small copy button in center of controller
    .onTrue(new InstantCommand(drivebase::zeroGyro));                                                        
    
    
  //  /*******************************************************************/
  //  //                      turns off analog0 sensor conditions.not sure on button 
  //  /*******************************************************************/
  //   UpperController
  //   .button(7)
  //   .onTrue(new InstantCommand(() -> isAnalogActivated = false));      
       
  }

  public Command getAutonomousCommand() 
  {      
      return autoChooser.getSelected();    
           
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void joyStick()
  {
    
  }
}
