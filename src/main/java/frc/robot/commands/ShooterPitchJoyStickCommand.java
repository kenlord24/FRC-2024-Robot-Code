package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Analog1ShooterPitchEncoderSubsystem;
import frc.robot.subsystems.ShooterPitchSubsystem;


public class ShooterPitchJoyStickCommand extends Command
{
  private final Analog1ShooterPitchEncoderSubsystem  Analog1ShooterPitchEncodeSubsystem;
  private final ShooterPitchSubsystem ShooterPitchSubsystem;
  private final Supplier<Double> speedFunction;
                   

  /*********************************************************/
  /*   ShooterPitchoyCmd  constructor   During teleop         */
  /*********************************************************/
  public ShooterPitchJoyStickCommand( Analog1ShooterPitchEncoderSubsystem  Analog1ShooterPitchEncodeSubsystem,
                                      ShooterPitchSubsystem  ShooterPitchSubsystem,
                                      Supplier<Double> speedFunction)
                              
  {
    this. ShooterPitchSubsystem=  ShooterPitchSubsystem;
    this.Analog1ShooterPitchEncodeSubsystem = Analog1ShooterPitchEncodeSubsystem;
    this.speedFunction = speedFunction;    //Y axis on xbox controller 
    addRequirements(ShooterPitchSubsystem);
    addRequirements(Analog1ShooterPitchEncodeSubsystem);
  }
  /*********************************************************/
  /*   initlize runs after constructor automatically       */
  /*********************************************************/
  @Override
  public void initialize() 
  {
      SmartDashboard.putNumber("ShooterJoy AVG Voltage",  Analog1ShooterPitchEncodeSubsystem.getShooterPitchJoyAverageVoltage());
 
  }
  /*********************************************************/
  /*   Execute runs isFinished method returns a true       */
  /*********************************************************/

  @Override
  public void execute() 
  {
    double realTimeSpeed = speedFunction.get();   //Y  is now positive up
                                                  //negative for down....
    SmartDashboard.putNumber("Y controller2",  realTimeSpeed);

    //Joy Stick ramping
    double realTimeSpeedSqr = realTimeSpeed * realTimeSpeed;

    if((realTimeSpeed <.05) && (realTimeSpeed > -.05))
    {
        realTimeSpeedSqr =  0;
    }
    else
    { 
        if(realTimeSpeed < 0)
        {
          realTimeSpeedSqr = realTimeSpeedSqr * -1;
        }  
    }

    double motor = realTimeSpeedSqr;

    double currentDistancePosition = ShooterPitchSubsystem.getPosition();
    double max = -0.25;
    double min = -0.02;

       
    // - 0.251       < -0.25               //.25
    if(currentDistancePosition < max && motor < 0)
    {
         ShooterPitchSubsystem.setShooterPitchSpeed(0.001);
    }
    //if reached min and joystick still trying to go down stop the whole thing...
    // //            -0.04 > -.05            && -.01 joystick
    else if (currentDistancePosition > min && motor > 0)
    {
          ShooterPitchSubsystem.setShooterPitchSpeed(0.001);
    }
    else  ShooterPitchSubsystem.setShooterPitchSpeed(motor*0.1);

  }

  @Override
  public void end(boolean interrupted) 
  {
  
  }

  /*********************************************************/
  /*   isFinished always returns false so CMD during telop */
  /*        never ends                                     */
  /*********************************************************/
  @Override
  public boolean isFinished() 
  {
    
      return false;
  }
}
