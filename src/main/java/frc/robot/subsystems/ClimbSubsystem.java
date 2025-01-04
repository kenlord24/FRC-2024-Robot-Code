package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase 
{
  
     TalonSRX _ClimbRightMotor = new TalonSRX(26);
    

    /** electic brake during neutral */
	final NeutralMode kBrakeDurNeutral = NeutralMode.Brake;
 

  /*********************************************************/
  /*   ClimbSubSystem constructor                         */
  /*********************************************************/
  public ClimbSubsystem() 
  {
  
        TalonSRXConfiguration configs = new TalonSRXConfiguration();

       _ClimbRightMotor.configAllSettings(configs);
    	   
     	 _ClimbRightMotor.setInverted(true);		
		   _ClimbRightMotor.setNeutralMode(kBrakeDurNeutral);    
       _ClimbRightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 200); 
           
  }
 
  public void setClimbSpeed(double speed)
  {
    _ClimbRightMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() 
  {

    SmartDashboard.putNumber("Climber Current", _ClimbRightMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Climber Current", _ClimbRightMotor.getSupplyCurrent());

  }

}




