package frc.robot.subsystems;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 public class ShooterPitchSubsystem extends SubsystemBase 
{

  private final TalonFX m_fx = new TalonFX(17);
  private final CANcoder m_cancoder = new CANcoder(37);

  
  public double RPMMultipler;
  public double kP = 28;
  public double kI = 0;
  public double kD = 0.000;
  public double kV =  0.3;
  public String subSystem = "ShooterPitch";
  public double lastPosition = 0;
  //torque

  public TalonFXConfiguration configs = new TalonFXConfiguration();
  public double kFF, kMaxOutput, kMinOutput, maxRPM;

  
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final NeutralOut m_brake = new NeutralOut();

    //****************************************************************/
  //*             Drive Train Subsystem  Constructor               */
  //*Sets up 2 talons with integrated sensors                      */
  //*Setup 2                                                       */
  //****************************************************************/
  public  ShooterPitchSubsystem()
  {
  

    configs = new TalonFXConfiguration();
    
    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = kP; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = kI; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = kD; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = kV; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
  
    configs.Feedback.FeedbackRemoteSensorID = m_cancoder.getDeviceID();
    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {

      status = m_fx.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    
     m_fx.setControl(m_brake);
     m_fx.setPosition(0);
     var rotorPosSignal = m_fx.getRotorPosition();
  
     rotorPosSignal.refresh();
     m_fx.setControl(m_brake);
     m_fx.setNeutralMode(NeutralModeValue.Brake );
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

          
  }


  public void setShooterPitchRPS(double desiredRoationsPerSecond)
  {
   
    m_fx.setControl(m_voltageVelocity.withVelocity(desiredRoationsPerSecond));
    
  }
  public TalonFX getShooterPitchMotor()
  {
    return m_fx;
  } 

  public void setShooterPitchInverted(boolean inverted)
  {
    m_fx.setInverted(inverted);

  } 
   
  public void setShooterPitchSpeed(double percentOutput)
  {
    m_fx.set(percentOutput);
  
  }


  public double getPosition()
  {
  
  
    return (m_cancoder.getAbsolutePosition().getValueAsDouble());

  }

 
  public void setShooterPitchCoast()
  {
    
    m_fx.setNeutralMode(NeutralModeValue.Coast);
    //m_fx.setControl(m_brake); 
 
     
  }

  public void setShooterPitchBrake()
  {
         m_fx.setNeutralMode(NeutralModeValue.Brake ); 
  
  
  }
 
  public double getShooterPitchVelocity()
  {
  
    return m_fx.getVelocity().getValue();
  }

  @Override
  public void periodic() 
  {  

    SmartDashboard.putNumber("ShooterPitch m_Cancoder Abs", m_cancoder.getAbsolutePosition().getValueAsDouble()); 

  }

}