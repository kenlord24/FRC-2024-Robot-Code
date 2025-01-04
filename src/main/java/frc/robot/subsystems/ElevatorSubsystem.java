package frc.robot.subsystems;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 public class ElevatorSubsystem extends SubsystemBase 
{

  private final TalonFX m_fx = new TalonFX(28);
  private final DigitalInput DIO = new DigitalInput(0);
  
  public double RPMMultipler;
  public double kP = 0.028;
  public double kI = 0.00;
  public double kD = 0.00003;
  public double kV =  0.0;
  public String subSystem = "Elevator";
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
  public  ElevatorSubsystem()
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
    m_fx.getPosition().setUpdateFrequency(20);
  
  }

  public void setElevatorRPS(double desiredRoationsPerSecond)
  {
   
    m_fx.setControl(m_voltageVelocity.withVelocity(desiredRoationsPerSecond));
    
  }
  public TalonFX getElevatorMotor()
  {
    return m_fx;
  } 

  public void setElevatorDisable()
  {
    m_fx.disable();
  } 

  public void setElevatorInverted(boolean inverted)
  {
    m_fx.setInverted(inverted);

  } 
   
  public void setElevatorSpeed(double percentOutput)
  {
    m_fx.set(percentOutput);
    SmartDashboard.putNumber("Elevator Power SS", percentOutput);

  }
public double getPosition()
  {
   	

    var rotorPosSignal = m_fx.getRotorPosition();
    return(rotorPosSignal.getValue());

  }

  public void setElevatorCoast()
  {
    
    m_fx.setNeutralMode(NeutralModeValue.Coast);
    //m_fx.setControl(m_brake); 

  }

  public void setElevatorBrake()
  {
         m_fx.setNeutralMode(NeutralModeValue.Brake ); 
  
  
  }
  
  public double getElevatorVelocity()
  {
  
    return m_fx.getVelocity().getValue();
  }

 

  @Override
  public void periodic() 
  {  


    if(!DIO.get())
    {
      m_fx.setPosition(0);
      lastPosition = 0; 
    
    }

    SmartDashboard.putNumber("Elevator periodic" , this.getPosition());
    SmartDashboard.putNumber("Elevator Velocity",  m_fx.getVelocity().getValue());
    SmartDashboard.putNumber("Elevator Current", m_fx.getSupplyCurrent().getValue());
    SmartDashboard.putBoolean("Elevator DIO",  DIO.get());
  }


}
