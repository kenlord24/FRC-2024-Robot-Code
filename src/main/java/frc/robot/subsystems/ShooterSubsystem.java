package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 public class ShooterSubsystem extends SubsystemBase 
{

  private final TalonFX m_fx = new TalonFX(18);
  private final TalonFX m_fxB = new TalonFX(19);

  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  
  double MotorA_SD_Velocity = 0;
  double MotorB_SD_Velocity = 0;
  


  //****************************************************************/
  //*             Drive Train Subsystem  Constructor               */
  //*Sets up 2 talons with integrated sensors                      */
  //*Setup 2                                                       */
  //****************************************************************/
  public ShooterSubsystem()
  {


    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;
    
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(configs);
      status = m_fxB.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }


    m_fx.getPosition().setUpdateFrequency(5);
    m_fxB.getPosition().setUpdateFrequency(5);
  
    
 
  }


  public void setShooterController(double desiredRoationsPerSecond)
  {
   
    m_fx.setControl(m_voltageVelocity.withVelocity(desiredRoationsPerSecond));
    m_fxB.setControl(m_voltageVelocity.withVelocity(desiredRoationsPerSecond));
  }
  public Object getShooterMotor()
  {
    return m_fx;
  } 

  public Object getShooterMotorB()
  {
    return m_fxB;
  } 
 
 
  public void setShooterInverted(boolean invertedA,boolean invertedB )
  {
    m_fx.setInverted(invertedA);
    m_fxB.setInverted(invertedA);
   
  
  }


  //
  public void setShooterRPS(double RPS, double RPSB)
  {
    m_fx.setControl(m_voltageVelocity.withVelocity(RPS));
    m_fx.setControl(m_voltageVelocity.withVelocity(RPS));
    m_fxB.setControl(m_voltageVelocity.withVelocity(RPSB));
   
    
  }
 
  public void setShooterCoast()
  {
    
    m_fx.setNeutralMode(NeutralModeValue.Coast);
    m_fxB.setNeutralMode(NeutralModeValue.Coast);

     
  }
  

  public double getShooterVelocity()
  {
  
    return m_fx.getVelocity().getValue();
  }

  public double getShooterVelocityB()
  {
  
    return m_fx.getVelocity().getValue();
  }


 
  @Override
  public void periodic() 
  {  

    
    SmartDashboard.putNumber("Shooter Velocity", m_fx.getVelocity().getValue());
    SmartDashboard.putNumber("Shooter Current A", m_fx.getSupplyCurrent().getValue());
    SmartDashboard.putNumber("Shooter Current B", m_fxB.getSupplyCurrent().getValue());
       
  }


}
