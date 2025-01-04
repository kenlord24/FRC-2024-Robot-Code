// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;

public class BlinkinSubsystem extends SubsystemBase {
  private static boolean purpled;
  private static Spark blinkinLED = null;
  /** Creates a new BlinkinSubsystem. */
  public BlinkinSubsystem(int port) 
  {
    blinkinLED  = new Spark(port);
    blinkinYellowSet();
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  
  public  double getVoltage(){
      return getVoltage();
  }
  public static void setVoltage(double volts){
    blinkinLED.setVoltage(volts);
  }

  public  void blinkinSolidOrangeSet(){
    blinkinLED.set(.65);
  }

  public  void blinkinSolidYellowSet(){
    blinkinLED.set(.65);
  }
  
  public  void blinkinRainbowSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_RAINBOW);
  }
  
  public  void blinkinRedSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_RED);
  }
  
  public  void blinkinBlueSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_BLUE);
  }

  public  void blinkinYellowSet(){
     blinkinLED.set(BlinkinConstants.BLINKIN_YELLOW);
  }

  public void blinkinVioletSet(){
    purpled = true;
    blinkinLED.set(BlinkinConstants.BLINKIN_VIOLET);
  }
  
  public void blinkinGreenSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_GREEN);
  }

  public void blinkinFlashingGreenSet(){
    
    blinkinLED.set(BlinkinConstants.BLINKIN_GREEN);
  }
  public void blinkinHotPinkSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_HOT_PINK);
  }
  
  public void blinkinRainbowWaveSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_RAINBOW_WAVE);
  }
  
  public void blinkinRainbowSinelonSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_RAINBOW_SINELON);
  }
  
  public  void blinkinConfettiSet() {
    blinkinLED.set(BlinkinConstants.BLINKIN_CONFETTI);
  }
  
  // Fades from color set on device into black
  public  void blinkinFadeToBlack(){
    blinkinLED.set(BlinkinConstants.BLINKIN_FADE_TO_BLACK);
  }

  public  void blinkinFireSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_FIRE);
  }

  public  void blinkinGlitterSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_GLITTER);
  }

  public void blinkinPartyWaveSet()
  {
    blinkinLED.set(BlinkinConstants.BLINKIN_PARTY_WAVE);
  }

  public void blinkinShotRedSet()
  {
    blinkinLED.set(BlinkinConstants.BLINKIN_SHOT_RED);
  }

  public  boolean isPurpled(){
    return purpled;
  }

  

  /** Gets the last PWM value set to the Blinkin */
  public  double getBlinkinColor(){
    return blinkinLED.get();
  }

  

}