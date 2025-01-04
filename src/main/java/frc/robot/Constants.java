// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final Translation2d flModuleOffset = new Translation2d(0.3, 0.3);
  public static final Translation2d frModuleOffset = new Translation2d(0.3, -0.3);
  public static final Translation2d blModuleOffset = new Translation2d(-0.3, 0.3);
  public static final Translation2d brModuleOffset = new Translation2d(-0.3, -0.3);

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
   public static final class AutonConstants
 {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(2.0, 0.0, 0.05);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.9, 0.006, 0.045);
  }
  /****************************turn button */  
///used for turning with buttons only........
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 16;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularSpeedRadiansPerSecondSquared);
            public static final double trackWidth = Units.inchesToMeters(23);
            public static final double wheelBase = Units.inchesToMeters(23);
            public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

}

    public static class BlinkinConstants {
      public static final double BLINKIN_RED = 0.61;
      public static final double BLINKIN_BLUE = 0.87;
      public static final double BLINKIN_YELLOW = 0.69;
      public static final double BLINKIN_VIOLET = 0.91;
      public static final double BLINKIN_RAINBOW = -0.99;
      public static final double BLINKIN_HOT_PINK = 0.57;
      public static final double BLINKIN_GREEN = 0.77;
      public static final double BLINKIN_FADE_TO_BLACK = -0.03;
      public static final double BLINKIN_RAINBOW_WAVE = -0.45;
      public static final double BLINKIN_RAINBOW_SINELON = -0.45;
      public static final double BLINKIN_CONFETTI = -0.87;
      public static final double BLINKIN_FIRE = -0.59;
      public static final double BLINKIN_GLITTER = -0.89;
      public static final double BLINKIN_PARTY_WAVE = -0.43;
      public static final double BLINKIN_SHOT_RED = -0.85;
     
    }
    public static final class PWMPorts 
    {
      public static final int kBlinkin = 0;
      public static final int kAddressableLED = 9;
  }
  
}
