// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //sparks
  public static final int DRIVETRAIN_LEFT_FRONT_SPARK = 0;
  public static final int DRIVETRAIN_LEFT_BACK_SPARK = 1;
  public static final int DRIVETRAIN_RIGHT_FRONT_SPARK = 2;
  public static final int DRIVETRAIN_RIGHT_BACK_SPARK = 3;
  public static final int CLAW_SPARK = 4;



  //Spark Encoder / Can#
   public static final int ARM_SPARK = 1;

  //xbox controllers
  public static final int DRIVER_CONTROLLER = 0;
  public static final int OPERATOR_CONTROLLER = 1;
  
  //limitSwitches
  public static final int BOTTOM_LIMIT = 1;
  public static final int TOP_LIMIT = 0;
  public static final int CLAW_LIMIT = 2;

  //gyroscope
  //public static String GYRO = "SPI.Port.kOnboardCS0";
  
  //limiters
  public static final double DRIVETRAIN_CONTROL_SPEED_LIMIT = 0.75;

  //speed constants
  public static final double ARM_EXTEND_SPEED  = -0.2;
  public static final double ARM_RETRACT_SPEED =  0.05;
  public static final double CLAW_OPEN_SPEED = 1;
  public static final double CLAW_CLOSE_SPEED = -1;

  //time constants
  public static final double ARM_MIDDLE_REVERSE_TIME = 2.9  ; // measured in seconds
  public static final double CLAW_OPEN_CLOSE_TIME  = 0.12 ; // measured in seconds 
  public static final double CLAW_CONE_CLOSE_TIME     = 0.25 ; // measured in seconds 
  public static final double CLAW_CUBE_CLOSE_TIME     = 0.375; // measured in seconds 
  public static final double AUTO_DROP_OFF_DRIVE_TIME = 1.0  ; // measured in seconds
  public static final double AUTO_SAFE_ZONE_REMOVAL   = 5.0  ; // measured in seconds

  //level constants
  public static final double ROBOT_OFF_BALANCE_THRESHOLD = 10; // measured in degrees
  public static final double ROBOT_ON_BALANCE_THRESHOLD  =  5; // measured in degrees
}