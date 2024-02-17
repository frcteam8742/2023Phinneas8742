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
  public static final int CLAW2_SPARK = 5;



  //Spark Encoder / Can#
   public static final int ARM_SPARK = 1;

  //xbox controllers
  public static final int DRIVER_CONTROLLER = 0;
  public static final int OPERATOR_CONTROLLER = 1;
  
  //limitSwitches
  // public static final int BOTTOM_LIMIT = 1;
  // public static final int TOP_LIMIT = 0;
  // public static final int CLAW_LIMIT = 2;
  public static final int BIMBA_MRS = 0;
  public static final int PHOTOELECTRIC_SWITCH = 1;
  
  //Range Finders
  public static final int ULTRASONIC_LR = 0;
  public static final int ULTRASONIC_SR = 1;

  //gyroscope
  //public static String GYRO = "SPI.Port.kOnboardCS0";
  
  //limiters
  public static final double DRIVETRAIN_CONTROL_SPEED_LIMIT = 0.85;

  //speed constants
  // public static final double ARM_EXTEND_SPEED  = -0.25;
  // public static final double ARM_RETRACT_SPEED =  0.1;
  public static final double CLAW_IN_SPEED = .60;
  public static final double CLAW_OUT_SPEED = -.35;
  public static final double CLAW_HOLD_SPEED = 0.22; //0.22
  // public static final double ARM_HOLD_MID_SPEED = -0.045;
  // public static final double ARM_HOLD_DOWN_SPEED = -0.02;
  // public static final double ARM_HOLD_UP_SPEED = -0.052;

  
  //time constants
  public static final double ARM_MIDDLE_REVERSE_TIME = 0.65  ; // measured in seconds
  public static final double ARM_TOP_REVERSE_TIME = 0.01; // measured in seconds
  public static final double ARM_MOSTLYDOWN_REVERSE_TIME = 3.1; // measured in seconds
  public static final double AUTO_DROP_OFF_DRIVE_TIME = 0.58   ; // measured in seconds
  public static final double AUTO_SAFE_ZONE_REMOVAL   = 0.58  ; // measured in seconds
  public static final double AUTO_PLATFORM   = 0.5  ; // measured in seconds
  public static final double AUTO_JUST_LEAVE = 1.0 ; // time to just leave the zone in seconds
  public static final double AUTO_RELEASE_CUBE_TIME = 1.0; //time to release cube in auto
  public static final double AUTO_BACK_OUT_TIME = 3.0; //time to release cube in auto
  public static final double CUBE_TO_BALANCE = 1.5 ; //time to move to balance after dropping cube in auto
  public static final double TEST_AUTO_SPEED_TIME = 1;


  //level constants
  public static final double ROBOT_OFF_BALANCE_THRESHOLD = 10; // measured in degrees
  public static final double ROBOT_ON_BALANCE_THRESHOLD  =  5; // measured in degrees

  //LED Constants
  public static final String LED_RESET  = "c"; 
  public static final String LED_YELLOW = "y"; 
  public static final String LED_PURPLE = "p"; 
}