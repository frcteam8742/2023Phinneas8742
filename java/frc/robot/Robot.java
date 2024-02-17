// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DigitalInput;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.cameraserver.CameraServer;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;

import java.io.*;
// import edu.wpi.first.wpilibj.SerialPort;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.util.Color;
// import com.revrobotics.ColorSensorV3;
// import com.revrobotics.ColorMatchResult;
// import com.revrobotics.ColorMatch;
// import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.AddressableLED;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.DriverStation;

public class Robot extends TimedRobot {

    private static final int deviceID = 1;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    public double rotations;
    private double min;
    private double max;

    // // Code for REV Color Sensor v3 plugged into I2C port.
    // public final I2C.Port i2cPort = I2C.Port.kOnboard;
    // private final Color WHITE = new Color(0.268, 0.464, 0.268);
    // private final Color YELLOW = new Color(0.350, 0.518, 0.132);
    // private final Color RED = new Color(.483, .358, .160);
    // private final Color GREEN = new Color(.209, .539, .252);
    // private final Color BLUE = new Color(0.149, 0.343, 0.508);
    // private final Color PURPLE = new Color(0.2, 0.32, 0.48);

    // // Code for Ultrasonic RangeFinder plugged into Analog 0
    // private final AnalogInput ultrasonicLongRange = new
    // AnalogInput(Constants.ULTRASONIC_LR);
    // private final double Volts_to_Inches_Long = 40;

    // public double getVoltageLong() {
    // return ultrasonicLongRange.getVoltage();
    // }

    // public double getDistanceLong() {
    // return getVoltageLong() * Volts_to_Inches_Long;
    // }

    // // Code for Ultrasonic RangeFinder short range
    // private final AnalogInput ultrasonicShortRange = new
    // AnalogInput(Constants.ULTRASONIC_SR);
    // private final double Volts_to_Inches_Short = 1.85;

    // public double getVoltageShort() {
    // return ultrasonicShortRange.getVoltage();
    // }

    // public double getDistanceShort() {
    // return ((getVoltageShort() * Volts_to_Inches_Short) + 2.5);
    // }

    // LED Stuff?
    // AddressableLED m_led = new AddressableLED(9);

    // create a sendable chooser to choose which auto program we want
    // public static SendableChooser sendablechooser

    private static final String kNewAuto = "Cube and stay";
    private static final String kDefaultAuto = "ONLY DRIVES OUT";
    private static final String kEverythingAuto = "Cube and drive back";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    /**
     * DRIVETRAIN
     */

    // physical motor interfaces
    PWMSparkMax leftFrontSpark = new PWMSparkMax(Constants.DRIVETRAIN_LEFT_FRONT_SPARK);
    PWMSparkMax leftBackSpark = new PWMSparkMax(Constants.DRIVETRAIN_LEFT_BACK_SPARK);
    PWMSparkMax rightFrontSpark = new PWMSparkMax(Constants.DRIVETRAIN_RIGHT_FRONT_SPARK);
    PWMSparkMax rightBackSpark = new PWMSparkMax(Constants.DRIVETRAIN_RIGHT_BACK_SPARK);

    // puts the physical motors into a single group for synchronous control
    MotorControllerGroup leftMotors = new MotorControllerGroup(leftFrontSpark, leftBackSpark);
    MotorControllerGroup rightMotors = new MotorControllerGroup(rightFrontSpark, rightBackSpark);
    private DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);

    /**
     * ARM CONTROL
     */
    CANSparkMax armMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
    CANSparkMax arm_clawMotor = new CANSparkMax(2, MotorType.kBrushed);
    CANSparkMax other_clawMotor = new CANSparkMax(3, MotorType.kBrushed);
    // DigitalInput arm_topLimitSwitch = new DigitalInput(Constants.TOP_LIMIT);
    // DigitalInput arm_bottomLimitSwitch = new
    // DigitalInput(Constants.BOTTOM_LIMIT);
    // DigitalInput arm_clawLimitSwitch = new DigitalInput(Constants.CLAW_LIMIT);

    /**
     * HIDs
     */
    private XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER);
    private XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER);

    /**
     * MOTION MEASUREMENT DEVICES
     */
    AHRS ahrs; // NavX
    ADIS16470_IMU imu = new ADIS16470_IMU(); // RoboRIO IMU

    // SerialPort LEDSerialPort = new SerialPort(9600, SerialPort.Port.kUSB2);

    FileWriter LEDController;


    // ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    // ColorMatch m_colorMatcher = new ColorMatch();
    // DigitalInput m_bimbaMRS = new DigitalInput(Constants.BIMBA_MRS);
    // DigitalInput m_photoElectric = new
    // DigitalInput(Constants.PHOTOELECTRIC_SWITCH);

    /**
     * Claw SPEEDS
     */
    // static final double clawOpenSpeed = 1;
    // static final double clawCloseSpeed = -1;
    // double currentTime = 0;

    boolean autoBalanceYMode;

    /**
     * CLAW STATES
     */
    enum CLAW_MODE {
        stop,
        in,
        out,

    };

    /**
     * ARM STATES
     */
    enum ARM_MODE {
        stop,
        up,
        down,
        middle,
        low,
        moveHigherMid,
        moveLowerMid,
        moveLowerHigh,
        moveHigherLow
    };

    ARM_MODE currentArmAction = ARM_MODE.stop;
    double armMotionStartTime = 0; // used to time the arm motion in "middle"

    CLAW_MODE currentClawAction = CLAW_MODE.stop;
    double clawMotionStartTime = 0;

    /**
     * AUTO STAGES
     */
    enum AUTO_STAGE {
        start,
        grab_cone,
        lift_arm,
        move_to_drop,
        release_cube,
        move_out_of_safe_zone,
        drop_arm,
        drive_to_platform,
        move_to_balance,
        end,
        stop,
    };

    AUTO_STAGE currentAutoStage = AUTO_STAGE.start;
    double autoMotionStartTime = 0;
    double initialPitchAngleDegrees = 0;
    double initalYDegrees = 0;

    @Override
    public void robotInit() {

        /**
         * The restoreFactoryDefaults method can be used to reset the configuration
         * parameters
         * in the SPARK MAX to their factory default state. If no argument is passed,
         * these
         * parameters will not persist between power cycles
         */
        // m_colorMatcher.addColorMatch(YELLOW);
        // m_colorMatcher.addColorMatch(WHITE);
        // m_colorMatcher.addColorMatch(PURPLE);
        // m_colorMatcher.addColorMatch(RED);
        // m_colorMatcher.addColorMatch(GREEN);
        // m_colorMatcher.addColorMatch(BLUE);
        imu.reset();
        armMotor.restoreFactoryDefaults();
        arm_clawMotor.restoreFactoryDefaults();
        other_clawMotor.restoreFactoryDefaults();
        // LEDSerialPort.writeString(Constants.LED_RESET);

        /**
         * In order to use PID functionality for a controller, a SparkMaxPIDController
         * object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = armMotor.getPIDController();

        // Encoder object created to display position values
        m_encoder = armMotor.getEncoder();

        // PID coefficients
        kP = 0.01;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMaxOutput = -0.2;
        kMinOutput = 0.2;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);

        // reversing the right side to match with forwards on the robot and forwards on
        // the controller
        leftMotors.setInverted(true);

        // make sure that the gyro resets to 0 at the start of the match
        imu.reset();

        // start the camera server on robot startup
       // CameraServer.startAutomaticCapture();
        CameraServer.startAutomaticCapture();
        m_chooser.addOption("Cube and stay", kNewAuto);
        m_chooser.addOption("Cube and drive back", kEverythingAuto);
        m_chooser.setDefaultOption("ONLY DRIVES OUT", kDefaultAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

        rotations = 0;

        // LED Lights Maybe? PWM Port 9
        // AddressableLED m_led = new AddressableLED(9);
        // AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
        // m_led.setLength(m_ledBuffer.getLength());
        // m_led.setData(m_ledBuffer);
        // m_led.start();


    }

    @Override
    public void robotPeriodic() {

        /**
         * The method GetColor() returns a normalized color value from the sensor and
         * can be
         * useful if outputting the color to an RGB LED or similar. To
         * read the raw color, use GetRawColor().
         * 
         * The color sensor works best when within a few inches from an object in
         * well lit conditions (the built in LED is a big help here!). The farther
         * an object is the more light from the surroundings will bleed into the
         * measurements and make it difficult to accurately determine its color.
         */

        // Color detectedColor = m_colorSensor.getColor();

        // The sensor returns a raw IR value of the infrared light detected, which means
        // distance

        // double IR = m_colorSensor.getIR();

        // String colorString;
        // ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        // if (match.confidence >= .95) {
        // if (match.color == RED) {
        // colorString = "Red";
        // } else if (match.color == YELLOW) {
        // colorString = "Cone";
        // } else if (match.color == BLUE) {
        // colorString = "Blue";
        // } else if (match.color == PURPLE) {
        // colorString = "Cube";
        // } else if (match.color == GREEN) {
        // colorString = "Green";
        // } else if (match.color == WHITE) {
        // colorString = "White";
        // } else {
        // colorString = "Unknown";
        // }
        // } else {
        // colorString = "Unknown";
        // // LED
        // // for (var i = 0; i<m_ledBuffer.getLength(); i++){
        // // m_led.setRGB(i, 255, 0, 0);
        // // }
        // }

        // // limit switches
        // SmartDashboard.putBoolean("Bimba MRS", m_bimbaMRS.get());
        // SmartDashboard.putBoolean("Photoelectric Switch", m_photoElectric.get());

        // // color sensor
        // SmartDashboard.putNumber("Red", detectedColor.red);
        // SmartDashboard.putNumber("Blue", detectedColor.blue);
        // SmartDashboard.putNumber("Green", detectedColor.green);
        // SmartDashboard.putNumber("Confidnce", match.confidence);
        // SmartDashboard.putString("Detected Color", colorString);

        // // smart dashboard
        // SmartDashboard.putNumber("Red", m_colorSensor.getRed());
        // SmartDashboard.putNumber("Green", m_colorSensor.getGreen());
        // SmartDashboard.putNumber("Blue", m_colorSensor.getBlue());
        // SmartDashboard.putNumber("IR", IR);

        // // proximity
        // int proximity = m_colorSensor.getProximity();
        // SmartDashboard.putNumber("Proximity", proximity);

        // // Ultrasonic Rangefinder (long range)
        // SmartDashboard.putNumber("Distance Long Sensor (volts)", getVoltageLong());
        // SmartDashboard.putNumber("Distance Long Sensor (real)", getDistanceLong());
        // // Ultrasonic Rangefinder(short range)
        // SmartDashboard.putNumber("Distance Short Sensor (volts)", getVoltageShort());
        // SmartDashboard.putNumber("Distance Short Sensor (real)", getDistanceShort());

        // put the gyro values to the dashboard for debugging purposes

        SmartDashboard.putNumber("Gyro_z", imu.getAngle());
        SmartDashboard.putNumber("Gyro_y", imu.getYComplementaryAngle());
        SmartDashboard.putNumber("Gyro_x", imu.getXComplementaryAngle());
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        m_pidController.setOutputRange(-1 * min, -1 * max);

        if ((p != kP)) {
            m_pidController.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            m_pidController.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            m_pidController.setD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            m_pidController.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            m_pidController.setFF(ff);
            kFF = ff;
        }
        m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);

        SmartDashboard.putNumber("SetPoint", rotations);
        SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
    }

    @Override
    public void teleopInit() {
        // make sure that the arm stops moving once teleop begins
        arm_clawMotor.set(0);
        other_clawMotor.set(0);
        armMotor.set(0);
        rotations = 0;
    }

    public void teleopPeriodic() {
        // makes sure that we only get this once; increases performance slightly
        int operatorPOV = operatorController.getPOV();

        //p is purple/Cube
        if(driverController.getXButtonPressed()){
            changeColor(Constants.LED_PURPLE);
        }
        //y is yellow/Cone
        if(driverController.getYButtonPressed()){
            changeColor(Constants.LED_YELLOW);
        }
        //c is clear
        if(driverController.getBButtonPressed()){
            changeColor(Constants.LED_RESET);
        }

        // if (driverController.getXButton()) {
        //     armMotor.set(0);

        // }

        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller

        // if((max != kMaxOutput) || (min != kMinOutput)) {

        // kMinOutput = min; kMaxOutput = max;
        // }

        /**
         * PIDController objects are commanded to a set point using the
         * SetReference() method.
         * 
         * The first parameter is the value of the set point, whose units vary
         * depending on the control type set in the second parameter.
         * 
         * The second parameter is the control type can be set to one of four
         * parameters:
         * com.revrobotics.CANSparkMax.ControlType.kDutyCycle
         * com.revrobotics.CANSparkMax.ControlType.kPosition
         * com.revrobotics.CANSparkMax.ControlType.kVelocity
         * com.revrobotics.CANSparkMax.ControlType.kVoltage
         */

        // add speed limit to the controllers to lower the normal speed (makes it go
        // slower)
        // uses left and right joystick for control
        drivetrain.tankDrive(Constants.DRIVETRAIN_CONTROL_SPEED_LIMIT * driverController.getLeftY(),
                Constants.DRIVETRAIN_CONTROL_SPEED_LIMIT * driverController.getRightY());

        if ((operatorController.getRightTriggerAxis() >= 0.2) && (operatorController.getLeftTriggerAxis() >= 0.2)) {
            currentClawAction = CLAW_MODE.stop;
        } else if (operatorController.getLeftTriggerAxis() >= 0.2) {
            currentClawAction = CLAW_MODE.in;
        } else if (operatorController.getRightTriggerAxis() >= 0.2) {
            currentClawAction = CLAW_MODE.out;
        } else { // don't do anything if none of them are pressed
            currentClawAction = CLAW_MODE.stop;
        }

        operateClaw(); // update the claw motors based on controller input

        if (operatorPOV == 0) { // plus button pressed in the "up" position
            currentArmAction = ARM_MODE.up;
        } else if (operatorPOV == 180) { // plus button pressed in the "down" position
            currentArmAction = ARM_MODE.down;
        } else if (operatorPOV == 90) { // plus button pressed in the "right" position
            currentArmAction = ARM_MODE.middle;
        } else if (operatorPOV == 270) { // plus button pressed in the "left" position
            currentArmAction = ARM_MODE.low;
        } else if (operatorController.getAButton()) {
            currentArmAction = ARM_MODE.moveHigherMid;
        } else if (operatorController.getBButton()) {
            currentArmAction = ARM_MODE.moveLowerMid;
        } else if (operatorController.getXButton()) {
            currentArmAction = ARM_MODE.moveHigherLow;
        } else if (operatorController.getYButton()) {
            currentArmAction = ARM_MODE.moveLowerHigh;
        }

        operateArm(); // update the motor speeds based on controller input
    }

    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        armMotor.set(0);
        System.out.println("Auto selected: " + m_autoSelected);
        currentAutoStage = AUTO_STAGE.start;
        // double initialAngle = imu.getXComplementaryAngle();
    }

    @Override
    public void autonomousPeriodic() {

        switch (m_autoSelected) {



            case kEverythingAuto:

            if (currentAutoStage == AUTO_STAGE.start) {
                currentAutoStage = AUTO_STAGE.lift_arm;
                currentArmAction = ARM_MODE.up;
                currentClawAction = CLAW_MODE.in;
                autoMotionStartTime = Timer.getFPGATimestamp(); // /reset T later use
            }

            else if (currentAutoStage == AUTO_STAGE.lift_arm) {
                if ((Timer.getFPGATimestamp() - autoMotionStartTime) >= 1.0) {
                    currentAutoStage = AUTO_STAGE.move_to_drop;
                    currentClawAction = CLAW_MODE.in;
                    autoMotionStartTime = Timer.getFPGATimestamp(); // /reset T later use

                } else {
                    operateArm();
                    operateClaw();
                }
            }

            else if (currentAutoStage == AUTO_STAGE.move_to_drop) {
                if ((Timer.getFPGATimestamp() - autoMotionStartTime) >= Constants.AUTO_DROP_OFF_DRIVE_TIME) {
                    currentAutoStage = AUTO_STAGE.release_cube;
                    currentClawAction = CLAW_MODE.out;
                    drivetrain.tankDrive(0, 0);
                    autoMotionStartTime = Timer.getFPGATimestamp();
                } else {
                    drivetrain.tankDrive(-0.55, -0.55); // negative is good, forward
                }
            }

            else if (currentAutoStage == AUTO_STAGE.release_cube) {
                if ((Timer.getFPGATimestamp() - autoMotionStartTime) >= Constants.AUTO_RELEASE_CUBE_TIME) {
                    currentArmAction = ARM_MODE.down;
                    autoMotionStartTime = Timer.getFPGATimestamp(); // reset T later use
                    currentClawAction = CLAW_MODE.stop;
                    currentAutoStage = AUTO_STAGE.move_out_of_safe_zone;
                    operateClaw();

                } else {
                    operateClaw();
                }
            }

            else if (currentAutoStage == AUTO_STAGE.move_out_of_safe_zone) {
                if ((Timer.getFPGATimestamp() - autoMotionStartTime) >= Constants.AUTO_BACK_OUT_TIME) {

                    drivetrain.tankDrive(0.0, 0.0);
                    currentAutoStage = AUTO_STAGE.end;
                    // System.out.println("stop");
                } else {
                    drivetrain.tankDrive(0.7, 0.7);
                    // currentClawAction = CLAW_MODE.cone;
                    operateClaw();
                    operateArm();

                    // System.out.println("go");
                }
            }

            else if (currentAutoStage == AUTO_STAGE.end) {
                drivetrain.tankDrive(0.0, 0.0);
                currentClawAction = CLAW_MODE.stop;
                operateClaw();
                currentArmAction = ARM_MODE.down;
                operateArm();

            }

            break;
    

            case kDefaultAuto:

                if (currentAutoStage == AUTO_STAGE.start) {
                    currentAutoStage = AUTO_STAGE.move_out_of_safe_zone;
                    autoMotionStartTime = Timer.getFPGATimestamp(); // reset T later use
                    System.out.println("started");
                }

                else if (currentAutoStage == AUTO_STAGE.move_out_of_safe_zone) {
                    if ((Timer.getFPGATimestamp() - autoMotionStartTime) >= Constants.AUTO_BACK_OUT_TIME) {
                        currentAutoStage = AUTO_STAGE.end;
                        autoMotionStartTime = 0; // reset for later use
                        drivetrain.tankDrive(0.0, 0.0);
                        System.out.println("stop");
                    } else {
                        drivetrain.tankDrive(0.7, 0.7);
                        System.out.println("go");
                    }
                } else if (currentAutoStage == AUTO_STAGE.end) {
                    drivetrain.tankDrive(0.0, 0.0);
                }
                break;
            // This aut code left the arm, moves forward a small amount, drops off a cube,
            // then leaves the zone.
            // if possible in future it was turn around grab another object,
            // turn back around and drop that object off probably on bottom or mid level

            /*
             * this right here seperetes them auto things. Just putting this here for
             * convience
             */

            case kNewAuto:
            default:

                if (currentAutoStage == AUTO_STAGE.start) {
                    currentAutoStage = AUTO_STAGE.lift_arm;
                    currentArmAction = ARM_MODE.up;
                    currentClawAction = CLAW_MODE.in;
                    autoMotionStartTime = Timer.getFPGATimestamp(); // /reset T later use
                }

                else if (currentAutoStage == AUTO_STAGE.lift_arm) {
                    if ((Timer.getFPGATimestamp() - autoMotionStartTime) >= 1.0) {
                        currentAutoStage = AUTO_STAGE.move_to_drop;
                        currentClawAction = CLAW_MODE.in;
                        autoMotionStartTime = Timer.getFPGATimestamp(); // /reset T later use

                    } else {
                        operateArm();
                        operateClaw();
                    }
                }

                else if (currentAutoStage == AUTO_STAGE.move_to_drop) {
                    if ((Timer.getFPGATimestamp() - autoMotionStartTime) >= Constants.AUTO_DROP_OFF_DRIVE_TIME) {
                        currentAutoStage = AUTO_STAGE.release_cube;
                        currentClawAction = CLAW_MODE.out;
                        drivetrain.tankDrive(0, 0);
                        autoMotionStartTime = Timer.getFPGATimestamp();
                    } else {
                        drivetrain.tankDrive(0, 0); // negative is good, forward
                    }
                }

                else if (currentAutoStage == AUTO_STAGE.release_cube) {
                    if ((Timer.getFPGATimestamp() - autoMotionStartTime) >= Constants.AUTO_RELEASE_CUBE_TIME) {
                        currentArmAction = ARM_MODE.down;
                        autoMotionStartTime = Timer.getFPGATimestamp(); // reset T later use
                        currentClawAction = CLAW_MODE.stop;
                        currentAutoStage = AUTO_STAGE.move_out_of_safe_zone;
                        operateClaw();

                    } else {
                        operateClaw();
                    }
                }

                else if (currentAutoStage == AUTO_STAGE.move_out_of_safe_zone) {
                    if ((Timer.getFPGATimestamp() - autoMotionStartTime) >= Constants.AUTO_SAFE_ZONE_REMOVAL) {

                        drivetrain.tankDrive(0.0, 0.0);
                        currentAutoStage = AUTO_STAGE.end;
                        // System.out.println("stop");
                    } else {
                        // drivetrain.tankDrive(0.7, 0.7);
                        drivetrain.tankDrive(0, 0);
                        // currentClawAction = CLAW_MODE.cone;
                        operateClaw();
                        operateArm();

                        // System.out.println("go");
                    }
                }

                else if (currentAutoStage == AUTO_STAGE.end) {
                    drivetrain.tankDrive(0.0, 0.0);
                    currentClawAction = CLAW_MODE.stop;
                    operateClaw();
                    currentArmAction = ARM_MODE.down;
                    operateArm();

                }

                break;
        }
    }

    /**
     * based on currentClawAction, control the claw motor based on what the current
     * action should be
     */
    public void operateClaw() {
        if (currentClawAction == CLAW_MODE.in) {
            arm_clawMotor.set(Constants.CLAW_IN_SPEED);
            other_clawMotor.set(Constants.CLAW_IN_SPEED);
            // other_clawMotor.set(0);

        } else if (currentClawAction == CLAW_MODE.out) {
            arm_clawMotor.set(Constants.CLAW_OUT_SPEED);
            other_clawMotor.set(Constants.CLAW_OUT_SPEED);
            // other_clawMotor.set(0);
        } else if (currentClawAction == CLAW_MODE.stop) {
            arm_clawMotor.set(Constants.CLAW_HOLD_SPEED);
            other_clawMotor.set(Constants.CLAW_HOLD_SPEED);
            // other_clawMotor.set(0);
        } else {
            System.err.println("Unknown CLAW_MODE set. Ignoring");
        }
    }

    /**
     * based on currentArmAction, control the arm motor based on what the current
     * action should be
     */
    private void operateArm() {
        if (currentArmAction == ARM_MODE.up) {
            rotations = -44;
            max = -0.4;
            min = 0.4;

        } else if (currentArmAction == ARM_MODE.down) {
            rotations = 0;
            max = -0.05;
            min = 0.05;
        } else if (currentArmAction == ARM_MODE.middle) { 
            rotations = -39;
            max = -0.4;
            min = 0.4;  
        } else if (currentArmAction == ARM_MODE.low) { //
            rotations = -4;
            max = -0.2;
            min = 0.2;
        } else if (currentArmAction == ARM_MODE.moveLowerMid) {
            rotations = -34.5;
            max = -0.4;
            min = 0.4;
        } else if (currentArmAction == ARM_MODE.moveHigherMid) {
            rotations = -42;
            max = -0.4;
            min = 0.4;
        } else if (currentArmAction == ARM_MODE.moveLowerHigh) {
            rotations = -43;
            max = -0.4;
            min = 0.4;
        } else if (currentArmAction == ARM_MODE.moveHigherLow) {
            rotations = -6;
            max = -0.2;
            min = 0.4;
        } else if (currentArmAction == ARM_MODE.stop) {
            armMotor.set(0);

        } else {
            System.err.println("Unknown ARM_MODE set. Ignoring");
        }

    }

    private void changeColor(String color){
        try{
            LEDController = new FileWriter("/dev/ttyACM0");
            LEDController.write(color);
            LEDController.close();
        } catch (Exception e) {
            try{
            FileWriter LEDControllerB = new FileWriter("/dev/ttyACM1");
            LEDControllerB.write(color);
            LEDControllerB.close();
            } catch (Exception f){}
        }
    }

}
