// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;



public class Robot extends TimedRobot {
    frc.robot.Constants constants;

    /**
     * DRIVETRAIN
    */

    //physical motor interfaces
    PWMSparkMax leftFrontSpark = new PWMSparkMax(Constants.DRIVETRAIN_LEFT_FRONT_SPARK);
    PWMSparkMax leftBackSpark = new PWMSparkMax(Constants.DRIVETRAIN_LEFT_BACK_SPARK);
    PWMSparkMax rightFrontSpark = new PWMSparkMax(Constants.DRIVETRAIN_RIGHT_FRONT_SPARK);
    PWMSparkMax rightBackSpark = new PWMSparkMax(Constants.DRIVETRAIN_RIGHT_BACK_SPARK);

    //puts the physical motors into a single group for synchronous control
    MotorControllerGroup leftMotors = new MotorControllerGroup(leftFrontSpark, leftBackSpark);
    MotorControllerGroup rightMotors = new MotorControllerGroup(rightFrontSpark, rightBackSpark);
    private DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);


    /**
     * ARM CONTROL
     */
    CANSparkMax armMotor = new CANSparkMax(1, MotorType.kBrushless);
    private PWMSparkMax arm_clawMotor = new PWMSparkMax(Constants.CLAW_SPARK);
    DigitalInput arm_topLimitSwitch = new DigitalInput(Constants.TOP_LIMIT);
    DigitalInput arm_bottomLimitSwitch = new DigitalInput(Constants.BOTTOM_LIMIT);
    DigitalInput arm_clawLimitSwitch = new DigitalInput(Constants.CLAW_LIMIT);


    /**
     * HIDs
     */
    private XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER);
    private XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER);


    /**
     * MOTION MEASUREMENT DEVICES
     */
    AHRS ahrs; // NavX
    ADIS16470_IMU the_gyro = new ADIS16470_IMU(); // RoboRIO IMU


    /**
     * Claw SPEEDS
     */
    static final double clawOpenSpeed = 1;
    static final double clawCloseSpeed = -1;

    boolean autoBalanceYMode;


    /**
     * ARM STATES
     */
    enum ARM_MODE{
        up,
        down,
        middle,
        stop
    };

    ARM_MODE currentArmAction = ARM_MODE.stop;
    double armMotionStartTime = 0; // used to time the arm motion in "middle"


    /**
     * CLAW STATES
     */
    enum CLAW_MODE{
        cone,
        cube,
        open, // not sure what this one does, so make sure to rename it
        tighter,
        stop,
        release
    };
    CLAW_MODE currentClawAction = CLAW_MODE.stop;
    double clawMotionStartTime = 0;


    /**
     * AUTO STAGES
     */
    enum AUTO_STAGE{
        start,
        grab_cone,
        lift_arm,
        move_to_drop,
        release_cone,
        move_out_of_safe_zone,
        drop_arm,
        drive_to_platform,
        end
    };
    AUTO_STAGE currentAutoStage = AUTO_STAGE.start;
    double autoMotionStartTime = 0;

    @Override
    public void robotInit(){
        //reversing the right side to match with forwards on the robot and forwards on the controller
        leftMotors.setInverted(true);

        //make sure that the gyro resets to 0 at the start of the match
        the_gyro.reset();

        //start the camera server on robot startup
        CameraServer.startAutomaticCapture();
    }

    @Override
    public void robotPeriodic(){
        //put the gyro values to the dashboard for debugging purposes
        SmartDashboard.putNumber("Gyro_z", the_gyro.getAngle());
        SmartDashboard.putNumber("Gyro_y", the_gyro.getYComplementaryAngle());
        SmartDashboard.putNumber("Gyro_x", the_gyro.getXComplementaryAngle());
    }

    @Override
    public void teleopInit(){
        //make sure that the arm stops moving once teleop begins
        arm_clawMotor.set(0);
        armMotor.set(0);
    }

    public void teleopPeriodic(){
        // add speed limit to the controllers to lower the normal speed (makes it go slower)
        // uses left and right joystick for control
        drivetrain.tankDrive(Constants.DRIVETRAIN_CONTROL_SPEED_LIMIT * driverController.getLeftY(),
            Constants.DRIVETRAIN_CONTROL_SPEED_LIMIT * operatorController.getRightY());

        //makes sure that we only get this once; increases performance slightly
        int operatorPOV = operatorController.getPOV();

        if(currentArmAction == ARM_MODE.stop){ // check to see if the arm is doing something, if not, then read input

            if(operatorPOV == 0) { // plus button pressed in the "up" position
                currentArmAction = ARM_MODE.up;
            } else if (operatorPOV == 180) { // plus button pressed in the "down" position
                currentArmAction = ARM_MODE.down;
            } else if (operatorPOV == 90) { // plus button pressed in the "up" position
                currentArmAction = ARM_MODE.middle;
            }

        }

        operateArm(); // update the motor speeds based on controller input

        if(currentClawAction == CLAW_MODE.stop){ // only accept input if the claw has finished (AKA, not doing anything)
            if(operatorController.getAButton()) {
                currentClawAction = CLAW_MODE.open;
            } else if (operatorController.getBButton()) {
                currentClawAction = CLAW_MODE.cube;
            } else if (operatorController.getYButton()) {
                currentClawAction = CLAW_MODE.cone;
            } else if (operatorController.getXButton()){
                currentClawAction = CLAW_MODE.tighter;
            } else { // don't do anything if none of them are pressed
                currentClawAction = CLAW_MODE.stop;
            }
        }

        operateClaw(); // update the claw motors based on controller input
    }

    @Override
    public void autonomousPeriodic(){
        if(currentAutoStage == AUTO_STAGE.start){
            currentClawAction = CLAW_MODE.cone;
        } else if(currentAutoStage == AUTO_STAGE.grab_cone) {
            if(currentClawAction == CLAW_MODE.stop) { // if the claw mode is stopped, we know it finished
                currentAutoStage = AUTO_STAGE.lift_arm;
                currentArmAction = ARM_MODE.up;
                autoMotionStartTime = Timer.getFPGATimestamp();
            } else {
                operateClaw();
            }
        } else if (currentAutoStage == AUTO_STAGE.lift_arm) {
            if(currentArmAction == ARM_MODE.stop){ // if the arm mode stopped, then we know we finished
                currentAutoStage = AUTO_STAGE.move_to_drop;
                autoMotionStartTime = Timer.getFPGATimestamp();
            } else {
                operateArm();
            }
        } else if (currentAutoStage == AUTO_STAGE.move_to_drop) {
            if((Timer.getFPGATimestamp() - autoMotionStartTime) >= Constants.AUTO_DROP_OFF_DRIVE_TIME){
                currentAutoStage = AUTO_STAGE.release_cone;
                currentClawAction = CLAW_MODE.release;
                autoMotionStartTime = 0; // reset for later use
                drivetrain.tankDrive(0.0,0.0);
            } else {
                drivetrain.tankDrive(-0.5,0.5);
            }
        } else if (currentAutoStage == AUTO_STAGE.release_cone){
            if(currentClawAction == CLAW_MODE.stop){
                currentAutoStage = AUTO_STAGE.move_out_of_safe_zone;
            } else {
                operateClaw();
            }
        } else if (currentAutoStage == AUTO_STAGE.move_out_of_safe_zone){
            if((Timer.getFPGATimestamp() - autoMotionStartTime) >= Constants.AUTO_SAFE_ZONE_REMOVAL){
                currentAutoStage = AUTO_STAGE.drop_arm;
                currentArmAction = ARM_MODE.down;
                autoMotionStartTime = 0; // reset for later use
                drivetrain.tankDrive(0.0,0.0);
            } else {
                drivetrain.tankDrive(0.5,-0.5);
            }
        } else if (currentAutoStage == AUTO_STAGE.drop_arm){
            if(currentArmAction == ARM_MODE.stop){
                currentAutoStage = AUTO_STAGE.end;
            } else {
                operateArm();
            }
        }
    }

    /**
     * based on currentArmAction, control the arm motor based on what the current action should be
     */
    private void operateArm(){
        if(currentArmAction == ARM_MODE.up){
            if(arm_topLimitSwitch.get()){ //tests to see if the arm has hit its upper limit
                currentArmAction = ARM_MODE.stop;
            } else {
                armMotor.set(Constants.ARM_EXTEND_SPEED); //allow the arm to go further
            }
        } else if (currentArmAction == ARM_MODE.down){
            if(arm_bottomLimitSwitch.get()){ // tests to see if the arm has hit its lower limit
                currentArmAction = ARM_MODE.stop;
            } else {
                armMotor.set(Constants.ARM_RETRACT_SPEED);
            }
        } else if (currentArmAction == ARM_MODE.middle){
            if(armMotionStartTime == 0){ //if we have not set the time, then we are going up instead of down

                if(arm_topLimitSwitch.get()){ //check to see if it hit the top 
                    armMotor.set(0);
                    armMotionStartTime = Timer.getFPGATimestamp();
                } else {
                    armMotor.set(Constants.ARM_EXTEND_SPEED);
                }

            } else { //we have set the start time

                // check to see if we have been reversing for more than ARM_MIDDLE_REVERSE_TIME (seconds)
                // >= because if only equal, if the time is longer than that time, we will smash into our robot
                //
                // may be amusing, but also annoying :)
                if((Timer.getFPGATimestamp() - armMotionStartTime) >= Constants.ARM_MIDDLE_REVERSE_TIME){
                    armMotor.set(0);
                    armMotionStartTime = 0; // reset start time to 0 because we depend on that being true to get to here
                    currentArmAction = ARM_MODE.stop;
                } else { // has not quite gone for long enough
                    armMotor.set(Constants.ARM_RETRACT_SPEED);
                }

            }
        } else if (currentArmAction == ARM_MODE.stop) {
            armMotor.set(0);
        } else {
            System.err.println("Unknown ARM_MODE set. Ignoring");
        }

        operateClaw();
    }

    /**
     * based on currentClawAction, control the claw motor based on what the current action should be
     */
    public void operateClaw(){
        if(currentClawAction == CLAW_MODE.cone) {
            if(clawMotionStartTime == 0){ // check if timer has started
                if(arm_clawLimitSwitch.get()){ // go to the most-open position
                    arm_clawMotor.set(0);
                    clawMotionStartTime = Timer.getFPGATimestamp();
                } else {
                    arm_clawMotor.set(Constants.CLAW_OPEN_SPEED);
                }
            } else { // timer has started
                if( (Timer.getFPGATimestamp() - clawMotionStartTime) >= Constants.CLAW_CONE_CLOSE_TIME){
                    clawMotionStartTime = 0; // reset time to 0 so code does not break
                    currentClawAction = CLAW_MODE.stop;
                } else {
                    arm_clawMotor.set(Constants.CLAW_CLOSE_SPEED);
                }
            }
        } else if (currentClawAction == CLAW_MODE.cube) {
            if(clawMotionStartTime == 0){ // check if timer has started
                if(arm_clawLimitSwitch.get()){ // go to the most-open position
                    arm_clawMotor.set(0);
                    clawMotionStartTime = Timer.getFPGATimestamp();
                } else {
                    arm_clawMotor.set(Constants.CLAW_OPEN_SPEED);
                }
            } else { // timer has started
                if( (Timer.getFPGATimestamp() - clawMotionStartTime) >= Constants.CLAW_CUBE_CLOSE_TIME){
                    clawMotionStartTime = 0; // reset time to 0 so code does not break
                    currentClawAction = CLAW_MODE.stop;
                } else {
                    arm_clawMotor.set(Constants.CLAW_CLOSE_SPEED);
                }
            }
        } else if (currentClawAction == CLAW_MODE.open) {
            if(clawMotionStartTime == 0){ // check if timer has started
                if(arm_clawLimitSwitch.get()){ // go to the most-open position
                    arm_clawMotor.set(0);
                    clawMotionStartTime = Timer.getFPGATimestamp();
                } else {
                    arm_clawMotor.set(Constants.CLAW_OPEN_SPEED);
                }
            } else { // timer has started
                if( (Timer.getFPGATimestamp() - clawMotionStartTime) >= Constants.CLAW_OPEN_CLOSE_TIME){
                    clawMotionStartTime = 0; // reset time to 0 so code does not break
                    currentClawAction = CLAW_MODE.stop;
                } else {
                    arm_clawMotor.set(Constants.CLAW_CLOSE_SPEED);
                }
            }
        } else if (currentClawAction == CLAW_MODE.tighter){
            arm_clawMotor.set(Constants.CLAW_CLOSE_SPEED);
        } else if (currentClawAction == CLAW_MODE.stop) {
            arm_clawMotor.set(0);
        } else {
            System.err.println("Unknown CLAW_MODE set. Ignoring");
        }
    }

    // NOTE: I am assuming that this works, so I am keeping it this way (just some reformatting)
    public void balanceBot() {
        //This command is used to balance the robot if the angle reading on a gyroscope is more than ROBOT_OFF_BALANCE_THRESHOLD degrees off.
        double xAxisRate = 0;   
  
        //Need to figure out which one is correct
        //double pitchAngleDegrees = the_gyro.getAngle();
       //double pitchAngleDegrees = the_gyro.getXComplementaryAngle();
        double pitchAngleDegrees = the_gyro.getYComplementaryAngle();
        
        boolean isBalanced = false;
          
        while(isBalanced == false){
            if (!autoBalanceYMode && (Math.abs(180-pitchAngleDegrees) >= Math.abs(Constants.ROBOT_OFF_BALANCE_THRESHOLD))) {
                autoBalanceYMode = true;
                isBalanced = false;
                pitchAngleDegrees = getGyroAngle();
            } else if ( autoBalanceYMode && (Math.abs(180-pitchAngleDegrees) <= Math.abs(Constants.ROBOT_ON_BALANCE_THRESHOLD))) {
                autoBalanceYMode = false;
                isBalanced = true;
                pitchAngleDegrees = getGyroAngle();
            }
      
            if ( autoBalanceYMode ) {
                xAxisRate = Math.sin(degreesToRadians(pitchAngleDegrees)) * -1;
            }
            drivetrain.tankDrive(xAxisRate, xAxisRate);
            Timer.delay(0.005);
        }
        drivetrain.tankDrive(0, 0);
    }

    private double getGyroAngle(){
        return the_gyro.getYComplementaryAngle();
    }

    private double degreesToRadians(double degrees){
        return (degrees/180)*Math.PI;
    }
}