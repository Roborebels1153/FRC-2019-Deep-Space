/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsytems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Robot.RobotID;
import frc.robot.OI;
import frc.robot.lib.LidarLitePWM;
import edu.wpi.first.wpilibj.DigitalInput;


import frc.robot.lib.RebelDriveHelper;
import frc.robot.lib.DriveSignal;

/**
 * Drivetrain subsystem
 */
public class Drive extends Subsystem {

  // Drivetrain Motor Controllers
  private WPI_TalonSRX leftMaster;
  private WPI_TalonSRX leftFollower1;
  private WPI_TalonSRX leftFollower2;

  private WPI_TalonSRX rightMaster;
  private WPI_TalonSRX rightFollower1;
  private WPI_TalonSRX rightFollower2;

  // Rebel Drive Helper (Cheesy Drive)
  private RebelDriveHelper driveHelper;

  private ADXRS450_Gyro gyro;

  private LidarLitePWM mLidar;


  // private DifferentialDrive robotDrive;

  public int teleOpDriveSide; // this is a global variable which determines which side is the front of the bot

  public Drive() {
    leftMaster = new WPI_TalonSRX(RobotMap.LEFT_MASTER);
    leftFollower1 = new WPI_TalonSRX(RobotMap.LEFT_FOLLOWER_1);
    leftFollower2 = new WPI_TalonSRX(RobotMap.LEFT_FOLLOWER_2);

    rightMaster = new WPI_TalonSRX(RobotMap.RIGHT_MASTER);
    rightFollower1 = new WPI_TalonSRX(RobotMap.RIGHT_FOLLOWER_1);
    rightFollower2 = new WPI_TalonSRX(RobotMap.RIGHT_FOLLOWER_2);

    driveHelper = new RebelDriveHelper();
    // robotDrive = new DifferentialDrive(leftMaster, rightMaster);

    gyro = new ADXRS450_Gyro();
    
    if(Robot.robotID == RobotID.PROTO){
      mLidar = new LidarLitePWM(new DigitalInput(RobotMap.LIDAR));
    }

    teleOpDriveSide = (Robot.robotID == RobotID.PROTO ? -1 : 1); // at the start of the match, set one side to be the
                                                                 // front

    configMasterTalons();
    setFollowers();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Left Encoder Count", getLeftEncoderCount());
    SmartDashboard.putNumber("Right Encoder Count", getRightEncoderCount());

    SmartDashboard.putNumber("Left Motor Velocity", getLeftMotorSpeed());
    SmartDashboard.putNumber("Right Motor Velocity", getRightMotorSpeed());
    SmartDashboard.putNumber("Gyro Value", getGyro());

    SmartDashboard.putNumber("Lidar Distance", getLidarDistance());
  }

  public double getLidarDistance() {
    if(Robot.robotID == RobotID.PROTO){
      return mLidar.getDistance();
    }else{
      return 0;
    }
    
  }

  public void calibrateGyro() {
    gyro.calibrate();
  }

  public double getGyro() {
    return gyro.getAngle();
  }

  public void stopSubystem() {
    rightMaster.set(ControlMode.PercentOutput, 0);
    leftMaster.set(ControlMode.PercentOutput, 0);
  }

  public void setFollowers() {
    // configure left side followers
    leftFollower1.follow(leftMaster);
    leftFollower2.follow(leftMaster);

    // configure right side followers
    rightFollower1.follow(rightMaster);
    rightFollower2.follow(rightMaster);

  }

  /*
   * public void arcadeDrive() { robotDrive.arcadeDrive(0.8 *
   * Robot.oi.getDriverStick().getRawAxis(OI.JOYSTICK_LEFT_Y), -0.8
   * *Robot.oi.getDriverStick().getRawAxis(OI.JOYSTICK_RIGHT_X)); }
   */

  // start master talon configurations
  public void configMasterTalons() {
    leftMaster.set(ControlMode.PercentOutput, 0);
    rightMaster.set(ControlMode.PercentOutput, 0);

    rightMaster.configNominalOutputForward(0, Constants.kTimeoutMs);
    rightMaster.configNominalOutputReverse(0, Constants.kTimeoutMs);
    rightMaster.configPeakOutputForward(1, Constants.kTimeoutMs);
    rightMaster.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    leftMaster.configNominalOutputForward(0, Constants.kTimeoutMs);
    leftMaster.configNominalOutputReverse(0, Constants.kTimeoutMs);
    leftMaster.configPeakOutputForward(1, Constants.kTimeoutMs);
    leftMaster.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    leftMaster.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    leftFollower1.setNeutralMode(NeutralMode.Coast);
    leftFollower2.setNeutralMode(NeutralMode.Coast);
    rightFollower1.setNeutralMode(NeutralMode.Coast);
    rightFollower1.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Below is drive code which is used in the cheesy Drive Command
   */
  public void configDrive(ControlMode controlMode, double left, double right) {
    leftMaster.set(controlMode, left);
    rightMaster.set(controlMode, -right);
  }

  /**
   * Adjusting cheesy drive
   * 
   * @param value
   * @param deadband
   * @return
   */
  protected double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public void createDriveSignal(boolean squaredInputs) {
    boolean quickTurn = Robot.drive.quickTurnController();
    double rawMoveValue = Robot.oi.getDriverStick().getRawAxis(OI.JOYSTICK_LEFT_Y);
    double rawRotateValue = Robot.oi.getDriverStick().getRawAxis(OI.JOYSTICK_RIGHT_X);

    double moveValue = 0;
    double rotateValue = 0;
    if (squaredInputs == true) {
      double deadBandMoveValue = applyDeadband(rawMoveValue, 0.02);
      double deadBandRotateValue = applyDeadband(rawRotateValue, 0.02);
      moveValue = Math.copySign(deadBandMoveValue * deadBandMoveValue, deadBandMoveValue);
      rotateValue = Math.copySign(deadBandRotateValue * deadBandRotateValue, deadBandRotateValue);
    } else {
      rawMoveValue = moveValue;
      rotateValue = rawRotateValue;
    }

    DriveSignal driveSignal = driveHelper.rebelDrive(teleOpDriveSide * Constants.k_drive_coefficient * moveValue,
        Constants.k_turn_coefficient * rotateValue, quickTurn, false);
    Robot.drive.driveWithHelper(ControlMode.PercentOutput, driveSignal);

  }

  public void createHybridDriveSignal(boolean squaredInputs) {
    double rawLeftValue = Robot.oi.getDriverStick().getRawAxis(OI.JOYSTICK_TRIGGER_LEFT);
    double rawRightValue = Robot.oi.getDriverStick().getRawAxis(OI.JOYSTICK_TRIGGER_RIGHT);

    if (rawLeftValue > 0 || rawRightValue > 0) {
      createTankDriveSignal(squaredInputs);
    } else {
      createDriveSignal(squaredInputs);
    }
  }

  public void createTankDriveSignal(boolean squaredInputs) {
    double rawLeftValue = -Robot.oi.getDriverStick().getRawAxis(OI.JOYSTICK_TRIGGER_LEFT);
    double rawRightValue = -Robot.oi.getDriverStick().getRawAxis(OI.JOYSTICK_TRIGGER_RIGHT);

    double leftValue = 0;
    double rightValue = 0;
    if (squaredInputs == true) {
      double deadBandLeftValue = applyDeadband(rawLeftValue, 0.02);
      double deadBandRightValue = applyDeadband(rawRightValue, 0.02);
      leftValue = Math.copySign(deadBandLeftValue * deadBandLeftValue, deadBandLeftValue);
      rightValue = Math.copySign(deadBandRightValue * deadBandRightValue, deadBandRightValue);
    } else {
      leftValue = rawLeftValue;
      rightValue = rawRightValue;
    }

    DriveSignal driveSignal = driveHelper.tankDrive(teleOpDriveSide * Constants.k_drive_coefficient * (teleOpDriveSide == -1 ? rightValue : leftValue),
        teleOpDriveSide * Constants.k_drive_coefficient * (teleOpDriveSide == -1 ? leftValue : rightValue));
    Robot.drive.driveWithHelper(ControlMode.PercentOutput, driveSignal);
  }

  public void driveWithHelper(ControlMode controlMode, DriveSignal driveSignal) {
    this.configDrive(controlMode, driveSignal.getLeft(), driveSignal.getRight());
  }

  public boolean quickTurnController() {
    if (Robot.oi.getDriverStick().getRawAxis(OI.JOYSTICK_LEFT_Y) < 0.2
        && Robot.oi.getDriverStick().getRawAxis(OI.JOYSTICK_LEFT_Y) > -0.2) {
      return true;
    } else {
      return false;
    }
  }

  public void cheesyDriveWithoutJoysticks(double move, double rotate) {
    double moveValue = move;
    double rotateValue = rotate;
    DriveSignal driveSignal = driveHelper.rebelDrive(-1 * moveValue, rotateValue, true, false);
    Robot.drive.driveWithHelper(ControlMode.PercentOutput, driveSignal);
  }

  // set mag encoders as feedback device for taloms
  public void configTalonFeedback() {
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);
  }

  public void configRightMotionMagic(double rightCoefficient) {
    /* Set relevant frame periods to be at least as fast as periodic rate */
    rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
    rightMaster.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);

    /* set acceleration and vcruise velocity - see documentation */
    int cruiseVelocity = (int) (rightCoefficient * Constants.kMotionMagicCruiseVelocity);
    System.out.println("Setting right cruiseVelocity to " + cruiseVelocity);
    rightMaster.configMotionCruiseVelocity(cruiseVelocity, Constants.kTimeoutMs);
    rightMaster.configMotionAcceleration(Constants.kMotionMagicAcceleration, Constants.kTimeoutMs);
  }

  public void configLeftMotionMagic(double leftCoefficient) {
    /* Set relevant frame periods to be at least as fast as periodic rate */
    leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
    leftMaster.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);

    /* set acceleration and vcruise velocity - see documentation */
    int cruiseVelocity = (int) (leftCoefficient * Constants.kMotionMagicCruiseVelocity);
    System.out.println("Setting left cruiseVelocity to " + cruiseVelocity);
    leftMaster.configMotionCruiseVelocity(cruiseVelocity, Constants.kTimeoutMs);
    leftMaster.configMotionAcceleration(Constants.kMotionMagicAcceleration, Constants.kTimeoutMs);
  }

  public void enactRightMotorMotionMagic(double targetPos) {
    rightMaster.set(ControlMode.MotionMagic, targetPos);
  }

  public void enactLeftMotorMotionMagic(double targetPos) {
    leftMaster.set(ControlMode.MotionMagic, targetPos);
  }

  // reset daaa enkodurs
  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public int getLeftEncoderCount() {
    return leftMaster.getSelectedSensorPosition();
  }

  public double getLeftMotorOutput() {
    return leftMaster.getMotorOutputPercent();
  }

  public int getLeftMotorSpeed() {
    return leftMaster.getSelectedSensorVelocity();
  }

  public int getRightEncoderCount() {
    return rightMaster.getSelectedSensorPosition();
  }

  public double getRightMotorOutput() {
    return rightMaster.getMotorOutputPercent();
  }

  public int getRightMotorSpeed() {
    return rightMaster.getSelectedSensorVelocity();
  }

  public boolean stopMotion() {
    return Robot.oi.getDriverStick().getRawButton(1);
  }
}
