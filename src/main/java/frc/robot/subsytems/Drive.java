/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsytems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {

  private WPI_TalonSRX leftMaster;
  private VictorSPX leftFollower1;
  private VictorSPX leftFollower2;

  private WPI_TalonSRX rightMaster;
  private VictorSPX rightFollower1;
  private VictorSPX rightFollower2;

  public Drive() {
    leftMaster = new WPI_TalonSRX(RobotMap.LEFT_MASTER);
    leftFollower1 = new VictorSPX(RobotMap.LEFT_FOLLOWER_1);
    leftFollower2 = new VictorSPX(RobotMap.LEFT_FOLLOWER_2);

    rightMaster = new WPI_TalonSRX(RobotMap.RIGHT_MASTER);
    rightFollower1 = new VictorSPX(RobotMap.RIGHT_FOLLOWER_1);
    rightFollower2 = new VictorSPX(RobotMap.RIGHT_FOLLOWER_2);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setFollowers() {
    // configure left side followers
    leftFollower1.follow(leftMaster);
    leftFollower2.follow(leftMaster);

    // configure right side followers
    rightFollower1.follow(rightMaster);
    rightFollower2.follow(rightMaster);

  }

  // set both master talons to the preferred control mode and feed the values for
  // both talons
  public void setControlMode(ControlMode controlMode, double leftValue, double rightValue) {
    leftMaster.set(controlMode, leftValue);
    rightMaster.set(controlMode, rightValue);
  }

  //start master talon configurations
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
  }

  //set mag encoders as feedback device for taloms
  public void configTalonFeedback() {
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);
  }

  //reset daaa enkodurs
  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public void getEncoderCount(WPI_TalonSRX talon) {
    talon.getSelectedSensorPosition();
  }

  public void getMotorOutput(WPI_TalonSRX talon) {
    talon.getMotorOutputPercent();
  }

  public void getMotorSpeed(WPI_TalonSRX talon) {
    talon.getSelectedSensorVelocity();
  }
}
