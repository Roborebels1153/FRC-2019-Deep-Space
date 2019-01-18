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
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.OI;

import frc.robot.lib.RebelDriveHelper;
import frc.robot.lib.DriveSignal;


/**
 * Drivetrain subsystem
 */
public class Drive extends Subsystem {

  //Drivetrain Motor Controllers
  private WPI_TalonSRX leftMaster;
  private VictorSPX leftFollower1;
  private VictorSPX leftFollower2;

  private WPI_TalonSRX rightMaster;
  private VictorSPX rightFollower1;
  private VictorSPX rightFollower2;

  //Rebel Drive Helper (Cheesy Drive)
  private RebelDriveHelper driveHelper;

  public Drive() {
    leftMaster = new WPI_TalonSRX(RobotMap.LEFT_MASTER);
    leftFollower1 = new VictorSPX(RobotMap.LEFT_FOLLOWER_1);
    leftFollower2 = new VictorSPX(RobotMap.LEFT_FOLLOWER_2);

    rightMaster = new WPI_TalonSRX(RobotMap.RIGHT_MASTER);
    rightFollower1 = new VictorSPX(RobotMap.RIGHT_FOLLOWER_1);
    rightFollower2 = new VictorSPX(RobotMap.RIGHT_FOLLOWER_2);
    
    driveHelper = new RebelDriveHelper();

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

  /**
	 * Below is drive code which is used in the cheesy Drive Command
	 */
	public void configDrive(ControlMode controlMode, double left, double right) {
		leftMaster.set(controlMode, left);
		rightMaster.set(controlMode, -right);
	}

	/**
	 * Adjusting cheesy drive
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

    DriveSignal driveSignal = driveHelper.rebelDrive(-1 * Constants.k_drive_coefficient * moveValue, 
                                                      Constants.k_turn_coefficient * rotateValue, quickTurn, false);
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
