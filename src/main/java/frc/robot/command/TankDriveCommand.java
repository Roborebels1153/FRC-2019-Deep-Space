/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TankDriveCommand extends Command {
  private double right;
  private double left;
  private double time;
  long startTime;

  public TankDriveCommand(double rightValue, double leftValue, double driveTime) {
    requires(Robot.drive);
    right = rightValue;
    left = leftValue;
    time = driveTime;
  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drive.tankDriveNoJoystick(right, left);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (System.currentTimeMillis() - startTime) > (time*1000);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.stopMotion();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
