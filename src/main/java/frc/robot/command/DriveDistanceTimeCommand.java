/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveDistanceTimeCommand extends Command {
  long startTime;
  private double time;
  private double speed;
  private double turnSpeedA;

  public DriveDistanceTimeCommand(double driveTime, double driveSpeed, double turnSpeed) {
    requires(Robot.drive);
    time = driveTime;
    speed = driveSpeed;
    turnSpeedA = turnSpeed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Starting robot drive");
    startTime = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drive.cheesyDriveWithoutJoysticks(speed, turnSpeedA);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (System.currentTimeMillis() - startTime) > (time*1000);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Timed Drive Ended");
    Robot.drive.cheesyDriveWithoutJoysticks(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
