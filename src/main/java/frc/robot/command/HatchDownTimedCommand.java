/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class HatchDownTimedCommand extends Command {
  long startTime;
  private double time;
  private double speed;

  public HatchDownTimedCommand(double driveTime, double driveSpeed) {
    requires(Robot.hatchCollector);
    time = driveTime;
    speed = driveSpeed;
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
    Robot.drive.cheesyDriveWithoutJoysticks(-speed, 0);
    System.out.println("Executing");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (System.currentTimeMillis() - startTime) > (time*1000);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Command Ended");
    Robot.drive.cheesyDriveWithoutJoysticks(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
