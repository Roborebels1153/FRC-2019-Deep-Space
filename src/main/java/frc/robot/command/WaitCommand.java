/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class WaitCommand extends Command {
  long startTime;
  private double time;

  public WaitCommand(double waitTime) {
    requires(Robot.hatchCollector);
    time = waitTime;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Starting robot Wait");
    startTime = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.hatchCollector.setArticulatorPower(0.3);
  }

  

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (System.currentTimeMillis() - startTime) > (time*1000);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Wait Ended");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
