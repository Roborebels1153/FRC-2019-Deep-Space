/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;

public class PowerStiltsStopDrivingCommand extends Command {

  double encoderThreshold;
  public PowerStiltsStopDrivingCommand(double encoderThreshold) {
    requires(Robot.cargoCollector);
    requires(Robot.climber);
    this.encoderThreshold = encoderThreshold;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drive.cheesyDriveWithoutJoysticks(-0.6, 0);
    Robot.climber.autoClimbDownA();
    Robot.climber.autoClimbDownB();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Robot.climber.getClimbAEncoder() < encoderThreshold || Robot.climber.getClimbBEncoder() < encoderThreshold);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Stopping driving");
    Robot.drive.cheesyDriveWithoutJoysticks(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
