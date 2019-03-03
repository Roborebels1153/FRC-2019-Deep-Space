/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ScoreHatchCommand extends Command {
  long startTime;
  private double armDownTime;
  private double commandFinishTime;
  private double speed;

  public ScoreHatchCommand(double armDownTimme, double finishTime,  double driveSpeed) {
    requires(Robot.hatchCollector);
    requires(Robot.drive);
    armDownTime = armDownTimme;
    speed = driveSpeed;
    commandFinishTime = finishTime;
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
    //Robot.drive.cheesyDriveWithoutJoysticks(0.3, 0);
     if  (System.currentTimeMillis() - this.startTime < (armDownTime * 1000)) {
      Robot.hatchCollector.setArticulatorPower(speed);  
    } else {
        if ((System.currentTimeMillis() - startTime) > (commandFinishTime * 1000)) {

        } else {
          Robot.hatchCollector.collectReverse();
        }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (System.currentTimeMillis() - startTime) > (commandFinishTime * 1000);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Command Ended");
    Robot.drive.cheesyDriveWithoutJoysticks(0, 0);
    Robot.cargoCollector.collectStop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
