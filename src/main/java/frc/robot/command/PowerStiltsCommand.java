/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;

public class PowerStiltsCommand extends Command {
  public PowerStiltsCommand() {
    requires(Robot.climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.cargoCollector.setArticulatorPower(-0.2);
    Robot.climber.autoClimbDownA();
    Robot.climber.autoClimbDownB();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Robot.climber.getClimbLightSensorA() && Robot.climber.getClimbLightSensorB());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climber.climb(0, 0);
    Robot.cargoCollector.setArticulatorPower(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
