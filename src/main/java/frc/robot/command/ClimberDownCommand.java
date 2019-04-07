/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command;
import  frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ClimberDownCommand extends Command {

  private boolean stopForLightSensor;

  public ClimberDownCommand(boolean stopForLightSensor) {
    requires (Robot.climber);
    this.stopForLightSensor = stopForLightSensor;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Test Climb Logic");
    
    /** 
    if (stopForLightSensor) {
      if (Robot.climber.getClimbLightSensorA()  == false && Robot.climber.getClimbLightSensorB() == false) {
        Robot.climber.climb(-0.75, -0.75);
      } else {
        Robot.climber.climb(0, 0);
      }
    } else {
      Robot.climber.climb(-0.75, -0.75);
    }
    */
    Robot.climber.climbWithLimitSwitch(-1, -1);
    
   // Robot.climber.climb(-0.75, -0.75);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
