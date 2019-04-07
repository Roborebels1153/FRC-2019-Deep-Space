/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.command.BringArmDownCommand;
import frc.robot.command.PowerStiltsWithArmCommand;
import frc.robot.command.*;

public class AutomatedClimbCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutomatedClimbCommand() {
    addSequential(new BringArmDownCommand(-3050));
    addSequential(new PowerStiltsWithArmCommand(-6500));
    addSequential(new PowerStiltsCommand());
    addSequential(new DriveDistanceTimeCommand(1, -0.4, 0));
    //addSequential(new PowerStiltsCommand());

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
