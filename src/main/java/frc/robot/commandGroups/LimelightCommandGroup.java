/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.command.VisionDrive;
import frc.robot.command.DriveDistanceTimeCommand;
import frc.robot.command.HatchDownTimedCommand;
import frc.robot.command.WaitCommand;

public class LimelightCommandGroup extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LimelightCommandGroup() {
    
    addSequential(new DriveDistanceTimeCommand(0.5, 0.5));
    addSequential(new WaitCommand(0.5));
    addSequential(new VisionDrive(6));
    addSequential(new HatchDownTimedCommand(1, 0.4));
  }
}
