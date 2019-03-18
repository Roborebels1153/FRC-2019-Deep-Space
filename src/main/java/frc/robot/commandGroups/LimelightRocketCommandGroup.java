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
import frc.robot.command.TankDriveCommand;
import frc.robot.command.WaitCommand;

public class LimelightRocketCommandGroup extends CommandGroup {

  public LimelightRocketCommandGroup() {
    addSequential(new DriveDistanceTimeCommand(0.5, 0.5, 0));
    addSequential(new WaitCommand(1));

    addSequential(new TankDriveCommand(0.5, 0.3, 1));
    
    addSequential(new VisionDrive(7, 20, 5));
  }
}
