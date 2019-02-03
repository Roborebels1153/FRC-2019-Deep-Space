/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsytems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class HatchCollector extends Subsystem {

    private Victor mRoller;
    private Victor mArticulator;

    public HatchCollector() {
        mRoller = new Victor(RobotMap.HATCH_ROLLER);
        mArticulator = new Victor(RobotMap.HATCH_ARTICULATOR);
    }

    public void setArticulatorPower(double value) {
        mArticulator.set(value);
    }

    public void setRollerPower(double value) {
        mRoller.set(value);
    }

    @Override
    public void initDefaultCommand() {
    }
}