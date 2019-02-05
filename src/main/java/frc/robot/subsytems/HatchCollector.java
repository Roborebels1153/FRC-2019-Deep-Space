/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsytems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class HatchCollector extends Subsystem {

    private Victor mRoller;
    private Victor mArticulator;

    private static final double kCollectPowerForward = -1;
    private static final double kCollectPowerReverse = 1;
    private static final double kCollectPowerStop = 0;

    public HatchCollector() {
        mRoller = new Victor(RobotMap.HATCH_ROLLER);
        mArticulator = new Victor(RobotMap.HATCH_ARTICULATOR);
        mRoller.setInverted(true);
        mArticulator.setInverted(true);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Roller Speed Value", getRollerPower());
        SmartDashboard.putNumber("Articulator Speed Value", getArticulatorPower());
    }

    public void setArticulatorPower(double value) {
        mArticulator.set(value);
    }

    public void setRollerPower(double value) {
        mRoller.set(value);
    }

    public void collectForward() {
        setRollerPower(kCollectPowerForward);
    }

    public void collectReverse() {
        setRollerPower(kCollectPowerReverse);
    }

    public void collectStop() {
        setRollerPower(kCollectPowerStop);
    }

    public double getRollerPower() {
        return mRoller.get();
    }

    public double getArticulatorPower() {
        return mArticulator.get();
    }

    @Override
    public void initDefaultCommand() {
    }
}