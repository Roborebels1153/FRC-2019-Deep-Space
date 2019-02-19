/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsytems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Robot.RobotID;

public class HatchCollector extends Subsystem {

    private Victor mRoller;
    private Victor mArticulator;
    private WPI_TalonSRX mRollerTalon;
    private WPI_TalonSRX mArticulatorTalon;
    private DigitalInput limitSwitchA;
    private DigitalInput limitSwitchB;

    private static final double kCollectPowerForward = 1;
    private static final double kCollectPowerReverse = -1;
    private static final double kCollectPowerStop = 0;

    public HatchCollector() {
        if (Robot.robotID  == RobotID.FINAL) {
            mRollerTalon = new WPI_TalonSRX(RobotMap.HATCH_ROLLER_TALON);
            mArticulatorTalon = new WPI_TalonSRX(RobotMap.HATCH_ARTICULATOR_TALON);
            mRollerTalon.setInverted(true);
            mArticulatorTalon.setInverted(true);
        } else {
            mRoller = new Victor(RobotMap.HATCH_ROLLER);
            mArticulator = new Victor(RobotMap.HATCH_ARTICULATOR);
            mRoller.setInverted(true);
            mArticulator.setInverted(true);
        }
        limitSwitchA = new DigitalInput(RobotMap.HATCH_LIMIT_SWITCH_A);
        limitSwitchB = new DigitalInput(RobotMap.HATCH_LIMIT_SWITCH_B);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Roller Speed Value", getRollerPower());
        SmartDashboard.putNumber("Articulator Speed Value", getArticulatorPower());
        SmartDashboard.putBoolean("Hatch Limit Switch", getHatchLimitSwitchA());
    }

    public void stopSubsystem() {
        setRollerPower(0);
        setArticulatorPower(0);
    }

    public boolean getHatchLimitSwitchA() {
        return limitSwitchA.get();
    }

    public boolean getHatchLimitSwitchB() {
        return limitSwitchB.get();
    }

    public void setArticulatorPower(double value) {
        if (Robot.robotID == RobotID.FINAL) {
            mArticulatorTalon.set(ControlMode.PercentOutput, value);
        } else {
            mArticulator.set(value);
        }
    }

    public void setRollerPower(double value) {
        if (Robot.robotID == RobotID.FINAL) {
            mRollerTalon.set(ControlMode.PercentOutput, value);
        } else {
            mRoller.set(value);
        }   
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
        if (Robot.robotID == RobotID.FINAL) {
            return mRollerTalon.getMotorOutputPercent();
        } else {
            return mRoller.get();
        }   
    }

    public double getArticulatorPower() {
        if (Robot.robotID == RobotID.FINAL) {
            return mArticulatorTalon.getMotorOutputPercent();
        } else {
            return mArticulator.get();
        }   
    }

    @Override
    public void initDefaultCommand() {
    }
}