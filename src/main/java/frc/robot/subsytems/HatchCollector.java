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
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class HatchCollector extends Subsystem {

   
    private WPI_TalonSRX mRollerTalon;
    private WPI_TalonSRX mArticulatorTalon;
    private DigitalInput limitSwitchA;
    private DigitalInput limitSwitchB;

    private static final double kCollectPowerForward = 1;
    private static final double kCollectPowerReverse = -0.5;
    private static final double kCollectPowerStop = 0;

    public HatchCollector() {
            mRollerTalon = new WPI_TalonSRX(RobotMap.HATCH_ROLLER_TALON);
            mArticulatorTalon = new WPI_TalonSRX(RobotMap.HATCH_ARTICULATOR_TALON);
            mRollerTalon.setInverted(true);
            mArticulatorTalon.setInverted(true);
        
        limitSwitchA = new DigitalInput(RobotMap.HATCH_LIMIT_SWITCH_A);
        limitSwitchB = new DigitalInput(RobotMap.HATCH_LIMIT_SWITCH_B);

        configTalons();
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
        mArticulatorTalon.set(ControlMode.PercentOutput, value);
    }

    public void setRollerPower(double value) {
         mRollerTalon.set(ControlMode.PercentOutput, value);
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
        return mRollerTalon.getMotorOutputPercent();
    }

    public double getArticulatorPower() {
        return mArticulatorTalon.getMotorOutputPercent();
    }

    public void configTalons() {
 
        mArticulatorTalon.configNominalOutputForward(0, Constants.kTimeoutMs);
        mArticulatorTalon.configNominalOutputReverse(0, Constants.kTimeoutMs);
        mArticulatorTalon.configPeakOutputForward(1, Constants.kTimeoutMs);
        mArticulatorTalon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        mRollerTalon.configNominalOutputForward(0, Constants.kTimeoutMs);
        mRollerTalon.configNominalOutputReverse(0, Constants.kTimeoutMs);
        mRollerTalon.configPeakOutputForward(1, Constants.kTimeoutMs);
        mRollerTalon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        mArticulatorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx,
            Constants.kTimeoutMs);       
            
        mArticulatorTalon.setNeutralMode(NeutralMode.Brake);

    }

    @Override
    public void initDefaultCommand() {
    }
}