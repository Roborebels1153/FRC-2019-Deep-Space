/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsytems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class HatchCollector extends Subsystem {

    private WPI_TalonSRX intakeMotor;

    public HatchCollector() {
        intakeMotor = new WPI_TalonSRX(RobotMap.HATCH_MOTOR);
    }

    public void setMotorSpeed(double value) {
        intakeMotor.set(ControlMode.PercentOutput, value);
    }
    @Override
    public void initDefaultCommand() {

    }

}
