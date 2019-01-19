/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsytems;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

 
public class CargoCollector extends Subsystem {
    private WPI_TalonSRX collectorA;
    private WPI_TalonSRX collectorB;

    private static final double kCollectPowerForward = 1;
    private static final double kCollectPowerReverse = -1;
    private static final double kCollectPowerStop = 0;

    public CargoCollector(){
        collectorA = new WPI_TalonSRX(RobotMap.CARGO_TALON_A);
        collectorB = new WPI_TalonSRX(RobotMap.CARGO_TALON_B);
        configCollectorMotorOutput();
    }

    public void collectForward() {
        setMotorPower(kCollectPowerForward, kCollectPowerForward);
    }

    public void collectReverse() {
        setMotorPower(kCollectPowerReverse, kCollectPowerReverse);
    }

    public void collectStop() {
        setMotorPower(kCollectPowerStop, kCollectPowerStop);
    }

    public void setMotorPower(double a, double b){
        collectorA.set(ControlMode.PercentOutput, a);
        collectorB.set(ControlMode.PercentOutput, b);
    }

    public void configCollectorMotorOutput() {
		collectorA.configNominalOutputForward(0, Constants.kTimeoutMs);
		collectorA.configNominalOutputReverse(0, Constants.kTimeoutMs);
		collectorA.configPeakOutputForward(1, Constants.kTimeoutMs);
        collectorA.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        
        collectorB.configNominalOutputForward(0, Constants.kTimeoutMs);
		collectorB.configNominalOutputReverse(0, Constants.kTimeoutMs);
		collectorB.configPeakOutputForward(1, Constants.kTimeoutMs);
		collectorB.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		
	}

    @Override
    public void initDefaultCommand() {

        
    }

    public void getMotorAOutputPercent(){
        collectorA.getMotorOutputPercent();
    }

    public void getMotorBOutputPercent(){
        collectorB.getMotorOutputPercent();
    }



}

