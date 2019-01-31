/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsytems;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Victor;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

 
public class CargoCollector extends Subsystem {
    //private WPI_TalonSRX articulator;
    private Victor collector;

    private static final double kCollectPowerForward = -1;
    private static final double kCollectPowerReverse = 0.8;
    private static final double kCollectPowerStop = 0;

    public CargoCollector(){
        //articulator = new WPI_TalonSRX(RobotMap.CARGO_TALON_A);
        collector = new Victor(0);
        configCollectorMotorOutput();
    }

    public void collectForward() {
        setMotorPower(kCollectPowerForward);
    }

    public void collectReverse() {
        setMotorPower(kCollectPowerReverse);
    }

    public void collectStop() {
        setMotorPower(kCollectPowerStop);
    }

    public void setMotorPower(double b){
        collector.set(b);
    }

    public void configCollectorMotorOutput() {
        /*
		articulator.configNominalOutputForward(0, Constants.kTimeoutMs);
		articulator.configNominalOutputReverse(0, Constants.kTimeoutMs);
		articulator.configPeakOutputForward(1, Constants.kTimeoutMs);
        articulator.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		*/
	}

    @Override
    public void initDefaultCommand() {

        
    }

    public void getMotorAOutputPercent(){
        //articulator.getMotorOutputPercent();
    }


}

