/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsytems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Robot.RobotID;

public class CargoCollector extends Subsystem {

    private WPI_TalonSRX mArticulatorA;
    private WPI_TalonSRX mArticulatorB;
    private Victor mRollerVictor;
    private WPI_TalonSRX mRollerTalon;

    private DigitalInput cargoLightSensor;

    private static final double kCollectPowerForward = 0.75;
    private static final double kCollectPowerReverse = -0.75;
    private static final double kCollectPowerStop = 0;

    public CargoCollector() {
        mArticulatorA = new WPI_TalonSRX(RobotMap.CARGO_TALON_A);
        mArticulatorB = new WPI_TalonSRX(RobotMap.CARGO_TALON_B);
        
       if(Robot.robotID == RobotID.FINAL){
        mRollerTalon = new WPI_TalonSRX(RobotMap.CARGO_ROLLER_TALON);  
       }else{
        mRollerVictor = new Victor(RobotMap.CARGO_ROLLER);  
       }
        
        cargoLightSensor = new DigitalInput(RobotMap.CARGO_LIGHT_SENSOR);

        configCollectorMotorOutput();
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Cargo Roller Power", getRollerPower());
        SmartDashboard.putNumber("Cargo Articulator Power", getArticulatorPower());
        SmartDashboard.putBoolean("Cargo Light Sensor Status", getLightSensor());
    }

    public void stopSubsystem() {
        if(Robot.robotID == RobotID.FINAL){
            mRollerTalon.set(0); 
        }else{
            mRollerVictor.set(0); 
        }
        
        mArticulatorA.set(ControlMode.PercentOutput, 0);
        mArticulatorB.set(ControlMode.PercentOutput, 0);
    }

    public boolean getLightSensor() {
        return cargoLightSensor.get();
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

    public void setRollerPower(double b) {
        if(Robot.robotID == RobotID.FINAL){
            mRollerTalon.set(ControlMode.PercentOutput, b); 
        }else{
            mRollerVictor.set(b); 
        }
    }

    public void configCollectorMotorOutput() {
        mArticulatorA.set(ControlMode.PercentOutput, 0);

        mArticulatorA.configNominalOutputForward(0, Constants.kTimeoutMs);
        mArticulatorA.configNominalOutputReverse(0, Constants.kTimeoutMs);
        mArticulatorA.configPeakOutputForward(1, Constants.kTimeoutMs);
        mArticulatorA.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        mArticulatorB.configNominalOutputForward(0, Constants.kTimeoutMs);
        mArticulatorB.configNominalOutputReverse(0, Constants.kTimeoutMs);
        mArticulatorB.configPeakOutputForward(1, Constants.kTimeoutMs);
        mArticulatorB.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        mArticulatorB.follow(mArticulatorA);
        mArticulatorB.setInverted(true);
        if(Robot.robotID == RobotID.FINAL){
            mRollerTalon.set(ControlMode.PercentOutput, 0);  
        }else{
            mRollerVictor.set(0);  
        }
        
        setBrakeMode();
    }

    public void setBrakeMode () {
        mArticulatorA.setNeutralMode(NeutralMode.Brake);
        mArticulatorB.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void initDefaultCommand() {

    }

    public void setArticulatorPower(double value) {
        mArticulatorA.set(ControlMode.PercentOutput, value);
        mArticulatorB.set(ControlMode.PercentOutput, value);
    }

    public double getRollerPower() {
        if(Robot.robotID == RobotID.FINAL){
            return mRollerTalon.get();  
        }else{
            return mRollerVictor.get();  
        }
    }

    public double getArticulatorPower() {
        return mArticulatorA.getMotorOutputPercent();
    }
}