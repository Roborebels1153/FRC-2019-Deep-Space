/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsytems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
    private WPI_TalonSRX climbA;
    private WPI_TalonSRX climbB;

    private DigitalInput lightSensorA;
    private DigitalInput lightSensorB;


    public Climber() {
      climbA = new WPI_TalonSRX(12);
      climbB = new WPI_TalonSRX(13);

      lightSensorA = new DigitalInput(4);
      lightSensorB = new DigitalInput(5);

      configTalons();
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Climb A Encoder Value", getClimbAEncoder());
    SmartDashboard.putNumber("Climb B Encoder Value", getClimbBEncoder());

    SmartDashboard.putBoolean("Climb Light Sensor A", getClimbLightSensorA());
    SmartDashboard.putBoolean("Climb Light Sensor B", getClimbLightSensorB());
  }

  public void configTalons() {

    climbA.configNominalOutputForward(0, Constants.kTimeoutMs);
    climbA.configNominalOutputReverse(0, Constants.kTimeoutMs);
    climbA.configPeakOutputForward(1, Constants.kTimeoutMs);
    climbA.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    climbA.setNeutralMode(NeutralMode.Brake);

    climbB.configNominalOutputForward(0, Constants.kTimeoutMs);
    climbB.configNominalOutputReverse(0, Constants.kTimeoutMs);
    climbB.configPeakOutputForward(1, Constants.kTimeoutMs);
    climbB.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    climbB.setNeutralMode(NeutralMode.Brake);

    climbB.setInverted(true);
  }

  public void climb(double inA, double inB) {
    climbA.set(ControlMode.PercentOutput, inA);
    climbB.set(ControlMode.PercentOutput, inB);
  }

  public void climbB(double in) {
    climbB.set(ControlMode.PercentOutput, in);

  }

  public void climbA(double in) {
    climbA.set(ControlMode.PercentOutput, in);

  }
  public void autoClimbDownA(){
    if (getClimbLightSensorA()){
      climbA(0);
    } else {
      climbA(-0.95);
    }
  }
  public void autoClimbUpA() {
    if (getClimbAEncoder() > -300){
      climbA(0);
    }
    else {
      climbA(0.95);
    }
  }
  public void autoClimbDownB(){
    if (getClimbLightSensorB()){
      climbB(0);
    } else {
      climbB(-1);
    }
  }
  public void autoClimbUpB(){
    if (getClimbBEncoder() > -300){
      climbB(0);
    }
    else {
      climbB(1);
    }
  }

  public void climbWithLimitSwitch(double inA, double inB){  
    if (getClimbLightSensorA() == false && getClimbLightSensorB() == false) {
      climb(inA, inB);
    } else {
      climb(0,0);
    }
  }

  public void configTalonFeedback() {
    climbA.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx,
    Constants.kTimeoutMs);
    climbB.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx,
    Constants.kTimeoutMs);
}


public boolean getClimbLightSensorA() {
  return lightSensorA.get();
}

public boolean getClimbLightSensorB() {
  return lightSensorB.get();
}

public double getClimbAEncoder() {
    return climbA.getSelectedSensorPosition();
}

public double getClimbBEncoder() {
    return climbB.getSelectedSensorPosition();
}

public void resetEncoders() {
    climbA.setSelectedSensorPosition(0);
    climbB.setSelectedSensorPosition(0);

}
}
