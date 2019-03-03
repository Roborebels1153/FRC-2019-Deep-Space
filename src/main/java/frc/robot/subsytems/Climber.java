/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsytems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
    private WPI_TalonSRX climbA;
    private WPI_TalonSRX climbB;

    public Climber() {
      climbA = new WPI_TalonSRX(12);
      climbB = new WPI_TalonSRX(13);

      configTalons();
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void configTalons() {

    climbA.configNominalOutputForward(0, Constants.kTimeoutMs);
    climbA.configNominalOutputReverse(0, Constants.kTimeoutMs);
    climbA.configPeakOutputForward(1, Constants.kTimeoutMs);
    climbA.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    climbB.configNominalOutputForward(0, Constants.kTimeoutMs);
    climbB.configNominalOutputReverse(0, Constants.kTimeoutMs);
    climbB.configPeakOutputForward(1, Constants.kTimeoutMs);
    climbB.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    climbB.follow(climbA);
    climbB.setInverted(true);
  }

  public void climb(double in) {
    climbA.set(ControlMode.PercentOutput, in);
  }
}
