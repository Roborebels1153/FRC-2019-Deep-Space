/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class RebelGyro extends SPIGyro implements PIDSource {

    private PIDSourceType pidSource;

    public RebelGyro() {
	super();
	pidSource = PIDSourceType.kDisplacement;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
	this.pidSource = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
	return pidSource;
    }

    @Override
    public double pidGet() {
	return getAngle();
    }

    @Override
    public double getAngle() {
	return super.getAngle() % 360;
    }

}