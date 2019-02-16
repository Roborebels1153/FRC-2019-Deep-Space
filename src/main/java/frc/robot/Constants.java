/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Constants {

    /**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2
	 * or 3. Only the first two (0,1) are visible in web-based configuration.
	 */
	public static final int kSlotIdx = 0;

	/*
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now
	 * we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/*
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 10;


	/**
	 * Use these values to tune up or tune down the drive and turn values.
	 * Call these variables in the cheesy drive methods
	 */
	public static final double k_drive_coefficient  = .8;
	public static final double k_turn_coefficient  = -.75;

	/**
	 * Use these constants for motion magic configuration
	 */
	public static final int kMotionMagicCruiseVelocity = 1800;
	public static final int kMotionMagicAcceleration = 1800;

	public static final double kLeftTicksPerInch = 100;
	public static final double kRightTicksPerInch = 100;


}
