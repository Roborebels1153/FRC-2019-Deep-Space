/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public RobotMap(){

		
	}
	
	
	//Drivetrain Talons/Victors
	public static final int LEFT_MASTER = 1;
	public static final int LEFT_FOLLOWER_1 = 2;
	public static final int LEFT_FOLLOWER_2 = 3;
	public static final int RIGHT_MASTER = 4;
	public static final int RIGHT_FOLLOWER_1 = 5;
	public static final int RIGHT_FOLLOWER_2 = 6;
	

	//Cargo motor controllers
	public static final int CARGO_TALON_A = 7;
	public static final int CARGO_TALON_B = 8;

	//Hatch ground intake motor controllor
	public static final int HATCH_MOTOR = 9;
	

}
