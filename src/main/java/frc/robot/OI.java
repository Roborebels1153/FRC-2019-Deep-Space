/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.Robot.RobotID;
import frc.robot.command.CargoCollectForwardCommand;
import frc.robot.command.CargoCollectReverseCommand;
import frc.robot.command.CargoCollectStopCommand;
import frc.robot.command.HatchCollectForwardCommand;
import frc.robot.command.HatchCollectReverseCommand;
import frc.robot.command.HatchCollectStopCommand;
import frc.robot.command.StopMotionCommand;
import frc.robot.commandGroups.AutomatedClimbCommand;
import frc.robot.lib.RebelTrigger;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	private static final int DRIVER_JOYSTICK = 0;
	private static final int OPERATOR_STICK = 1;

	public static final int JOYSTICK_LEFT_Y = 1;
	public static final int JOYSTICK_RIGHT_X = 4;
	public static final int JOYSTICK_RIGHT_Y = 5;
	public static final int JOYSTICK_TRIGGER_LEFT = 2;
	public static final int JOYSTICK_TRIGGER_RIGHT = 3;

	private Joystick opStick = new Joystick(OPERATOR_STICK);
	private Joystick driverStick = new Joystick(DRIVER_JOYSTICK);

	public Button drTriggerL = new RebelTrigger(driverStick, 2);
	public Button drTriggerR = new RebelTrigger(driverStick, 3);

	public Button drButtonY = new JoystickButton(driverStick, 4);
	public Button drButtonA = new JoystickButton(driverStick, 1);
	public Button drButtonB = new JoystickButton(driverStick, 2);
	public Button drButtonX = new JoystickButton(driverStick, 3);

	public Button drBumperL = new JoystickButton(driverStick, 5);
	public Button drBumperR = new JoystickButton(driverStick, 6);

	public Button opTriggerL = new RebelTrigger(opStick, 2);
	public Button opTriggerR = new RebelTrigger(opStick, 3);

	public Button opBack = new JoystickButton(opStick, 7);
	public Button opStart = new JoystickButton(opStick, 8);
	public Button opButtonY = new JoystickButton(opStick, 4);
	public Button opButtonA = new JoystickButton(opStick, 1);
	public Button opButtonB = new JoystickButton(opStick, 2);
	public Button opButtonX = new JoystickButton(opStick, 3);

	public Button opBumperL = new JoystickButton(opStick, 5);
	public Button opBumperR = new JoystickButton(opStick, 6);

	public OI() {
		
		opTriggerR.whenPressed(new CargoCollectForwardCommand());
		opTriggerR.whenReleased(new CargoCollectStopCommand());

		opTriggerL.whenPressed(new CargoCollectReverseCommand(false));
		opTriggerL.whenReleased(new CargoCollectStopCommand());

		opButtonA.whenPressed(new CargoCollectReverseCommand(true));
		opButtonA.whenReleased(new CargoCollectStopCommand());

		opBumperR.whenPressed(new HatchCollectForwardCommand());
		opBumperR.whenReleased(new HatchCollectStopCommand());

		opBumperL.whenPressed(new HatchCollectReverseCommand());
		opBumperL.whenReleased(new HatchCollectStopCommand());
		
		//opButtonB.whenPressed(new AutomatedClimbCommand());
		//opButtonB.whenReleased(new StopMotionCommand());
	}
	

	public Joystick getOpStick() {
		return opStick;
	}

	public Joystick getDriverStick() {
		return driverStick;
	}
}
