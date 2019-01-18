package frc.robot.lib;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

public class RebelTrigger extends Button {

	private Joystick joystick;
	private int trigger;

	public RebelTrigger(Joystick joystick, int trigger) {
		this.joystick = joystick;
		this.trigger = trigger;
	}

	@Override
	public boolean get() {
		return joystick.getRawAxis(trigger) > 0.5;
	}

}
