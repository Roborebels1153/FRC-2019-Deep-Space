/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Adds a new command method option
 */
public class RebelButton extends JoystickButton {

    /**
     * Create a joystick button for triggering commands.
     *
     * @param joystick     The GenericHID object that has the button (e.g. Joystick,
     *                     KinectStick, etc)
     * @param buttonNumber The button number (see
     *                     {@link GenericHID#getRawButton(int) }
     */
    public RebelButton(GenericHID joystick, int buttonNumber) {
        super(joystick, buttonNumber);
    }

    /**
     * Starts the given command when the button is first pressed and cancels the command
     * when the button is released.
     *
     * @param command the command to start
     */
    public void runWhenActive(final Command command) {
        new ButtonScheduler() {
            private boolean m_pressedLast = get();

            @Override
            public void execute() {
                boolean pressed = get();

                if (!m_pressedLast && pressed) {
                    command.start();
                } else if (m_pressedLast && !pressed) {
                    command.cancel();
                }

                m_pressedLast = pressed;
            }
        }.start();
    }

}
