/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsytems.Drive;
import frc.robot.subsytems.HatchCollector;
import frc.robot.subsytems.LimelightVision;
import frc.robot.OI;
import frc.robot.command.StopRobotCommand;
import frc.robot.command.VisionDrive;
import frc.robot.lib.RebelRumble;
import frc.robot.subsytems.CargoCollector;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private boolean mLastLightSensorValue = false;
  private boolean mLastLimitSwitchValue = false;
  private Command autoCommand;

  private RebelRumble mDriverVibrate;
  private RebelRumble mOpVibrate;

  public static OI oi;
  public static Drive drive;
  public static CargoCollector cargoCollector;
  public static HatchCollector hatchCollector;
  public static LimelightVision vision;

  public static enum RobotID {
    PROTO, FINAL
  }

  public static final RobotID robotID = RobotID.PROTO;

  private boolean mLastToggleState = false;
  private boolean isAutoKilled = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    drive = new Drive();
    cargoCollector = new CargoCollector();
    hatchCollector = new HatchCollector();
    vision = new LimelightVision();
    oi = new OI();

    mDriverVibrate = new RebelRumble(oi.getDriverStick());
    mOpVibrate = new RebelRumble(oi.getOpStick());
    drive.calibrateGyro();
  }

  private void updateDashboard() {
    drive.updateDashboard();
    hatchCollector.updateDashboard();
    cargoCollector.updateDashboard();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    updateDashboard();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    /*
    autoCommand = new VisionDrive(1);
    autoCommand.start();
    */
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    updateDashboard();
    vision.updateLimelightData();

    if (oi.getDriverStick().getRawButton(6)) {
      autoCommand.cancel();
      isAutoKilled = true;
    }

    if (isAutoKilled) {
      driverControl();
    }
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    updateDashboard();
    driverControl();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void driverControl() {
    mDriverVibrate.loop();
    mOpVibrate.loop();

    /**
     * When both bumper buttons of the drive controllers are pressed, inverse the
     * tele-op drive control.
     */
    boolean bothPressed = oi.getDriverStick().getRawButton(5) && oi.getDriverStick().getRawButton(6);
    if (bothPressed && !mLastToggleState) {
      drive.teleOpDriveSide = (drive.teleOpDriveSide > 0 ? -1 : 1);
    }
    mLastToggleState = bothPressed;

    // tele-op driving method
    drive.createDriveSignal(true);

    cargoCollector.setArticulatorPower(0.75 * oi.getOpStick().getY());
    hatchCollector.setArticulatorPower(oi.getOpStick().getRawAxis(5));

    //rumble controllers when cargo Light Sensor detects cargo
    if (mLastLightSensorValue && !cargoCollector.getLightSensor()) {
      mDriverVibrate.rumble(RebelRumble.PATTERN_RIGHT_TO_LEFT);
      mOpVibrate.rumble(RebelRumble.PATTERN_RIGHT_TO_LEFT);
    }

    if (mLastLimitSwitchValue && !hatchCollector.getHatchLimitSwitch()) {
      mDriverVibrate.rumble(RebelRumble.PATTERN_LEFT_TO_RIGHT);
      mOpVibrate.rumble(RebelRumble.PATTERN_LEFT_TO_RIGHT);
    }

    mLastLightSensorValue = cargoCollector.getLightSensor();
    mLastLimitSwitchValue = hatchCollector.getHatchLimitSwitch();
  }

}
