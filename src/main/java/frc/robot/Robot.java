/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsytems.Drive;
import frc.robot.subsytems.HatchCollector;
import frc.robot.subsytems.LimelightVision;
import frc.robot.OI;
import frc.robot.commandGroups.LimelightCargoShipCommandGroup;
import frc.robot.lib.RebelRumble;
import frc.robot.subsytems.CargoCollector;
import frc.robot.subsytems.Climber;
import frc.robot.subsytems.LimelightVision.Target;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static enum SandstormMode {
    RUN_AUTO, DRIVER_CONTROL
  }

  private static enum SandstormStrategy {
    CARGO_SHIP, ROCKET
  }

  private final SendableChooser<SandstormMode> mSandstormMode = new SendableChooser<>();
  private final SendableChooser<Integer> mLevelChooser = new SendableChooser<>();
  private final SendableChooser<Integer> mPositionChooser = new SendableChooser<>();
  private final SendableChooser<SandstormStrategy> mSandstormStrategyChooser = new SendableChooser<>();

  private boolean mLastLightSensorValue = false;
  private boolean mLastLimitSwitchValue = false;
  private Command autoCommand;

  public Target target;

  private RebelRumble mDriverVibrate;
  private RebelRumble mOpVibrate;

  public static OI oi;
  public static Drive drive;
  public static CargoCollector cargoCollector;
  public static HatchCollector hatchCollector;
  public static Climber climber;
  public static LimelightVision vision;

  public static enum RobotID {
    PROTO, FINAL
  }

  public static final RobotID robotID = RobotID.FINAL;

  private boolean mLastToggleState = false;
  private boolean isAutoKilled = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    mSandstormMode.setDefaultOption("Run Auto", SandstormMode.RUN_AUTO);
    mSandstormMode.addOption("Driver Control", SandstormMode.DRIVER_CONTROL);
    SmartDashboard.putData("Sandstorm Mode", mSandstormMode);

    mLevelChooser.setDefaultOption("Level 1", LimelightCargoShipCommandGroup.L1);
    mLevelChooser.addOption("Level 2", LimelightCargoShipCommandGroup.L2);
    SmartDashboard.putData("Level", mLevelChooser);

    mPositionChooser.setDefaultOption("Right", LimelightCargoShipCommandGroup.RIGHT);
    mPositionChooser.addOption("Center", LimelightCargoShipCommandGroup.CENTER);
    mPositionChooser.addOption("Left", LimelightCargoShipCommandGroup.LEFT);
    SmartDashboard.putData("Position", mPositionChooser);

    mSandstormStrategyChooser.setDefaultOption("Rocket", SandstormStrategy.ROCKET);
    mSandstormStrategyChooser.addOption("Cargo Ship", SandstormStrategy.CARGO_SHIP);
    SmartDashboard.putData("Strategy", mSandstormStrategyChooser);

    drive = new Drive();
    cargoCollector = new CargoCollector();
    hatchCollector = new HatchCollector();
    climber = new Climber();
    vision = new LimelightVision();
    oi = new OI();

    vision.setPipeline(0);
    mDriverVibrate = new RebelRumble(oi.getDriverStick());
    mOpVibrate = new RebelRumble(oi.getOpStick());
    drive.calibrateGyro();
  }

  private void updateDashboard() {
    drive.updateDashboard();
    hatchCollector.updateDashboard();
    cargoCollector.updateDashboard();
    climber.updateDashboard();
    SmartDashboard.putNumber("Limelight Area", vision.getTargetArea());
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

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
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
    SandstormMode mode = mSandstormMode.getSelected();
    int level = mLevelChooser.getSelected();
    int position = mPositionChooser.getSelected();
    SandstormStrategy strategy = mSandstormStrategyChooser.getSelected();

    if (mode == SandstormMode.RUN_AUTO) {
      if (strategy == SandstormStrategy.CARGO_SHIP) {
        autoCommand = new LimelightCargoShipCommandGroup(level, position);
      } else if (strategy == SandstormStrategy.ROCKET) {

      }
      //autoCommand = new AutomatedClimbCommand();
      autoCommand.start();
    }
    //autoCommand = new SideHatchAuto();
    //autoCommand.start();
    drive.resetGyro();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    updateDashboard();
    vision.updateLimelightData();

    SandstormMode mode = mSandstormMode.getSelected();

    if (oi.getDriverStick().getRawButton(6)) {
      if (autoCommand != null) {
        autoCommand.cancel();
      }
      vision.setPipeline(0);
      isAutoKilled = true;
    }

    if (isAutoKilled || mode == SandstormMode.DRIVER_CONTROL) {
      vision.setPipeline(0);
      driverControl();
    }
  }

  @Override
  public void teleopInit() {
    if(autoCommand != null) autoCommand.cancel();
    vision.setPipeline(0);
    //vision.turnOffLight();
    drive.resetGyro();
    cargoCollector.resetEncoders();
    climber.resetEncoders();
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
    boolean bothPressed = oi.getDriverStick().getRawButton(6);
    if (bothPressed && !mLastToggleState) {
      drive.teleOpDriveSide = (drive.teleOpDriveSide > 0 ? -1 : 1);
    }
    mLastToggleState = bothPressed;

    
    if (oi.getDriverStick().getRawButton(2) && !(vision.getTargetArea() > 20)){
      vision.updateLimelightData();
      vision.setPipeline(1);
      //vision.turnOnLight();
      System.out.println("Starting limleight vision");
      target = Robot.vision.getTargetValues();
      Robot.drive.cheesyDriveWithoutJoysticks(-1*drive.teleOpDriveSide * Constants.k_drive_coefficient * 
       Robot.oi.getDriverStick().getRawAxis(OI.JOYSTICK_LEFT_Y), Robot.vision.getHorizontalAlignOutput() * .8);
    } else if (oi.getOpStick().getRawButton(2)) {
      hatchCollector.setArticulatorPower(-1);
      drive.tankDriveNoJoystick(0.125, 0.125);
    } else {
      vision.setPipeline(0);
      hatchCollector.setArticulatorPower(-1 * oi.getOpStick().getRawAxis(5));
      drive.createHybridDriveSignal(true);
    }
    
    if (Math.abs(oi.getOpStick().getY()) > 0.1) {
      cargoCollector.setArticulatorPower(-0.75 * oi.getOpStick().getY());
    } else {
      cargoCollector.setArticulatorPower(0);
    }

    //rumble controllers when cargo Light Sensor detects cargo
    if (mLastLightSensorValue && !cargoCollector.getLightSensor()) {
      mDriverVibrate.rumble(RebelRumble.PATTERN_RIGHT_TO_LEFT);
      mOpVibrate.rumble(RebelRumble.PATTERN_RIGHT_TO_LEFT);
    }
  

    if (mLastLimitSwitchValue && !hatchCollector.getHatchLimitSwitchA()) {
      mDriverVibrate.rumble(RebelRumble.PATTERN_LEFT_TO_RIGHT);
      mOpVibrate.rumble(RebelRumble.PATTERN_LEFT_TO_RIGHT);
    }

      mLastLightSensorValue = cargoCollector.getLightSensor();

    mLastLimitSwitchValue = hatchCollector.getHatchLimitSwitchA();

    if (oi.getOpStick().getRawButtonPressed(4)) {
      climber.climb(1, 1);
    } else if (oi.getOpStick().getRawButtonPressed(3)) {
      climber.climb(-1, -1);
    } else if (oi.getOpStick().getRawButtonReleased(4) || oi.getOpStick().getRawButtonReleased(3)) {
      climber.climb(0, 0);
    }
  }

}
