/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.auto_actions.Cmd_PlayAutoRecord;
import frc.robot.commands.auto_actions.Cmd_StartAutoRecord;
import frc.robot.commands.hopper.Cmd_HopperManagement;
import frc.robot.commands.shooter.Cmd_ShooterIdle;
import frc.robot.subsystems.Sub_Climber;
import frc.robot.subsystems.Sub_Drivetrain;
import frc.robot.subsystems.Sub_Hopper;
import frc.robot.subsystems.Sub_Intake;
import frc.robot.subsystems.Sub_Recorder;
import frc.robot.subsystems.Sub_Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Subsystems
  public static final Sub_Drivetrain s_drivetrain = new Sub_Drivetrain();
  public static final Sub_Intake s_intake = new Sub_Intake();
  public static final Sub_Shooter s_shooter = new Sub_Shooter();
  public static final Sub_Climber s_climb = new Sub_Climber();
  public static final Sub_Hopper s_hopper = new Sub_Hopper();
  public static final Sub_Recorder s_recorder = new Sub_Recorder();

   // Driver joystick and buttons for logitech gamepad.
   Joystick driver = new Joystick(0);
   JoystickButton driverXButton = new JoystickButton(driver, Constants.X_BUTTON);
   JoystickButton driverYButton = new JoystickButton(driver, Constants.Y_BUTTON);
   JoystickButton driverAButton = new JoystickButton(driver, Constants.A_BUTTON);
   JoystickButton driverBButton = new JoystickButton(driver, Constants.B_BUTTON);
   JoystickButton driverBackButton = new JoystickButton(driver, Constants.BACK_BUTTON);
   JoystickButton driverStartButton = new JoystickButton(driver, Constants.START_BUTTON);
   JoystickButton driverLeftBumper = new JoystickButton(driver, Constants.LEFT_BUMPER);
   JoystickButton driverRightBumper = new JoystickButton(driver, Constants.RIGHT_BUMPER);
   JoystickButton driverLeftTrigger = new JoystickButton(driver, Constants.LEFT_TRIGEER);
   JoystickButton driverRightTrigger = new JoystickButton(driver, Constants.RIGHT_TRIGGER);
   JoystickButton driverLeftStickClick = new JoystickButton(driver, Constants.LEFT_STICK_CLICK);
   JoystickButton driverRightStickClick = new JoystickButton(driver, Constants.RIGHT_STICK_CLICK);
 
   // Operator joystick and buttons
   Joystick operator = new Joystick(1);
   JoystickButton operatorClimbButton = new JoystickButton(operator, Constants.CLIMB_MODE);
   JoystickButton operatorShootButton = new JoystickButton(operator, Constants.SHOOT_MODE);
   JoystickButton operatorPanelButton = new JoystickButton(operator, Constants.PANEL_MODE);
   JoystickButton operatorUnjamButton = new JoystickButton(operator, Constants.UNJAM_MODE);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    s_hopper.setDefaultCommand(new Cmd_HopperManagement(s_hopper));
    s_shooter.setDefaultCommand(new Cmd_ShooterIdle(s_shooter, false, false));

    ShuffleboardTab recorderTab = Shuffleboard.getTab("Recorder");
    s_recorder.addFileChooserOptions();
    recorderTab.add("Start record", new Cmd_StartAutoRecord(s_recorder, s_drivetrain)).withPosition(2, 0).withSize(2, 1);
    recorderTab.add("Playback record", new Cmd_PlayAutoRecord(s_recorder, s_drivetrain)).withPosition(2, 1).withSize(2, 1);
  }

  private void configureButtonBindings() {
    /*
    * Buttons
    * Intake on
    * Intake off
    * Shoot (Flywheels + Hopper)
    * Record drive on
    * Record off
    * 
    */

    /* Commands
     * Hopper idle
     * Shooter idle
     */
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
