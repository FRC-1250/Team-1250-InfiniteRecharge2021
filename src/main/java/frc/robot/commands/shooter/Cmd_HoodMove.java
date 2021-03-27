/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Sub_Shooter;

public class Cmd_HoodMove extends CommandBase {
  /**
   * Creates a new Cmd_HoodMove.
   */
  private final Sub_Shooter s_shooter;
  Joystick Gamepad;
  public Cmd_HoodMove(Sub_Shooter shooter, Joystick Gamepad) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_shooter = shooter;
    this.Gamepad = Gamepad;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_shooter.spinHoodMotor(0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_shooter.spinHoodMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
