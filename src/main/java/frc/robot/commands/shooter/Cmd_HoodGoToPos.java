/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sub_Shooter;

public class Cmd_HoodGoToPos extends CommandBase {
  private final Sub_Shooter s_shooter;

  double ticksToGo;

  public Cmd_HoodGoToPos(Sub_Shooter shooter, double ticks) {
    s_shooter = shooter;
    addRequirements(shooter);
    ticksToGo = ticks;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_shooter.hoodGoToPos(ticksToGo);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    double absPos = Math.abs(s_shooter.hoodPos());
    double absSetPoint = Math.abs(ticksToGo);

    return ((absPos - absSetPoint) <= 1);
  }
}
