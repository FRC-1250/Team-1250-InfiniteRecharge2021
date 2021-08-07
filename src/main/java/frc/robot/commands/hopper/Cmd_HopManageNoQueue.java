/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Sub_Hopper;

public class Cmd_HopManageNoQueue extends CommandBase {
  
  private final Sub_Hopper s_hopper;
  public Cmd_HopManageNoQueue(Sub_Hopper hopper) {
    s_hopper = hopper;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    s_hopper.spinHopperMotors(0.2);
    s_hopper.spinUptakeMotor(0);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
