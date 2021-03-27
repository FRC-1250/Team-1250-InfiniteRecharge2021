/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sub_Hopper;
import frc.robot.subsystems.Sub_Intake;

public class Cmd_UnjamHopper extends CommandBase {
  /**
   * Creates a new Cmd_UnjamIntake.
   */
  private final Sub_Hopper s_hopper;
  private final Sub_Intake s_intake;
  public Cmd_UnjamHopper(Sub_Hopper hopper, Sub_Intake intake) {
    s_hopper = hopper;
    s_intake = intake;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {
    s_hopper.spinHopperMotors(-0.2);
    s_hopper.spinUptakeMotor(-0.4);
    s_intake.extendCylinder();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    s_intake.retractCylinder();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
