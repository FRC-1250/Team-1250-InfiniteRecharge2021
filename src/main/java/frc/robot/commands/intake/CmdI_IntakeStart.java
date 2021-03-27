/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Sub_Intake;

public class CmdI_IntakeStart extends InstantCommand {

  private final Sub_Intake s_intake;

  public CmdI_IntakeStart(Sub_Intake intake) {
    s_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    s_intake.extendCylinder();
    s_intake.spinIntakeMotor(0.8);
  }
}
