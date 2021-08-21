// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Sub_Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CmdI_DisengagePTO extends InstantCommand {

  private final Sub_Drivetrain s_drive;

  public CmdI_DisengagePTO(Sub_Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    s_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_drive.disengagePTO();
  }
}
