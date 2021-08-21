// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Sub_Climber;
import frc.robot.subsystems.Sub_Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CmdG_PrepareClimb extends SequentialCommandGroup {

  public CmdG_PrepareClimb(Sub_Drivetrain s_drivetrain, Sub_Climber s_climb) {

  

    addCommands(
      new CmdI_ExtendBottomCylinder(s_climb),
      new CmdI_ExtendTopCylinder(s_climb)
      );
  }
}
