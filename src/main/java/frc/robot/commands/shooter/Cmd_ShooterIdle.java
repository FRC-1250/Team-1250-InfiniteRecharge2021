// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sub_Shooter;

public class Cmd_ShooterIdle extends CommandBase {
  private final Sub_Shooter s_shooter;
  private boolean stayHome = false;
  private boolean keepWarm = false;
  private double keepWarmRPM = 2000;

  public Cmd_ShooterIdle(Sub_Shooter shooter, boolean stayHome, boolean keepWarm) {
    this.s_shooter = shooter;
    this.stayHome = stayHome;
    this.keepWarm = keepWarm;
    addRequirements(s_shooter);
  }

  @Override
  public void execute() {
    s_shooter.turretGoHome();
    if (keepWarm) {
      s_shooter.setFlywheelVelocityControl(keepWarmRPM);
    } else {
      s_shooter.setFlywheelVelocityControl(0);
    }

    if (s_shooter.isHomeFound()) {
      if (stayHome) {
        s_shooter.hoodGoToPos(-3);
      }
    } else {
      s_shooter.hoodNEOGoHome();
    }
  }
}
