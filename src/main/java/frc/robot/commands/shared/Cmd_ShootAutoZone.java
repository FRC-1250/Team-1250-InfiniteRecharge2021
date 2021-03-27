// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shared;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sub_Hopper;
import frc.robot.subsystems.Sub_Shooter;

public class Cmd_ShootAutoZone extends CommandBase {
  private final Sub_Hopper s_hopper;
  private final Sub_Shooter s_shooter;
  int flywheelSpeedInTicks = 20000;
  double deadbandPercentage = 0.05;
  boolean shooterPrimed = false;

  public Cmd_ShootAutoZone(Sub_Hopper hopper, Sub_Shooter shooter) {
    this.s_hopper = hopper;
    this.s_shooter = shooter;
    addRequirements(hopper, shooter);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    s_shooter.hoodZoneControl();
    s_shooter.track();
    s_shooter.setFlywheelVelocityControl(flywheelSpeedInTicks);

    shooterPrimed = s_shooter.limelightSeesTarget()
        && (s_shooter.getFlyWheelSpeed() > flywheelSpeedInTicks - (flywheelSpeedInTicks * deadbandPercentage));

    if(shooterPrimed && s_hopper.getSensor()) {
      s_hopper.spinUptakeMotor(1);
      s_hopper.spinHopperMotors(0.6);
    } else if (!shooterPrimed && s_hopper.getSensor()) {
      s_hopper.spinUptakeMotor(0);
      s_hopper.spinHopperMotors(0.2);
    } else if (shooterPrimed && !s_hopper.getSensor()) {
      s_hopper.spinUptakeMotor(0.4);
      s_hopper.spinHopperMotors(0.2);
    } else if (!shooterPrimed && !s_hopper.getSensor()) {
      s_hopper.spinUptakeMotor(0.4);
      s_hopper.spinHopperMotors(0.2);
    }
  }

  @Override
  public void end(boolean interrupted) {
    s_shooter.spinTurretMotor(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
