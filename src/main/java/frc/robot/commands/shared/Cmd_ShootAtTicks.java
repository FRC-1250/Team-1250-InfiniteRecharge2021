// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shared;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sub_Hopper;
import frc.robot.subsystems.Sub_Intake;
import frc.robot.subsystems.Sub_Shooter;

public class Cmd_ShootAtTicks extends CommandBase {
  private final Sub_Hopper s_hopper;
  private final Sub_Shooter s_shooter;
  private final Sub_Intake s_intake;
  int flywheelSpeedInTicks = 20000;
  double deadbandPercentage = 0.05;
  boolean shooterPrimed = false;
  Long firingStateTimestamp = 0L;
  Long delayInMS;
  double ticks;

  /**
   * 
   * @param hopper
   * @param shooter
   * @param intake
   * @param delayInMS
   * @param ticks     ticks must be less then home position at -3 ticks.
   *                  Otherwise, the method is cancelled.
   */
  public Cmd_ShootAtTicks(Sub_Hopper hopper, Sub_Shooter shooter, Sub_Intake intake, Long delayInMS, double ticks) {
    this.s_hopper = hopper;
    this.s_shooter = shooter;
    this.s_intake = intake;
    this.delayInMS = delayInMS;
    if (ticks < -3) {
      this.ticks = ticks;
    } else {
      end(true);
    }
    addRequirements(hopper, shooter, intake);
  }

  @Override
  public void initialize() {
    this.s_intake.extendCylinder();
  }

  @Override
  public void execute() {
    s_shooter.hoodGoToPos(ticks);
    s_shooter.track();
    s_shooter.setFlywheelVelocityControl(flywheelSpeedInTicks);

    shooterPrimed = s_shooter.limelightSeesTarget()
        && (s_shooter.getFlyWheelSpeed() > flywheelSpeedInTicks - (flywheelSpeedInTicks * deadbandPercentage));

    if (shooterPrimed && s_hopper.getSensor()) {
      if (System.currentTimeMillis() > firingStateTimestamp + delayInMS) {
        s_hopper.spinUptakeMotor(1);
        s_hopper.spinHopperMotors(0.6);
        firingStateTimestamp = System.currentTimeMillis();
      }
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
    // Explicitly stop the turret motor on end and interrupt
    this.s_shooter.rotateTurret(0);
    this.s_intake.retractCylinder();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
