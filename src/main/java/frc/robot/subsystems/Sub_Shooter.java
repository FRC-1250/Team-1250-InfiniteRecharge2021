/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;
import java.util.Vector;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Robot;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.CAN_DeviceFaults;
import frc.robot.utilities.CAN_Input;

public class Sub_Shooter extends SubsystemBase implements CAN_Input {
  // Speed controllers created
  WPI_TalonSRX turretTalon = new WPI_TalonSRX(Constants.SHOOT_TURRET);
  CANSparkMax hoodNeo = new CANSparkMax(Constants.SHOOT_HOOD, MotorType.kBrushless);
  WPI_TalonFX flywheelFalconLeft = new WPI_TalonFX(Constants.SHOOT_FALCON_0);
  WPI_TalonFX flywheelFalconRight = new WPI_TalonFX(Constants.SHOOT_FALCON_1);

  public CANPIDController hoodPID = new CANPIDController(hoodNeo);
  PIDController turretPIDController = new PIDController(Constants.SHOOT_TURRET_P, 0, Constants.SHOOT_TURRET_D);

  public double turretCurrentPos;

  //Bools for hardstop config
  boolean goLeft = true;
  boolean goRight = true;

  //Limelight data, used for Limelight methods
  public NetworkTable table;
  NetworkTableEntry tableTx, tableTy, tableTv;
  double tx, ty, tv;

  // Shuffleboard position and data config
  ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  NetworkTableEntry turPos = shooterTab.add("Turret Position (ticks)", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("min", Constants.SHOOT_TURRET_LEFT_BOUND, "max", Constants.SHOOT_TURRET_RIGHT_BOUND))
    .withSize(2, 1)
    .withPosition(0, 0).getEntry();
  NetworkTableEntry distFromHome = shooterTab.add("Turret Distance from Home (ticks)", 0)
    .withSize(2, 1)
    .withPosition(2, 0).getEntry();
  NetworkTableEntry hoodTemp = shooterTab.add("Hood Temp", 0)
    .withPosition(4, 0).getEntry();
  NetworkTableEntry hoodTicks = shooterTab.add("Hood Ticks", 0)
    .withPosition(4, 2).getEntry();
  NetworkTableEntry hoodCurrent = shooterTab.add("Hood Current", 0)
    .withPosition(4, 1).getEntry();
  NetworkTableEntry shootRPM = shooterTab.add("Shooter RPM", 0)
    .withWidget(BuiltInWidgets.kGraph)
    .withPosition(5, 0).getEntry();
  NetworkTableEntry distFromPort = shooterTab.add("Distance from Outer Port", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("min", 12, "max", 629))
    .withSize(3, 1)
    .withPosition(5, 2).getEntry();
  NetworkTableEntry homeFound = shooterTab.add("Home Found", "false")
    .withPosition(8, 1).getEntry();
  NetworkTableEntry turretSpeed = shooterTab.add("Turrent Percent", -1)
    .withPosition(8, 2).getEntry();
  NetworkTableEntry turretDirection = shooterTab.add("Turret Direction", "")
    .withPosition(8, 4).withSize(2,1).getEntry();
  
  public ShuffleboardTab getTab() {
    return shooterTab;
  }

  //Hood Neo control
  boolean wasHomeFound = false; //Starts robot in a "no home found" state
  double interpolatedHoodPosition; //Deprecated for now

  public Sub_Shooter() {
   //Sets the right falcon to follow the opposite of that the left falcon is doing
   //Right = follower Left = leader
   flywheelFalconRight.follow(flywheelFalconLeft);
   flywheelFalconRight.setInverted(InvertType.OpposeMaster);

   //Stops flywheels from ever back driving
   flywheelFalconLeft.configPeakOutputReverse(0);
   flywheelFalconRight.configPeakOutputReverse(0);

   //Configures PID for hood position control
   hoodPID.setP(Constants.SHOOT_HOOD_P);
   hoodPID.setI(Constants.SHOOT_HOOD_I);
   hoodPID.setD(Constants.SHOOT_HOOD_D);

   //Configures PIDF for flywheel velocity control
   //F VALUE WILL WORK FOR ALL FALCON 500s
   flywheelFalconLeft.config_kP(0, Constants.SHOOT_FLYWHEEL_P);
   flywheelFalconLeft.config_kI(0, Constants.SHOOT_FLYWHEEL_I);
   flywheelFalconLeft.config_kD(0, Constants.SHOOT_FLYWHEEL_D);
   flywheelFalconLeft.config_kF(0, Constants.SHOOT_FLYWHEEL_F);

   //Configures the turret encoder to absolute (Armabot Turret 240)
   turretTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
   turretTalon.configFeedbackNotContinuous(true, 10); // Important for absolute encoders not to jump ticks randomly

   //Ramp rate to make sure hood neo finds home cleanly
   hoodNeo.setOpenLoopRampRate(0.4);
  }
  
  //Shuffleboard value update method
  public void setShuffleboard() {
    turPos.setDouble(turretTalon.getSelectedSensorPosition());
    hoodTicks.setDouble(hoodNeo.getEncoder().getPosition());
    shootRPM.setDouble(flywheelFalconLeft.getSelectedSensorVelocity());
    distFromHome.setDouble(turretDistFromHome());
    hoodTemp.setDouble(hoodNeo.getMotorTemperature());
    distFromPort.setDouble(getPortDist());
    hoodCurrent.setDouble(hoodNEOCurrentDraw());
    homeFound.setString(Boolean.toString(wasHomeFound));
    turretSpeed.setDouble(turretTalon.getMotorOutputPercent());
  }

   // Turret
   public void rotateTurret(double speed) {
    Boolean rightBoundBreached = turretTalon.getSelectedSensorPosition() > Constants.SHOOT_TURRET_RIGHT_BOUND;
    Boolean leftBoundBreached = turretTalon.getSelectedSensorPosition() < Constants.SHOOT_TURRET_LEFT_BOUND;

    if (!rightBoundBreached && !leftBoundBreached) {
      turretTalon.set(speed);
    } else if (leftBoundBreached && speed < 0) {
      turretTalon.set(speed);
    } else if (rightBoundBreached && speed > 0) {
      turretTalon.set(speed);
    } else {
      turretTalon.set(0);
    }
  }

  @Deprecated
  public void spinTurretMotor(double speed) {
    if (goLeft && speed < 0) {
      turretTalon.set(speed);
    } else if (goRight && speed > 0) {
      turretTalon.set(speed);
    } else {
      turretTalon.set(0);
    }
  }

   //Determines based on boundries whether or not the turret is allowed to turn to the left or right
   @Deprecated
   public void hardStopConfiguration() {
    if (turretTalon.getSelectedSensorPosition() > Constants.SHOOT_TURRET_RIGHT_BOUND) {
      // turretTalon.configPeakOutputReverse(0, 10);
      goRight = false;
    } else {
      // turretTalon.configPeakOutputReverse(-1, 10);
      goRight = true;
    }
    if (turretTalon.getSelectedSensorPosition() < Constants.SHOOT_TURRET_LEFT_BOUND) {
      // turretTalon.configPeakOutputForward(0, 10);
      goLeft = false;
    } else {
      // turretTalon.configPeakOutputForward(1, 10);
      goLeft = true;
    }
  }

  //Returns flywheel speed in ticks/100ms
  public double getFlyWheelSpeed() {
    return flywheelFalconLeft.getSelectedSensorVelocity();
  }

  //Percent control of flywheel motors
  public void spinFlywheelMotors(double speed) {
    flywheelFalconLeft.set(speed);
  }

  //Percent control of hood neo
  public void spinHoodMotor(double speed) {
    hoodNeo.set(speed);
  }

  //Velocity control of flywheel
  public void setFlywheelVelocityControl(double rpm) {
    flywheelFalconLeft.set(ControlMode.Velocity, rpm);
  }

  //Reurns position of the turret in ticks
  public double getTurretTicks() {
    return turretTalon.getSelectedSensorPosition();
  }

  //Position control method of hood neo
  public void hoodGoToPos(double ticks){
    hoodPID.setReference(ticks, ControlType.kPosition);
  }

  //Returns hood postion in ticks (Raw rotations of neo)
  public double hoodPos(){
    return hoodNeo.getEncoder().getPosition();
  }

  // Void to updates the limelight values
  public void updateLimelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tableTx = table.getEntry("tx");
    tableTy = table.getEntry("ty");
    tableTv = table.getEntry("tv");
    tx = tableTx.getDouble(-1);
    ty = tableTy.getDouble(-1);
    tv = tableTv.getDouble(-1);
  }

  //Tracking method that points the turret towards the target, using a PID controller
  public void track() {
    if (limelightSeesTarget()) {
      double heading_error = -tx + 0; // in order to change the target offset (in degrees), add it here
      // How much the limelight is looking away from the target (in degrees)

      double steering_adjust = turretPIDController.calculate(heading_error);
      // Returns the next output of the PID controller (where it thinks the turret should go)
      
      double xDiff = 0 - steering_adjust;
      double xCorrect = 0.05 * xDiff;
      rotateTurret(xCorrect);
    } else {
      turretGoHome();
    }
  }

  //Simple position control to send the turret home
  public void turretGoHome() {
    if ((turretCurrentPos > Constants.SHOOT_TURRET_HOME) && (turretCurrentPos - Constants.SHOOT_TURRET_HOME > 50)) {
      // If you're to the right of the center, move left until you're within 50 ticks (turret deadband)
      rotateTurret(0.3);
      turretDirection.setString("Going left");
    } else if ((turretCurrentPos < Constants.SHOOT_TURRET_HOME) && (turretCurrentPos - Constants.SHOOT_TURRET_HOME < -50)) {
      // If you're to the left of the center, move right until you're within 50 ticks
      rotateTurret(-0.3);
      turretDirection.setString("Going right");
    } else {
      rotateTurret(0);
      turretDirection.setString("At home");
    }
  }

  //Based on tv from the limelight, returns a bool of whether a target is seen
  public boolean limelightSeesTarget() {
    return tv == 1;
  }

  //Returns a string that determines whether or not a target is seen
  public String isTarget() {
    if (limelightSeesTarget()) {
      return "SEES TARGET";
    }
    return "NO TARGET";
  }
  
  //Returns how far the turret is from it's home position
  public double turretDistFromHome() {
    return Math.abs(turretCurrentPos - Constants.SHOOT_TURRET_HOME);
  }

  //Converts ty of limelight to return distance to the outer port in inches
  public double getPortDist() {
    return 60.25/(Math.tan(Math.toRadians(26.85) + Math.toRadians(ty)));
  }

  //Manual percent control of the hood neo
  public void hoodNEOPercentControl(double percent){
    hoodNeo.set(percent);
  }

  //Returns current draw of the hood neo
  public double hoodNEOCurrentDraw(){
    return hoodNeo.getOutputCurrent();
  }

  //Resets the bool that determines whether or not home was found
  public void resetHomeHood(){
   wasHomeFound = false;
  }

  //Sets the tick count of the internal encoder of the neo to 0
  //0 = home
  public void hoodNEOResetPos(){
    hoodNeo.getEncoder().setPosition(0);
  }

  //Finds home of the hood by colliding the hood with the end of the track
  //Looks at amount of amps drawn. When amps spike, home is assumed to be at that position
  public void hoodNEOGoHome() {
    if(!wasHomeFound) {
      if (hoodNEOCurrentDraw() < Constants.HOOD_HOME_COLLISION_AMPS) {
        hoodNEOPercentControl(0.35);
      } else if (hoodNEOCurrentDraw() >= Constants.HOOD_HOME_COLLISION_AMPS) {
        hoodNEOPercentControl(0);
        hoodNEOResetPos();
        wasHomeFound = true;
      }
    }
  }

  public void hoodZoneControl(int zone) {
   // zone parameter is a number 0-3 indicating which zone robot is in (0 is green, 3 is red)
    double[] zoneTicks = {0, 0, 0, 0}; // empirically tested hood tick values
    hoodGoToPos(zoneTicks[zone]);
  }

  @Override
  public void periodic() {
    //Periodic methods that are always needed for shooter to work-----------------------------
    updateLimelight();
    turretCurrentPos = turretTalon.getSelectedSensorPosition();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    //----------------------------------------------------------------------------------------
  }

  //Adding CAN devices for diagnostic LEDs
  public Vector<CAN_DeviceFaults> input() {
    Vector<CAN_DeviceFaults> myCanDevices = new Vector<CAN_DeviceFaults>();
    myCanDevices.add(new CAN_DeviceFaults(turretTalon));
    myCanDevices.add(new CAN_DeviceFaults(hoodNeo));
    myCanDevices.add(new CAN_DeviceFaults(flywheelFalconLeft));
    myCanDevices.add(new CAN_DeviceFaults(flywheelFalconRight));
    return myCanDevices;
  }

  public boolean isHomeFound() {
    return wasHomeFound;
  }
  
}
