/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Robot Devices----------------------------------------

    //Sub_Drivetrain
    public static final int DRV_RIGHT_FRONT = 10;
    public static final int DRV_RIGHT_BACK = 11;
    public static final int DRV_LEFT_FRONT = 12;
    public static final int DRV_LEFT_BACK = 13;
    public static final int DRV_PIGEON = 50;

    // Includes push downs
    public static final int CLM_SOL_PTO = 2;

    //Sub_Shooter
    public static final int SHOOT_FALCON_0 = 14;
    public static final int SHOOT_FALCON_1 = 15;
    public static final int SHOOT_TURRET = 16;
    public static final int SHOOT_HOOD = 17;

    //Sub_Panel
    public static final int PANEL_MOTOR = 1; // on PWM
    public static final int PANEL_SOL = 4;
    public static final I2C.Port PANEL_SENSOR_PORT = I2C.Port.kOnboard;

    //Sub_Intake
    public static final int INT_COL_MOTOR = 19;
    public static final int INT_COL_SOL = 3;

    //Sub_Hopper
    public static final int HOP_FALCON_0 = 20;
    public static final int HOP_FALCON_1 = 21;
    public static final int HOP_ELE_MOTOR = 22;
    public static final int HOP_ELE_SENS = 0;

    //Sub_Climber
    public static final int CLM_SOL_TOP = 0;
    public static final int CLM_SOL_BOTTOM = 1;
    
    //Constants--------------------------------------------

    public static final double DRV_TICKS_TO_INCH = 0.532469;

    public static final double SHOOT_TURRET_P = 0.6;
    public static final double SHOOT_TURRET_D = 0.0;

    public static final double SHOOT_TURRET_HOME = 2390;
    public static final double SHOOT_TURRET_LEFT_BOUND = 1120;
    public static final double SHOOT_TURRET_RIGHT_BOUND = 3560;

    public static final double DRV_KP_SIMPLE_STRAIT = 0.08;
    public static final double DRV_KP_SIMPLE = 0.06;

    public static final double SHOOT_HOOD_P = 1;
    public static final double SHOOT_HOOD_I = 0;
    public static final double SHOOT_HOOD_D = 0;

    public static final int HOOD_HOME_COLLISION_AMPS = 22;

    public static final double SHOOT_FLYWHEEL_P = 1;
    public static final double SHOOT_FLYWHEEL_I = 0;
    public static final double SHOOT_FLYWHEEL_D = 0;
    public static final double SHOOT_FLYWHEEL_F = 0.05115;

    public static final int LED_PWM_PORT = 9; // must be a PWM header

    // Buttons
    public static final int CLIMB_MODE = 5;
    public static final int SHOOT_MODE = 1;
    public static final int PANEL_MODE = 2;
    public static final int UNJAM_MODE = 3;

    public static final int X_BUTTON = 1;
    public static final int A_BUTTON = 2;
    public static final int B_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int LEFT_TRIGEER = 7;
    public static final int RIGHT_TRIGGER = 8;
    public static final int BACK_BUTTON = 9;
    public static final int START_BUTTON = 10;
    public static final int LEFT_STICK_CLICK = 11;
    public static final int RIGHT_STICK_CLICK = 12;
    
    // ALL DEVICES WILL HAVE TO BE PUT IN ORDER OF PHYSICAL DAISY CHAIN (FOR CAN DIAGNOSTICS)
    /* 
     * DRV_RIGHT_FRONT
     * DRV_RIGHT_BACK
     * PDP
     * DRV_LEFT_BACK
     * DRV_LEFT_FRONT
     * PCM
     * PANEL_MOTOR
     * HOP_ELE_MOTOR
     * HOP_FALCON_1
     * SHOOT_TURRET
     * HOP_FALCON_0
     * SHOOT_HOOD
     * SHOOT_FALCON_1
     * SHOOT_FALCON_0
     */
}
