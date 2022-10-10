// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *A
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Drive System Constants
    
    public static final double MAX_SPEED = 1;
    public static final double MIN_SPEED = 0.1;

    public static final double SHOOTER_STICK_DEADZONE = -0.8;

    public static final double TOWER_TURRET_STICK_MULTIPLIER = 0.2;
    public static final double TOWER_ANGLE_STICK_SPEED = 0.1;
    public static final double TOWER_ANGLE_STICK_THRESHOLD = 0.2;

    public static final double TALON_FX_PPR = 2048;

    public static final double DRIVE_WHEEL_CIRCUMEFERENCE_M = Units.inchesToMeters(6) * Math.PI;
    public static final double DRIVE_GEAR_RATIO = 10.7 / 1.0;

    public static final double TOWER_GEAR_RATIO = 15.0 / 200.0 * 1.0 / 30.0;
    public static final double SHOOTER_GEAR_RATIO = 24.0 / 36.0;

    // LimeLight Image Processing Constants

    public static final double LIMELIGHT_HEIGHT = 70; //70 //52
    public static final double TARGET_HEIGHT = 264;
    public static final double LIMELIGHT_ANGLE = 29; //29 //51

}
