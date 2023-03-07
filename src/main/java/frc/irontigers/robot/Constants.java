// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveVals {
    public final static int LEFT_ONE = 11;
    public final static int LEFT_TWO = 12;
    public final static int RIGHT_ONE = 13;
    public final static int RIGHT_TWO = 14;
    public final static double ROTATIONS_TO_METERS = (Units.inchesToMeters(2.901) * Math.PI) / 5.95;

    // sysid analysis output
    public final static double S = 0.34919;
    public final static double V = 3.3493;
    public final static double A = 0.40186;

    // Measurment delay = 68.703, Max control effort = 5.06 V
    public final static double LEFT_P = 0.22903;
    public final static double LEFT_I = 0.0;
    public final static double LEFT_D = 0.0;
    // Measurment delay = 68.703, Max control effort = 5.06 V
    public final static double RIGHT_P = 0.27403;
    public final static double RIGHT_I = 0.0;
    public final static double RIGHT_D = 0.0;

    public final static double TRACK_WIDTH = 0.5579;
    }
 
    public static final class ArmVals{
        public final static int ARM_ROTATOR_MASTER = 15;
        public final static int ARM_EXTENDER = 16;
        public final static int ARM_ROTATOR_SUB = 17;
        public final static double PULSES_TO_EXTENSION_METERS = 0;
        public final static double PULSES_TO_DEGREES = 1 / 2048.0 / 200 * 360;
        public final static int ARM_EXTENDER_UPPER_LIMIT = -137795; //negative velocity extends motor; upper limit has to be lower than lower limit
        public final static int ARM_EXTENDER_LOWER_LIMIT = -12000;
        public final static double EXTENDER_CONVERSION_FACTOR = 20.9375 / 132067.0;
        // Six feet high -> -57509
        // public final static double ROTATOR_CONVERSION_FACTOR = 
    }

    public static final class ClawVals{
        // public final static int CLAW = 17;
        
    }
}
