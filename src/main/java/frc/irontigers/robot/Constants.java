// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot;

import edu.wpi.first.math.MathUtil;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveSystemVals{
    public final static int LEFT_ONE = 11;
    public final static int LEFT_TWO = 12;
    public final static int RIGHT_ONE = 13;
    public final static int RIGHT_TWO = 14;
    public final static double PULSES_TO_DISTANCE_FEET = 1 / 42 * 5.95 * ((3.31 + (2 * 0.275591)) * Math.PI / 12);
    }
 
    public static final class ArmVals{
        public final static int ARM_ROTATOR = 15;
        public final static int ARM_EXTENDER = 16;
        public final static double PULSES_TO_EXTENSION_METERS = 0;
        public final static double PULSES_TO_DEGREES = 1 / 2048 / 200 * 360;
        public final static int ARM_EXTENDER_UPPER_LIMIT = -137795; //negative velocity extends motor; upper limit has to be lower than lower limit
        public final static int ARM_EXTENDER_LOWER_LIMIT = -12000;
        // Six feet high -> -43590
    }

    public static final class ClawVals{
        public final static int CLAW = 17;
        
    }
}
