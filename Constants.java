package org.firstinspires.ftc.teamcode;

public class Constants 
{
    public static final String FR_DIRECTION = Drivebase.REVERSE;
    public static final String BR_DIRECTION = Drivebase.REVERSE;
    public static final String FL_DIRECTION = Drivebase.FORWARD;
    public static final String BL_DIRECTION = Drivebase.FORWARD;

    public static final int TICKS_PER_MOTOR_REV = 28;
    public static final double GEAR_RATIO  = 13.7;
    
    
    //Bump up if going too far
    //Bump down if not going far enough
    public static final double WHEEL_DIAMETER = 11.6; //CM
    
    //Bump down if going too far
    //Bump up if going too little
    public static final double TRACK_WIDTH = 18.45; //CM
}
