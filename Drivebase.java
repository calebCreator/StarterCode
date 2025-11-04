/*
Copyright 2025 FIRST Tech Challenge Team 15083

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.lang.Math;

public class Drivebase {
    
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public IMU imu_IMU;
    private HardwareMap hardwareMap;
    
    private double speed = 0.75;
    private boolean wait = true;
    
    public static final String FORWARD = "forward";
    public static final String REVERSE = "reverse";
    
    // TICK CONSTANTS
    public final double ticksPerMotorRev = 28;
    public final double gearRatio = 15.2/1; // Number of turns of motor to one rotation of output
    public final double ticksPerShaftRev = ticksPerMotorRev * gearRatio;
    public final double ticksPerShaftDeg = ticksPerShaftRev / 360;
    public final double ticksPerArmMotorRev = 288;
    public final double armGearRatio = 40/10; // Number of turns of motor to one rotation of output
    public final double ticksPerArmShaftRev = ticksPerArmMotorRev * armGearRatio;
    public final double ticksPerArmShaftDeg = ticksPerArmShaftRev / 360;
    
    // WHEEL CONSTANTS
    public final double pi = 3.14159;
    public double wheelDiameter = 75;
    public final double wheelCircumference = wheelDiameter*pi;
    public final double mmOfMovementForDegreeOfMotorRoation = wheelCircumference/360;
    
    // ROTATION CONSTANTS
    public final double distanceBetweenSameSideWheels = 130; //In MM
    public final double distanceBetweenSides = 160;
    public final double radius = Math.sqrt((distanceBetweenSameSideWheels*distanceBetweenSameSideWheels)+(distanceBetweenSides*distanceBetweenSides));
    public final double diameter = radius * 2;
    public final double circumference = diameter * pi;
    public final double MMPerDeg = circumference/360;
    
    public void init(HardwareMap hardwareMap) {
        /* NAMING MOTORS AND SERVOS */
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        this.hardwareMap = hardwareMap;
        
        
        /* SETTING SOME THINGS IN REVERSE */
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        
        /* IMU */
        initIMU(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        imu_IMU.resetYaw();
    }
    //Getter/Setter methods
    /*This function sets whether a motor is reversed or not
     *@param motor The DcMotor object that you want to reverse
     *@param direction A string of the direction that you want to set the motor to
     */
    public void setMotorDirection(DcMotor motor, String direction)
    {
        if(direction.equals("reverse"))
        {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else
        {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }
    
    public void initIMU(RevHubOrientationOnRobot orientation)
    {
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        imu_IMU.initialize(new IMU.Parameters(orientation));
    }
    
    public void setSpeed(double speed)
    {
        this.speed = speed;
    }
    
    public void setWheelDiameter(int wheelDiameter)
    {
        this.wheelDiameter = wheelDiameter;
    }
    
    public void setWait(boolean wait){
        this.wait = wait;
    }
    
    
    
    
    
    /* MOVEMENT FUNCTIONS */
    
    /*This function moves the robot forward
     *@param mm The distance the robot will move forward in mm (forward +, backward -)
     *@param wait Whether or not the robot will wait until it is at the expected position before continuing with the program
     */
    public void forward(int mm) throws InterruptedException
    {
        double deg = (double)mm / mmOfMovementForDegreeOfMotorRoation;
        int[] positions = {0,0,0,0};
        positions[1] = turnMotor(frontRight, deg, (double)speed);
        positions[0] = turnMotor(frontLeft, deg, (double)speed);
        positions[3] = turnMotor(backRight, deg, (double)speed);
        positions[2] = turnMotor(backLeft, deg, (double)speed);
        if(wait){
            waitForMove(false,20,positions);
        }//End of if
    }//End of function
    
    /*This function moves the robot horizontally side to side
     *@param mm The distance the robot will move left/right in mm (right +, left -)
     *@param wait Whether or not the robot will wait until it is at the expected position before continuing with the program
     */
    public void strafe(int mm) throws InterruptedException
    {
        double deg = (double)mm / mmOfMovementForDegreeOfMotorRoation;
        int[] positions = {0,0,0,0};
        positions[1] = turnMotor(frontRight, -deg, (double)speed);
        positions[0] = turnMotor(frontLeft, deg, (double)speed);
        positions[3] = turnMotor(backRight, deg, (double)speed);
        positions[2] = turnMotor(backLeft, -deg, (double)speed);
        
        if(wait)
        {
            waitForMove(false,20,positions);
        }
    }
    
    /*This function rotates the robot
     *@param deg The angle that the robot will turn in degrees (clockwise +, counterclockwise -)
     *@param wait Whether or not the robot will wait until it is at the expected position before continuing with the program
     */
    public void turn(double deg) throws InterruptedException
    {
        double correctedDeg = deg * 1.37;
        double mm = correctedDeg * MMPerDeg;
        double degToMoveMM = mm /mmOfMovementForDegreeOfMotorRoation;
        int[] positions = {0,0,0,0};
        
        positions[1] = turnMotor(frontRight, (double)degToMoveMM, (double)speed);
        positions[0] = turnMotor(frontLeft, (double)-degToMoveMM, (double)speed);
        positions[3] = turnMotor(backRight, (double)degToMoveMM, (double)speed);
        positions[2] = turnMotor(backLeft, (double)-degToMoveMM, (double)speed);
        
        if(wait)
        {
            waitForMove(false,20,positions);
        }
    }
    // STRAFE LEFT UP
    /*This function moves the robot horizontally side to side
     *@param angle The angle that the robot will strafe at (unit circle, 0 degrees is right, rotates counterclockwise)
     *@param mm The distance the robot will move in mm (forward +, backward -)
     *@param wait Whether or not the robot will wait until it is at the expected position before continuing with the program
     */
    public void diagonal(double angle, int mm) throws InterruptedException
    {
        double deg = (double)mm / mmOfMovementForDegreeOfMotorRoation;
        double xCompenent = Math.cos(Math.toRadians(angle - 45)) * deg;
        double yCompenent = Math.sin(Math.toRadians(angle - 45)) * deg;
        
        
        int[] positions = {0,0,0,0}; //Need to add in the other motor values
        positions[1] = turnMotor(frontRight, xCompenent, (double)speed);
        positions[0] = turnMotor(frontLeft, yCompenent, (double)speed);
        positions[3] = turnMotor(backRight, yCompenent, (double)speed);
        positions[2] = turnMotor(backLeft, xCompenent, (double)speed);
        
        if(wait)
        {
        waitForMove(false,20,positions);
        }
    }
    public int roundTo(int place, int value)
    {
        return Math.round(value/place)*place;
        
    }
    
    /*This function has the robot wait until all the motors are in the correct positions
     *@param strict Whether or not the program requires the motors to stop their tasks, or just be close to the target position, before moving
     *@param strictness The maximum difference in ticks that the function will say is equal
     *@param motorPositions An array that holds the positions that the motors should be at
     */
    public void waitForMove(boolean strict, int strictness, int[] motorPositions) throws InterruptedException
    {
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //The positions array will take values as [frontLeft,frontRight,backLeft,backRight]
        
        if(strict)
        {
            while(frontRight.isBusy() || frontLeft.isBusy() || backRight.isBusy() || backLeft.isBusy()/* && opModeIsActive()*/)
            {
                Thread.sleep(20);
            }
        }
        else
        {
            strictness += 20;
            //Round all the values in the array
            for(int i = 0; i < motorPositions.length; i++)
            {
                motorPositions[i] = roundTo(strictness,motorPositions[i]);
            }
            
            int[] realMotorPositions = {0,0,0,0};
            realMotorPositions[0] = roundTo(strictness,frontLeft.getCurrentPosition());
            realMotorPositions[1] = roundTo(strictness,frontRight.getCurrentPosition());
            realMotorPositions[2] = roundTo(strictness,backLeft.getCurrentPosition());
            realMotorPositions[3] = roundTo(strictness,backRight.getCurrentPosition());
            
            while(motorPositions[0] != realMotorPositions[0] || motorPositions[1] != realMotorPositions[1] || motorPositions[2] != realMotorPositions[2] || motorPositions[3] != realMotorPositions[3] /*&& opModeIsActive()*/)
            {
                realMotorPositions[0] = roundTo(strictness,frontLeft.getCurrentPosition());
                realMotorPositions[1] = roundTo(strictness,frontRight.getCurrentPosition());
                realMotorPositions[2] = roundTo(strictness,backLeft.getCurrentPosition());
                realMotorPositions[3] = roundTo(strictness,backRight.getCurrentPosition());
                /*
                telemetry.addData("Wanted0", motorPositions[0]);
                telemetry.addData("Wanted1", motorPositions[1]);
                telemetry.addData("Wanted2", motorPositions[2]);
                telemetry.addData("Wanted3", motorPositions[3]);
                
                telemetry.addData("Real0", realMotorPositions[0]);
                telemetry.addData("Real1", realMotorPositions[1]);
                telemetry.addData("Real2", realMotorPositions[2]);
                telemetry.addData("Real3", realMotorPositions[3]);
                
                telemetry.update();
                */
                Thread.sleep(20);
                //If all the motors are done moving, then break out of the loop
                if(!(frontRight.isBusy() || frontLeft.isBusy() || backRight.isBusy() || backLeft.isBusy()))
                {
                    break;
                }
                if(false /*!opModeIsActive()*/)
                {
                    break;
                }
            }//End of while
        }//End of else
    }//End of function
    
    
    /*This function moves a motor by a set amount
     *@param motor The DcMotor object that you want to move
     *@param deg The number of degrees the motor will turn
     *@param speed The speed/power level the motor will turn at
     */
    public int turnMotor(DcMotor motor, double deg, double speed)
    {
        double ticks = deg * ticksPerShaftDeg;
        int pos = motor.getCurrentPosition();
        int goTo = (int)(pos+ticks);
        motor.setTargetPosition(goTo);
        motor.setPower(speed);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return goTo;
    }
    
    
    /*This function turns the robot to a set heading using the onboard IMU
     *@param heading The direction that the robot will face
     *@param speed The power level the motors are set to
     *@param wait Whether or not the robot will wait until it is at the expected position before continuing with the program
     */
    public void compassTurn(int heading) throws InterruptedException
    {
        int botHeading = (int)Math.toDegrees(-(imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        turn(-(heading-botHeading));
    }
}
