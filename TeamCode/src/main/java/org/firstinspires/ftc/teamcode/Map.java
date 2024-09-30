package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Map {
    boolean y = false;
    // names motors and sets motors to type null
    public DcMotor Frontright = null;
    public DcMotor Backright = null;
    public DcMotor Backleft = null;
    public DcMotor Frontleft = null;
    public DcMotor Lift = null;
    public Servo Claw = null;

    // sets hardware map to null and names it
    HardwareMap Map = null;
    // creates runtime variable
    public ElapsedTime runtime = new ElapsedTime();

    public void init(HardwareMap hmap){
        //sets up names for configuration
        Map = hmap;

        Frontright = hmap.get(DcMotor.class,"RF");
        Backright = hmap.get(DcMotor.class,"RB");
        Backleft = hmap.get(DcMotor.class,"LB");
        Frontleft = hmap.get(DcMotor.class,"LF");
        Lift = hmap.get(DcMotor.class,"L");
        Claw = hmap.get(Servo.class,"C");

        Frontright.setDirection(DcMotor.Direction.REVERSE);
        Backright.setDirection(DcMotor.Direction.REVERSE);


        // sets the lifts zeropowerbehavior to brake
        Frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Claw.setPosition(1);

    }
    // function for driving forward
    //runs motors forward at 60% power
    public void Forward(double seconds) {
        runtime.reset();
        while (runtime.milliseconds() < (seconds * 1000)) {
            Frontleft.setPower(0.6);
            Frontright.setPower(0.6);
            Backleft.setPower(0.6);
            Backright.setPower(0.6);
        }
    }
    //function for driving backward
    //runs motors backward at 60% power
    public void Backward(double seconds){
        runtime.reset();
        while (runtime.milliseconds()<(seconds*1000)){
            Frontleft.setPower(-0.6);
            Frontright.setPower(-0.6);
            Backleft.setPower(-0.6);
            Backright.setPower(-0.6);
        }
    }

    // function for turning left
    //runs left motors backward at 60% power
    //runs right motors forward at 60% power
    public void Left(double seconds){
        runtime.reset();
        while (runtime.milliseconds()<(seconds*1000)){
            Frontleft.setPower(-0.6);
            Frontright.setPower(0.6);
            Backleft.setPower(-0.6);
            Backright.setPower(0.6);
        }
    }
    // function for turning right
    //runs right motors backward at 60% power
    //runs left motors forward at 60% power
    public void Right(double seconds){
        runtime.reset();

        while (runtime.milliseconds() < (seconds * 1000)){
            Frontleft.setPower(0.6);
            Frontright.setPower(-0.6);
            Backleft.setPower(0.6);
            Backright.setPower(-0.6);

        }
    }
    // function for strafing right
    //runs frontleft motor forward at 60% power
    //runs frontright motor backward at 60% power
    //runs backleft motor backward at 60% power
    //runs backright motor forward at 60% power
    public void RightStrafe(double seconds){
        runtime.reset();

        while (runtime.milliseconds() < (seconds * 1000)){
            Frontleft.setPower(0.6);
            Frontright.setPower(-0.6);
            Backleft.setPower(-0.6);
            Backright.setPower(0.6);

        }
    }
    // function for strafing left
    //runs frontleft motor backward at 60% power
    //runs frontright motor forward at 60% power
    //runs backleft motor forward at 60% power
    //runs backright motor backward at 60% power
    public void LeftStrafe(double seconds){
        runtime.reset();
        while(runtime.milliseconds()<(seconds*1000)){
            Frontleft.setPower(-0.6);
            Frontright.setPower(0.6);
            Backleft.setPower(0.6);
            Backright.setPower(-0.6);
        }
    }

    // function for turning off motors
    public void Off(){
        Frontright.setPower(0);
        Frontleft.setPower(0);
        Backleft.setPower(0);
        Backright.setPower(0);
        Lift.setPower(0);
    }
    public void G(double position){Claw.setPosition(position);
    }
}
