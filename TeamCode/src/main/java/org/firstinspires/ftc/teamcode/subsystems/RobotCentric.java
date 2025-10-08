package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * RobotCentric subsystem
 * -call init(hardwareMap) from the OpMode init()
 * -call setDrive(strafe, forward, rotation) every loop to drive
 */
public class RobotCentric {
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    //last applied powers
    private double lastFL = 0.0, lastFR = 0.0, lastBL = 0.0, lastBR = 0.0;
    //robot config names
    private String leftFrontName = "leftFront";
    private String rightFrontName = "rightFront";
    private String leftBackName = "leftBack";
    private String rightBackName = "rightBack";
    public RobotCentric() {}
    public void init(HardwareMap hw) {
        leftFront = hw.get(DcMotor.class, leftFrontName);
        rightFront = hw.get(DcMotor.class, rightFrontName);
        leftBack = hw.get(DcMotor.class, leftBackName);
        rightBack = hw.get(DcMotor.class, rightBackName);
        //set directions to match your wiring/gearing
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        //dont use encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //when we have no power brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    //drive by: x = strafe (left +, right -), y = forward (forward +, back -), rx = rotation (ccw+)
    public void setDrive(double x, double y, double rx) {
        double leftFrontPower = y+x+rx;
        double leftBackPower = y-x+rx;
        double rightFrontPower = y-x-rx;
        double rightBackPower = y+x-rx;
        double max = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(leftBackPower), Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))));
        if (max>1.0) {
            leftFrontPower/=max;
            leftBackPower/=max;
            rightFrontPower/=max;
            rightBackPower/=max;
        }
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
        lastFL = leftFrontPower;
        lastFR = rightFrontPower;
        lastBL = leftBackPower;
        lastBR = rightBackPower;
    }
    //getters for data
    public double getLastFL() {return lastFL;}
    public double getLastFR() {return lastFR;}
    public double getLastBL() {return lastBL;}
    public double getLastBR() {return lastBR;}
    //for custom motor names
    public void setMotorNames(String fl, String fr, String bl, String br) {
        this.leftFrontName = fl;
        this.rightFrontName = fr;
        this.leftBackName = bl;
        this.rightBackName = br;
    }
}
