package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * OmniDrive subsystem
 * -call init(hardwareMap) from the OpMode init()
 * -call setDrive(strafe, forward, rotation) every loop to drive
 */
public class OmniDrive {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    //last applied powers
    private double lastFL = 0.0, lastFR = 0.0, lastBL = 0.0, lastBR = 0.0;
    //robot config names
    private String frontLeftName = "frontLeft";
    private String frontRightName = "frontRight";
    private String backLeftName = "backLeft";
    private String backRightName = "backRight";
    public OmniDrive() {}
    public void init(HardwareMap hw) {
        frontLeft = hw.get(DcMotor.class, frontLeftName);
        frontRight = hw.get(DcMotor.class, frontRightName);
        backLeft = hw.get(DcMotor.class, backLeftName);
        backRight = hw.get(DcMotor.class, backRightName);
        //set directions to match your wiring/gearing
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        //dont use encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //when we have no power brake
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    //drive by: x = strafe (left +, right -), y = forward (forward +, back -), rx = rotation (ccw+)
    public void setDrive(double x, double y, double rx) {
        double frontLeftPower = y+x+rx;
        double backLeftPower = y-x+rx;
        double frontRightPower = y-x-rx;
        double backRightPower = y+x-rx;
        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(backLeftPower), Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));
        if (max>1.0) {
            frontLeftPower/=max;
            backLeftPower/=max;
            frontRightPower/=max;
            backRightPower/=max;
        }
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
        lastFL = frontLeftPower;
        lastFR = frontRightPower;
        lastBL = backLeftPower;
        lastBR = backRightPower;
    }
    //getters for data
    public double getLastFL() {return lastFL;}
    public double getLastFR() {return lastFR;}
    public double getLastBL() {return lastBL;}
    public double getLastBR() {return lastBR;}
    //for custom motor names
    public void setMotorNames(String fl, String fr, String bl, String br) {
        this.frontLeftName = fl;
        this.frontRightName = fr;
        this.backLeftName = bl;
        this.backRightName = br;
    }
}
