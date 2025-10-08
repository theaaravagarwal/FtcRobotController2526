package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * RobotCentric subsystem
 * -call init(hardwareMap) from the OpMode init()
 * -call setDrive(strafe, forward, rotation) every loop to drive
 */
public class FieldCentric {
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    //last applied powers
    private double lastFL = 0.0, lastFR = 0.0, lastBL = 0.0, lastBR = 0.0;
    //robot config names
    private String leftFrontName = "leftFront";
    private String rightFrontName = "rightFront";
    private String leftBackName = "leftBack";
    private String rightBackName = "rightBack";
    public FieldCentric() {}
    public void init(HardwareMap hw) {
        leftFront = hw.get(DcMotor.class, leftFrontName);
        rightFront = hw.get(DcMotor.class, rightFrontName);
        leftBack = hw.get(DcMotor.class, leftBackName);
        rightBack = hw.get(DcMotor.class, rightBackName);
        //set directions to match your wiring/gearing
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
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
    public void setDrive(double x, double y, double rx, double hdRad) {
        //rot the stick in by the - of the robot dir
        double c = Math.cos(hdRad);
        double s = Math.sin(hdRad);
        double fldX = x*c-y*s;  //rot strafe
        double fldY = x*s+y*c;  //rot forward
        //use the rotated inputs for power
        double lfPow  = fldY+fldX+rx; double lbPow   = fldY-fldX+rx; //left
        double rfPow = fldY-fldX-rx; double rbPow  = fldY+fldX-rx; //right
        double max = Math.max(Math.abs(lfPow), Math.max(Math.abs(lbPow), Math.max(Math.abs(rfPow), Math.abs(rbPow)))); //limit values to 1.0
        if (max>1.0) {lfPow/=max; lbPow/=max; rfPow/=max; rbPow/=max;}
        //apply the powers
        leftFront.setPower(lfPow);
        rightFront.setPower(rfPow);
        leftBack.setPower(lbPow);
        rightBack.setPower(rbPow);
        lastFL = lfPow;
        lastFR = rfPow;
        lastBL = lbPow;
        lastBR = rbPow;
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
