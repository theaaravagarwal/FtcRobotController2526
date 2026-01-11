package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {

    private DcMotor leftFront, rightFront, leftBack, rightBack;

    private String leftFrontName = "leftFront";
    private String rightFrontName = "rightFront";
    private String leftBackName  = "leftBack";
    private String rightBackName = "rightBack";

    private double lastFL = 0.0, lastFR = 0.0, lastBL = 0.0, lastBR = 0.0;

    public Drive() {}
    public Drive(String fl, String fr, String bl, String br) {
        setMotorNames(fl, fr, bl, br);
    }

    public void setMotorNames(String fl, String fr, String bl, String br) {
        this.leftFrontName = fl;
        this.rightFrontName = fr;
        this.leftBackName  = bl;
        this.rightBackName = br;
    }

    public void init(HardwareMap hw) {
        leftFront = hw.get(DcMotor.class, leftFrontName);
        rightFront = hw.get(DcMotor.class, rightFrontName);
        leftBack = hw.get(DcMotor.class, leftBackName);
        rightBack = hw.get(DcMotor.class, rightBackName);

        // typical mecanum directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{leftFront, rightFront, leftBack, rightBack}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void robotCentric(double x, double y, double rot) {

        // // Extra torque bias toward forward/back (legal)
        // y *= 1.15;

        // // Slight traction boost
        // x *= 0.95;

        // Apply mecanum kinematics
        double lf = y + x + rot;
        double lb = y - x + rot;
        double rf = y - x - rot;
        double rb = y + x - rot;

        // Normalize properly
        double max = Math.max(
            1.0,
            Math.max(Math.abs(lf),
            Math.max(Math.abs(lb),
            Math.max(Math.abs(rf), Math.abs(rb))))
        );

        lf /= max;
        lb /= max;
        rf /= max;
        rb /= max;

        setPowers(lf, rf, lb, rb);
    }

    public void fieldCentric(double x, double y, double rot, double headingRad) {

        double c = Math.cos(headingRad);
        double s = Math.sin(headingRad);

        // field→robot transform
        double robX = x * c + y * s;
        double robY = -x * s + y * c;

        robotCentric(robX, robY, rot);
    }

    private void setPowers(double lf, double rf, double lb, double rb) {
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);

        lastFL = lf;
        lastFR = rf;
        lastBL = lb;
        lastBR = rb;
    }

    public double getLastFL() {return lastFL;}
    public double getLastFR() {return lastFR;}
    public double getLastBL() {return lastBL;}
    public double getLastBR() {return lastBR;}

    public void stop() {
        setPowers(0,0,0,0);
    }
}
