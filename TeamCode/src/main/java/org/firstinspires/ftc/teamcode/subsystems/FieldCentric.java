package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Simple helper for field centric mecanum drive calculations.
 *
 * <p>Usage:
 * <ol>
 *     <li>Create an instance and optionally override motor names.</li>
 *     <li>Call {@link #init(HardwareMap)} from the OpMode's {@code init()}.</li>
 *     <li>Call {@link #driveFieldCentric(double, double, double, double)} every loop.</li>
 * </ol>
 */
public class FieldCentric {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private String leftFrontName = "leftFront";
    private String rightFrontName = "rightFront";
    private String leftBackName = "leftBack";
    private String rightBackName = "rightBack";

    private double lastFL = 0.0;
    private double lastFR = 0.0;
    private double lastBL = 0.0;
    private double lastBR = 0.0;

    public FieldCentric() {
        // Default constructor uses standard motor names
    }

    public FieldCentric(String leftFrontName, String rightFrontName, String leftBackName, String rightBackName) {
        setMotorNames(leftFrontName, rightFrontName, leftBackName, rightBackName);
    }

    public void setMotorNames(String leftFrontName, String rightFrontName, String leftBackName, String rightBackName) {
        this.leftFrontName = leftFrontName;
        this.rightFrontName = rightFrontName;
        this.leftBackName = leftBackName;
        this.rightBackName = rightBackName;
    }

    public void init(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, leftFrontName);
        rightFront = hardwareMap.get(DcMotor.class, rightFrontName);
        leftBack = hardwareMap.get(DcMotor.class, leftBackName);
        rightBack = hardwareMap.get(DcMotor.class, rightBackName);

        // Adjust motor directions to match a conventional mecanum layout
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{leftFront, rightFront, leftBack, rightBack}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Convert driver field inputs into mecanum wheel powers.
     *
     * @param fieldStrafe desired field strafe (+ to the right)
     * @param fieldForward desired field forward (+ away from the driver station)
     * @param rotate desired rotational rate (+ CCW)
     * @param headingRad robot heading in radians, measured CCW from the field forward direction
     */
    public void driveFieldCentric(double fieldStrafe, double fieldForward, double rotate, double headingRad) {
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);

        // Rotate the field vector by -heading to obtain robot-centric motion commands.
        // The intermediate strafe axis here is still defined as +X being to the right.
        double robotStrafeRight = fieldStrafe * cos + fieldForward * sin;
        double robotForward = -fieldStrafe * sin + fieldForward * cos;

        // RobotCentric#setDrive expects strafe inputs where positive values move the
        // robot to the *left*. Convert the right-positive convention produced by the
        // rotation above so we drive the mecanum solver with matching semantics.
        double robotStrafe = -robotStrafeRight;

        double lf = robotForward + robotStrafe + rotate;
        double rf = robotForward - robotStrafe - rotate;
        double lb = robotForward - robotStrafe + rotate;
        double rb = robotForward + robotStrafe - rotate;

        double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));

        lf /= max;
        rf /= max;
        lb /= max;
        rb /= max;

        applyPower(leftFront, lf);  lastFL = lf;
        applyPower(rightFront, rf); lastFR = rf;
        applyPower(leftBack, lb);   lastBL = lb;
        applyPower(rightBack, rb);  lastBR = rb;
    }

    public void stop() {
        driveFieldCentric(0.0, 0.0, 0.0, 0.0);
    }

    private void applyPower(DcMotor motor, double power) {
        if (motor != null) {
            motor.setPower(power);
        }
    }

    public double getLastFL() {
        return lastFL;
    }

    public double getLastFR() {
        return lastFR;
    }

    public double getLastBL() {
        return lastBL;
    }

    public double getLastBR() {
        return lastBR;
    }
}
