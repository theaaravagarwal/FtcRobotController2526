package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Omni Drive", group = "Drive")
public class OmniDriveOpMode extends OpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    @Override
    public void init() {
        //map the motors to their vars to control individually
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        //directions for each motor
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; //fwd/bck y
        double x = gamepad1.left_stick_x;  //l/r x
        double rx = gamepad1.right_stick_x; //rotation x
        //motor powers
        double frontLeftPower = y+x+rx;
        double backLeftPower = y-x+rx;
        double frontRightPower = y-x-rx;
        double backRightPower = y+x-rx;
        //normalize powers if any is out of [-1, 1]
        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(backLeftPower), Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));
        if (max>1.0) {
            frontLeftPower/=max;
            backLeftPower/=max;
            frontRightPower/=max;
            backRightPower/=max;
        }
        //motor powers
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
        telemetry.addData("FL", frontLeftPower);
        telemetry.addData("FR", frontRightPower);
        telemetry.addData("BL", backLeftPower);
        telemetry.addData("BR", backRightPower);
        telemetry.update();
    }
}