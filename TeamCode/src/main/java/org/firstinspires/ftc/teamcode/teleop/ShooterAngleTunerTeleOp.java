package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Simple utility OpMode for manually sweeping the shooter angle servo to find the usable range.
 *
 * Controls (gamepad1):
 *  - D-pad up/down : move servo by coarse steps (hold left bumper for fine steps)
 *  - A             : record current position as mechanical minimum
 *  - B             : record current position as mechanical maximum
 *  - X             : reset recorded values
 */
@TeleOp(name = "Shooter Angle Tuner", group = "Tuning")
public class ShooterAngleTunerTeleOp extends OpMode {
    private static final String ANGLE_SERVO_NAME = "shooterAngle";
    private static final double COARSE_STEP = 0.02;
    private static final double FINE_STEP = 0.005;

    private Servo angleServo;
    private double servoPos = 0.5;
    private double recordedMin = Double.NaN;
    private double recordedMax = Double.NaN;

    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevRecordMin = false;
    private boolean prevRecordMax = false;
    private boolean prevReset = false;

    private String status = "Angle servo not initialized";

    @Override
    public void init() {
        try {
            angleServo = hardwareMap.get(Servo.class, ANGLE_SERVO_NAME);
            servoPos = Range.clip(angleServo.getPosition(), 0.0, 1.0);
            status = "Angle servo ready";
        } catch (IllegalArgumentException e) {
            status = e.getMessage();
        }
    }

    @Override
    public void loop() {
        if (angleServo == null) {
            telemetry.addData("Status", status);
            telemetry.update();
            return;
        }

        handleAdjustments();
        angleServo.setPosition(servoPos);
        pushTelemetry();
    }

    private void handleAdjustments() {
        double step = gamepad1.left_bumper ? FINE_STEP : COARSE_STEP;
        if (gamepad1.dpad_up && !prevDpadUp) {
            servoPos = Range.clip(servoPos + step, 0.0, 1.0);
        }
        if (gamepad1.dpad_down && !prevDpadDown) {
            servoPos = Range.clip(servoPos - step, 0.0, 1.0);
        }
        if (gamepad1.a && !prevRecordMin) {
            recordedMin = servoPos;
        }
        if (gamepad1.b && !prevRecordMax) {
            recordedMax = servoPos;
        }
        if (gamepad1.x && !prevReset) {
            recordedMin = Double.NaN;
            recordedMax = Double.NaN;
        }

        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevRecordMin = gamepad1.a;
        prevRecordMax = gamepad1.b;
        prevReset = gamepad1.x;
    }

    private void pushTelemetry() {
        telemetry.addData("Status", status);
        telemetry.addData("Current Pos", "%.3f", servoPos);
        telemetry.addData("Recorded Min (A)", Double.isNaN(recordedMin) ? "n/a" : String.format("%.3f", recordedMin));
        telemetry.addData("Recorded Max (B)", Double.isNaN(recordedMax) ? "n/a" : String.format("%.3f", recordedMax));
        telemetry.addLine("Controls: D-pad up/down adjust, LB = fine, A/B record min/max, X reset");
        telemetry.update();
    }
}
