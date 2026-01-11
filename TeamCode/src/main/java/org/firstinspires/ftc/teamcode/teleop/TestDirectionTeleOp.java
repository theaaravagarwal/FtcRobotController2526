package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestDirectionTeleOp")
public class TestDirectionTeleOp extends OpMode {
    
    private DcMotorEx flywheel;
    private DcMotorEx indexerLeft;
    private DcMotorEx indexerRight;
    private Servo feeder;

    // Use these to test your servo positions
    public static double READY_POS = 0.10;
    public static double PUSH_POS = 0.55;

    @Override
    public void init() {
        // Names must match your configuration on the Driver Station
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        indexerLeft = hardwareMap.get(DcMotorEx.class, "indexerLeft");
        indexerRight = hardwareMap.get(DcMotorEx.class, "indexerRight");
        feeder = hardwareMap.get(Servo.class, "feeder");

        telemetry.addLine("--- TEST MODE READY ---");
        telemetry.addLine("Hold Cross (A) -> Flywheel");
        telemetry.addLine("Hold Square (X) -> Indexer Left");
        telemetry.addLine("Hold Circle (B) -> Indexer Right");
        telemetry.addLine("Hold Triangle (Y) -> Feeder Servo");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. Test Flywheel (Gamepad 1 Cross/A)
        if (gamepad1.a) {
            flywheel.setPower(0.3); // Low power for safety
            telemetry.addData("Testing", "FLYWHEEL");
        } else {
            flywheel.setPower(0);
        }

        // 2. Test Indexer Left (Gamepad 1 Square/X)
        if (gamepad1.x) {
            indexerLeft.setPower(0.5);
            telemetry.addData("Testing", "INDEXER LEFT");
        } else {
            indexerLeft.setPower(0);
        }

        // 3. Test Indexer Right (Gamepad 1 Circle/B)
        if (gamepad1.b) {
            indexerRight.setPower(0.5);
            telemetry.addData("Testing", "INDEXER RIGHT");
        } else {
            indexerRight.setPower(0);
        }

        // 4. Test Feeder Servo (Gamepad 1 Triangle/Y)
        if (gamepad1.y) {
            feeder.setPosition(PUSH_POS);
            telemetry.addData("Testing", "FEEDER SERVO (PUSH)");
        } else {
            feeder.setPosition(READY_POS);
            telemetry.addData("Testing", "FEEDER SERVO (READY)");
        }

        telemetry.addLine("\nCheck if balls move UP and AWAY from intake.");
        telemetry.update();
    }
}