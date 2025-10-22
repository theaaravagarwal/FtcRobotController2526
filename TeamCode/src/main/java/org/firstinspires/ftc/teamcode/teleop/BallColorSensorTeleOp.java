package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.BallColorSensor;

@TeleOp(name = "Ball Color Sensor Test", group = "Testing")
public class BallColorSensorTeleOp extends OpMode {
    private BallColorSensor colorSensor;
    private boolean ledEnabled = true;
    private boolean prevAToggle = false;
    private boolean initFailed = false;
    private String initError = "";

    @Override
    public void init() {
        colorSensor = new BallColorSensor();
        try {
            colorSensor.init(hardwareMap);
            ledEnabled = colorSensor.isLedAvailable();
            colorSensor.setLedEnabled(ledEnabled);
        } catch (IllegalArgumentException e) {
            initFailed = true;
            initError = e.getMessage();
        }
    }

    @Override
    public void loop() {
        if (initFailed || !colorSensor.isInitialized()) {
            telemetry.addLine("Color sensor not initialized.");
            if (!initError.isEmpty()) telemetry.addLine(initError);
            telemetry.update();
            return;
        }

        handleLedToggle();

        BallColorSensor.DetectedColor detected = colorSensor.update();

        telemetry.addData("Detected", detected);
        telemetry.addData("Hue (deg)", "%.1f", colorSensor.getLastHue());
        telemetry.addData("Saturation", "%.2f", colorSensor.getLastSaturation());
        telemetry.addData("Value", "%.2f", colorSensor.getLastValue());
        telemetry.addData("LED", colorSensor.isLedAvailable() ? (ledEnabled ? "ON" : "OFF") : "N/A");
        telemetry.addLine("Press A to toggle LED (if available).");
        telemetry.update();
    }

    private void handleLedToggle() {
        boolean currentA = gamepad1.a;
        if (colorSensor.isLedAvailable() && currentA && !prevAToggle) {
            ledEnabled = !ledEnabled;
            colorSensor.setLedEnabled(ledEnabled);
        }
        prevAToggle = currentA;
    }
}
