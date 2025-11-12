package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Utility OpMode for tuning the shooter flywheel RPM mapping and controller gains.
 *
 * Controls (gamepad1):
 *  - Right trigger : enable flywheel and hold current target RPM
 *  - D-pad up/down : adjust setpoint (coarse, +/- step)
 *  - Left bumper   : hold to make adjustments fine-grained
 *  - Y             : zero target RPM immediately
 *  - A             : log the current reading to telemetry log
 */
@TeleOp(name = "Shooter Flywheel Tuner", group = "Tuning")
public class ShooterFlywheelTunerTeleOp extends OpMode {
    private static final String FLYWHEEL_NAME = "flywheel";

    private static final double BASE_RPM_STEP = 50.0;
    private static final double FINE_RPM_STEP = 10.0;
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 5000.0;

    private ShooterSubsystem shooter;
    private boolean shooterReady = false;
    private String shooterStatus = "Shooter not initialized";

    private double targetRpm = 2500.0;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevLogButton = false;
    private boolean prevZeroButton = false;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastUpdateTime = 0.0;

    @Override
    public void init() {
        try {
            shooter = new ShooterSubsystem(hardwareMap, FLYWHEEL_NAME, null);
            shooterReady = true;
            shooterStatus = "Shooter ready";
        } catch (IllegalArgumentException e) {
            shooterReady = false;
            shooterStatus = e.getMessage();
        }
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double now = loopTimer.seconds();
        double dt = now - lastUpdateTime;
        lastUpdateTime = now;

        handleAdjustments();
        if (!shooterReady) {
            telemetry.addData("Status", shooterStatus);
            telemetry.update();
            return;
        }

        boolean enabled = gamepad1.right_trigger > 0.05;
        double commandedRpm = enabled ? targetRpm : 0.0;
        shooter.setTargetRPM(commandedRpm);
        shooter.update();

        double measured = shooter.getFlywheelRPM();
        double error = commandedRpm - measured;
        pushTelemetry(enabled, commandedRpm, measured, error, dt);
        handleLogging(enabled, commandedRpm, measured);
    }

    private void handleAdjustments() {
        double step = gamepad1.left_bumper ? FINE_RPM_STEP : BASE_RPM_STEP;
        if (gamepad1.dpad_up && !prevDpadUp) {
            targetRpm = Range.clip(targetRpm + step, MIN_RPM, MAX_RPM);
        }
        if (gamepad1.dpad_down && !prevDpadDown) {
            targetRpm = Range.clip(targetRpm - step, MIN_RPM, MAX_RPM);
        }
        if (gamepad1.y && !prevZeroButton) {
            targetRpm = 0.0;
        }

        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevZeroButton = gamepad1.y;
    }

    private void handleLogging(boolean enabled, double commanded, double measured) {
        if (gamepad1.a && !prevLogButton) {
            telemetry.log().add("Enabled:%s Target:%.1f RPM  Measured:%.1f RPM", enabled, commanded, measured);
        }
        prevLogButton = gamepad1.a;
    }

    private void pushTelemetry(boolean enabled, double commanded, double measured, double error, double dt) {
        telemetry.addData("Status", shooterStatus);
        telemetry.addData("Flywheel Enabled (RT)", enabled);
        telemetry.addData("Target RPM", "%.1f", targetRpm);
        telemetry.addData("Commanded RPM", "%.1f", commanded);
        telemetry.addData("Measured RPM", "%.1f", measured);
        telemetry.addData("Error", "%.1f", error);
        telemetry.addData("Loop dt (s)", "%.3f", dt);
        telemetry.addLine("Controls: D-pad +/- RPM (LB=fine), RT enable, Y zero target, A log sample");
        telemetry.update();
    }
}
