package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterSubsystem {
    private final DcMotorEx flywheel;
    private final DcMotorEx indexer1, indexer2; // The 2 motors pushing ball up
    private final Servo feeder; // The Axon Servo
    
    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0.0;

    // ---      ing Constants ---
    public static double LINE_SHOT_RPM = 5100.0;
    public static double RPM_TOLERANCE = 55.0;   
    public static double INDEXER_POWER = 0.75; // Speed for moving balls up
    private final double TICKS_PER_REV;

    // --- PI Controller ---
    private double kP = 0.0012, kI = 0.0008; 
    private double integrator = 0.0, targetRPM = 0.0;

    // --- Feeder State ---
    private enum FeederState { IDLE, FLICKING, RETRACTING }
    private FeederState feederState = FeederState.IDLE;
    private double feederStateStart = 0.0;
    
    // Axon Servo Settings
    public static double SERVO_READY_POS = 0.10;
    public static double SERVO_PUSH_POS = 0.55;
    public static long FLICK_TIME_MS = 100;
    public static long RESET_TIME_MS = 150;

    // State Flags
    private boolean isShooting = false;
    private boolean isIntaking = false;

    public ShooterSubsystem(HardwareMap hw, String flywheelName, String idx1Name, String idx2Name, String feederName) {
        // Flywheel
        flywheel = hw.get(DcMotorEx.class, flywheelName);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        TICKS_PER_REV = flywheel.getMotorType().getTicksPerRev();

        // Indexers (The 2 DC motors)
        indexer1 = hw.get(DcMotorEx.class, idx1Name);
        indexer2 = hw.get(DcMotorEx.class, idx2Name);
        indexer1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo
        feeder = hw.get(Servo.class, feederName);
        if (feeder != null) feeder.setPosition(SERVO_READY_POS);
        
        lastTime = timer.seconds();
    }

    /**
     * Call this in init() to fix directions if motors are spinning wrong way.
     */
    public void setIndexerDirections(DcMotorSimple.Direction dir1, DcMotorSimple.Direction dir2) {
        indexer1.setDirection(dir1);
        indexer2.setDirection(dir2);
    }

    // --- User Commands ---

    /**
     * QUEUE MODE: Runs indexers to bring balls up, but keeps flywheel OFF.
     * Call this while holding your intake button.
     */
    public void runIntake() {
        this.isIntaking = true;
        this.isShooting = false; // Safety: Don't shoot while just intaking
        this.targetRPM = 0;      // Ensure flywheel is off
    }

    /**
     * STOP INTAKE: Call this when you release the intake button.
     */
    public void stopIntake() {
        this.isIntaking = false;
    }

    /**
     * FIRE MODE: Revs flywheel. When ready, runs indexers AND flicks servo.
     */
    public void fireAll() {
        this.isShooting = true;
        this.isIntaking = false; // Override intake
        this.targetRPM = LINE_SHOT_RPM;
    }

    /**
     * STOP EVERYTHING: Call on emergency or cancel.
     */
    public void stopEverything() {
        this.isShooting = false;
        this.isIntaking = false;
        this.targetRPM = 0;
        this.integrator = 0;
        flywheel.setPower(0);
        indexer1.setPower(0);
        indexer2.setPower(0);
    }

    // --- Main Update Loop ---

    public void update() {
        double now = timer.seconds();
        double dt = now - lastTime;
        if (dt <= 0.0) dt = 1e-3;
        lastTime = now;

        double currentRPM = getFlywheelRPM();

        // 1. Flywheel Logic
        if (targetRPM > 100) {
            // PI Controller
            double error = targetRPM - currentRPM;
            integrator += error * dt;
            if (Math.abs(integrator) > 1.0) integrator = Math.signum(integrator);
            double power = (kP * error) + (kI * integrator);
            flywheel.setPower(Math.max(0, Math.min(power, 1.0)));
        } else {
            flywheel.setPower(0);
            integrator = 0;
        }

        // 2. Indexer (DcMotor) Logic
        if (isIntaking) {
            // Just running up (Queueing)
            indexer1.setPower(INDEXER_POWER);
            indexer2.setPower(INDEXER_POWER);
        } else if (isShooting && isAtTarget()) {
            // FIRE MODE: Run indexers to feed the servo logic
            indexer1.setPower(INDEXER_POWER);
            indexer2.setPower(INDEXER_POWER);
        } else {
            // Idle
            indexer1.setPower(0);
            indexer2.setPower(0);
        }

        // 3. Servo Firing Logic
        // Only start flicking if we are in Shooting Mode AND the flywheel is ready
        if (isShooting && isAtTarget()) {
            if (feederState == FeederState.IDLE) {
                feederState = FeederState.FLICKING;
                feederStateStart = timer.milliseconds();
                if (feeder != null) feeder.setPosition(SERVO_PUSH_POS);
            }
        }

        updateFeederState();
    }

    private void updateFeederState() {
        if (feeder == null) return;
        double elapsed = timer.milliseconds() - feederStateStart;

        switch (feederState) {
            case FLICKING:
                if (elapsed >= FLICK_TIME_MS) {
                    feeder.setPosition(SERVO_READY_POS);
                    feederState = FeederState.RETRACTING;
                    feederStateStart = timer.milliseconds();
                }
                break;
            case RETRACTING:
                // If we are still holding the fire button, go back to IDLE to fire again immediately
                if (elapsed >= RESET_TIME_MS) {
                    feederState = FeederState.IDLE;
                }
                break;
            case IDLE:
                break;
        }
    }

    // --- Helpers ---
    public double getFlywheelRPM() { return (flywheel.getVelocity() * 60.0) / TICKS_PER_REV; }
    public boolean isAtTarget() { return targetRPM > 0 && Math.abs(getFlywheelRPM() - targetRPM) < RPM_TOLERANCE; }
}