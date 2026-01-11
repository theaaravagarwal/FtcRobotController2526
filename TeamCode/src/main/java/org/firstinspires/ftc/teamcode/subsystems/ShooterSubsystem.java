package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystem {
    // Hardware
    private final DcMotorEx flywheel;
    private final DcMotorEx indexer1; // The "bottom" or first indexer
    private final DcMotorEx indexer2; // The "top" or second indexer (used during shooting)
    private final Servo feeder;       // The Axon Servo
    
    // --- Tuning Constants ---
    public static double SHOOT_POWER = 0.9;     // Full power for shooting
    public static double INTAKE_POWER = 0.75;    // Power for intaking
    
    // Servo Positions (Tune these values in dashboard or code)
    public static double SERVO_INTAKE_POS = 0.5;
    public static double SERVO_IDLE_POS = 0.1;

    public ShooterSubsystem(HardwareMap hw, String flywheelName, String idx1Name, String idx2Name, String feederName) {
        // 1. Initialize Flywheel (Open Loop / No Encoder)
        flywheel = hw.get(DcMotorEx.class, flywheelName);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // 2. Initialize Indexers (Open Loop / No Encoder)
        indexer1 = hw.get(DcMotorEx.class, idx1Name);
        indexer2 = hw.get(DcMotorEx.class, idx2Name);
        
        indexer1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexer2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Brake makes them stop instantly when you let go of the button
        indexer1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 3. Initialize Servo
        feeder = hw.get(Servo.class, feederName);
    }

    /**
     * CONFIGURATION: Call this in init() to tune directions.
     * Change FORWARD/REVERSE here until positive power moves everything the correct way.
     */
    public void configureDirections(DcMotorSimple.Direction flyDir, DcMotorSimple.Direction idx1Dir, DcMotorSimple.Direction idx2Dir) { //, Servo.Direction servoDir //if required
        flywheel.setDirection(flyDir);
        indexer1.setDirection(idx1Dir);
        indexer2.setDirection(idx2Dir);
        //if (feeder != null) feeder.setDirection(servoDir);
    }

    // --- Actions ---

    /**
     * INTAKE: Runs Indexer 1, Indexer 2, and Servo.
     * (Flywheel stays OFF).
     */
    public void runIntake() {
        indexer1.setPower(INTAKE_POWER);
        indexer2.setPower(INTAKE_POWER);
        
        if (feeder != null) feeder.setPosition(SERVO_INTAKE_POS);
        
        flywheel.setPower(0); // Ensure flywheel is off
    }

    /**
     * SHOOT: Runs Flywheel (Max Power) and ONLY Indexer 2.
     * (Indexer 1 and Servo stay idle).
     */
    public void shoot() {
        flywheel.setPower(SHOOT_POWER);
        
        indexer1.setPower(0);            // Stop bottom motor
        indexer2.setPower(INTAKE_POWER); // Run top motor to feed flywheel
        
        if (feeder != null) feeder.setPosition(SERVO_IDLE_POS);
    }

    /**
     * STOP: Kills all power.
     */
    public void stop() {
        flywheel.setPower(0);
        indexer1.setPower(0);
        indexer2.setPower(0);
        if (feeder != null) feeder.setPosition(SERVO_IDLE_POS);
    }
}