package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "MaybeTeleOp")
public class MaybeTeleOp extends OpMode {
    
    // Subsystems
    private Drive drive;
    private ShooterSubsystem shooter;
    private GoBildaPinpointDriver pinpoint;
    
    // State Variables
    private boolean pinpointReady = false;
    private boolean fieldCentricMode = true;
    private boolean prevToggle = false;
    private double headingOffsetRad = 0.0;

    // Shooter Toggle Logic
    private boolean isFiring = false; 
    private boolean lastCircle = false;

    @Override
    public void init() {
        // 1. Initialize Drive
        drive = new Drive();
        drive.init(hardwareMap);

        // 2. Initialize Shooter
        try {
            shooter = new ShooterSubsystem(
                hardwareMap, 
                "flywheel",      
                "indexerLeft",   
                "indexerRight",  
                "feeder"         
            );
            
            // Set directions based on your specific motor mounting
            shooter.setIndexerDirections(
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE
            );
            
        } catch (Exception e) {
            telemetry.addLine("ERROR: Shooter init failed.");
        }

        // 3. Initialize Pinpoint
        initPinpoint();
        headingOffsetRad = getHeading();

        telemetry.addLine("Init complete.");
        telemetry.addLine("G1: Drive | G2: Shoot/Intake");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update Subsystems logic
        if (shooter != null) shooter.update();
        if (pinpointReady) pinpoint.update();

        // --- GAMEPAD 1: DRIVER (Movement) ---
        handleModeToggle(); // G1 Right Bumper
        
        double heading = 0.0;
        if (pinpointReady && fieldCentricMode) {
            heading = normalizeRadians(getHeading() - headingOffsetRad + Math.PI);
        }
        
        driveRobot(heading);

        // Reset Heading
        if (gamepad1.y) headingOffsetRad = getHeading();

        // --- GAMEPAD 2: OPERATOR (Shooting & Intake) ---

        // 1. INTAKE (Queue Balls) - Hold Right Trigger on Gamepad 2
        if (gamepad2.right_trigger > 0.1 && !isFiring) {
            if (shooter != null) shooter.runIntake();
        } else if (!isFiring) {
            if (shooter != null) shooter.stopIntake();
        }

        // 2. FIRE ALL (Toggle) - Press Circle (B) on Gamepad 2
        if (gamepad2.circle && !lastCircle) {
            isFiring = !isFiring; 
            if (isFiring) {
                if (shooter != null) shooter.fireAll();
            } else {
                if (shooter != null) shooter.stopEverything();
            }
        }
        lastCircle = gamepad2.circle;

        // 3. EMERGENCY STOP - Press Cross (A) on Gamepad 2
        if (gamepad2.cross) {
            isFiring = false;
            if (shooter != null) shooter.stopEverything();
        }

        // --- Telemetry Output ---
        telemetry.addData("G1 Drive Mode", fieldCentricMode ? "Field-Centric" : "Robot-Centric");
        
        if (shooter != null) {
            telemetry.addData("--- OPERATOR (G2) ---", "");
            telemetry.addData("Shooter Status", isFiring ? "FIRING SEQUENCE" : (gamepad2.right_trigger > 0.1 ? "INTAKING" : "IDLE"));
            telemetry.addData("Flywheel RPM", "%.0f", shooter.getFlywheelRPM());
            telemetry.addData("Speed Ready", shooter.isAtTarget() ? "YES" : "WAITING...");
        }

        if (pinpointReady) {
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose = pinpoint.getPosition();
            telemetry.addData("--- POSITION ---", "");
            telemetry.addData("X (in)", "%.1f", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", "%.1f", pose.getY(DistanceUnit.INCH));
        }
        
        telemetry.update();
    }

    // --- Private Helper Methods ---

    private void handleModeToggle() {
        boolean togglePressed = gamepad1.right_bumper; // Driver toggles drive mode
        if (togglePressed && !prevToggle) fieldCentricMode = !fieldCentricMode;
        prevToggle = togglePressed;
    }

    private void driveRobot(double headingRad) {
        // Driver speed controls
        double mult;
        if (gamepad1.left_bumper) mult = 0.4; // Precision Mode
        else mult = 1.0 + (gamepad1.left_trigger * 0.3); // Turbo Mode
        
        double lat = applyDeadband(gamepad1.left_stick_x) * mult;
        double vert = applyDeadband(-gamepad1.left_stick_y) * mult;
        double rot = applyDeadband(-gamepad1.right_stick_x) * (mult * 0.8);
        
        if (fieldCentricMode && pinpointReady) drive.fieldCentric(lat, vert, rot, headingRad);
        else drive.robotCentric(lat, vert, rot);
    }

    private double applyDeadband(double x) { return Math.abs(x) > 0.05 ? x : 0.0; }

    private void initPinpoint() {
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            configurePinpoint();
            pinpointReady = true;
        } catch (Exception e) {
            pinpointReady = false;
        }
    }

    private double getHeading() {
        if (pinpointReady) return pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
        return 0.0;
    }

    private double normalizeRadians(double x) {
        while (x > Math.PI) x -= 2 * Math.PI;
        while (x < -Math.PI) x += 2 * Math.PI;
        return x;
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }
}