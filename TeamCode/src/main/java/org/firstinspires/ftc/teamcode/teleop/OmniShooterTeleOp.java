package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.*;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "OmniShooterTeleOp")
public class OmniShooterTeleOp extends OpMode {

    // Subsystems
    private FieldCentric drive;
    private ShooterSubsystem shooter;
    private LimelightAprilTag limelight;
    private IMU imu;

    // Shooter constants
    private static final double GOAL_HEIGHT_METERS = 0.9845; // goal height (meters)
    private static final int TARGET_TAG_ID = 20; // april tag id for the goal (20 for blue, 24 for red)
    private double hdoffset = 0; // heading offset for field centric drive

    @Override
    public void init() {
        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new com.qualcomm.hardware.rev.RevHubOrientationOnRobot(
                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(params);

        // Initialize drive system
        drive = new FieldCentric();
        drive.init(hardwareMap);

        // Initialize shooter (uncomment once you have your shooter subsystem set up)
        // shooter = new ShooterSubsystem(hardwareMap, "flywheel", "feederServo");
        // shooter.setFeederPositions(0.0, 1.0);

        // Initialize limelight (uncomment once ready)
        // limelight = new LimelightAprilTag("http://limelight.local:5801", TARGET_TAG_ID);
    }

    @Override
    public void loop() {
        // Get drive control inputs
        double strafe = gamepad1.left_stick_x; //right is +
        double forward = -gamepad1.left_stick_y; //up is +
        double rotate = gamepad1.right_stick_x; //ccw is +

        // Reset heading offset when Y is pressed
        if (gamepad1.y) {
            hdoffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        // Compute heading relative to offset
        double rawhd = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double hdrad = rawhd - hdoffset;

        // Drive the robot
        drive.setDrive(strafe, forward, rotate, hdrad);

        // Shooter logic (uncomment when ready)
        /*
        double[] tagPose = limelight.getTargetPose();
        double distance = limelight.getDistanceMeters();
        double targetRPM = 0.0;

        if (distance > 0) {
            double v0 = shooter.computeRequiredBallVelocity(distance, GOAL_HEIGHT_METERS);
            targetRPM = shooter.ballSpeedToTargetRPM(v0);
            shooter.setTargetRPM(targetRPM);
        } else {
            shooter.setTargetRPM(0.0);
        }

        shooter.update();

        if (gamepad2.a && shooter.isAtTargetRPM()) {
            shooter.requestFeed();
        }

        telemetry.addData("Tag Pose", tagPose != null ? String.format("[%.2f, %.2f, %.2f]", tagPose[0], tagPose[1], tagPose[2]) : "Not seen");
        telemetry.addData("Distance (m)", distance);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Flywheel RPM", shooter.getFlywheelRPM());
        telemetry.addData("At Target RPM", shooter.isAtTargetRPM());
        telemetry.update();
        */
    }
}
