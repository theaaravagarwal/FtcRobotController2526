package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "OmniShooterTeleop")
public class OmniShooterTeleop extends OpMode {
    private RobotCentric drive;
    private ShooterSubsystem shooter;
    private LimelightAprilTag limelight;
    
    //shooter constants
    private final double GOAL_HEIGHT_METERS = 0.9845; //goal height (meters)
    private final int TARGET_TAG_ID = 20; //april tag id for the goal (20 for blue, 24 for red)

    @Override
    public void init() {
        //init drive
        //drive = new RobotCentric();
        drive = new FieldCentric();
        drive.init(hardwareMap);

        //init shooter
        //shooter = new ShooterSubsystem(`hardwareMap, "flywheel", "feederServo");
        //shooter.setFeederPositions(0.0, 1.0); //tune later for feeder we will have (maybe)

        //init limelight
        //limelight = new LimelightAprilTag("http://limelight.local:5801", TARGET_TAG_ID); //FIXME: placeholder limelight address
    }

    @Override
    public void loop() {
        //drive controls
        double strafe = -gamepad1.left_stick_x;
        double forward = gamepad1.left_stick_y;
        double rotate = -gamepad1.right_stick_x;
        drive.setDrive(strafe, forward, rotate, 0.0);

        //get dist from cam
        //double[] tagPose = limelight.getTargetPose();
        //double distance = limelight.getDistanceMeters();
        //compute required ball spd, rpm if we see tag
        //double targetRPM = 0.0;
        if (distance>0) {
            double v0 = shooter.computeRequiredBallVelocity(distance, GOAL_HEIGHT_METERS);
            targetRPM = shooter.ballSpeedToTargetRPM(v0);
            shooter.setTargetRPM(targetRPM);
        } else shooter.setTargetRPM(0.0);

        //update shooter controller and feeder
        shooter.update();

        //fire when gamepad2.a is pressed
        if (gamepad2.a&&shooter.isAtTargetRPM()) shooter.requestFeed();

        //get data for debugging
        telemetry.addData("Tag Pose", tagPose != null ? String.format("[%.2f, %.2f, %.2f]", tagPose[0], tagPose[1], tagPose[2]) : "Not seen");
        telemetry.addData("Distance (m)", distance);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Flywheel RPM", shooter.getFlywheelRPM());
        telemetry.addData("At Target RPM", shooter.isAtTargetRPM());
        telemetry.update();
    }
}