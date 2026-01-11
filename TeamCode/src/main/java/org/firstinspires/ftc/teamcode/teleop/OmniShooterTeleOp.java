package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "OmniShooterTeleOp")
public class OmniShooterTeleOp extends OpMode {
    private Drive drive;
    private GoBildaPinpointDriver pinpoint;
    private boolean pinpointReady = false;
    private boolean fieldCentricMode = true;
    private boolean prevToggle = false;
    private double headingOffsetRad = 0.0;
    @Override
    public void init() {
        drive = new Drive();
        drive.init(hardwareMap);
        initPinpoint();
        headingOffsetRad = getHeading();
        telemetry.addLine("Init complete.");
        telemetry.addData("Using heading from", pinpointReady?"Pinpoint":"NO IMU — RC only");
        telemetry.update();
    }
    @Override
    public void loop() {
        if (pinpointReady) pinpoint.update();
        handleModeToggle();
        double heading = 0.0;
        if (pinpointReady&&fieldCentricMode) {
            heading = getHeading()-headingOffsetRad+Math.PI; //offsetted by 180 deg cuz the pinpoint is backwards lol
            heading = normalizeRadians(heading);
        }
        driveRobot(heading);
        if (gamepad1.y) headingOffsetRad = getHeading();
        telemetry.addData("Drive Mode", fieldCentricMode?"Field-Centric":"Robot-Centric");
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.addData("Pinpoint Ready", pinpointReady);
        if (pinpointReady) {
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose = pinpoint.getPosition();
            telemetry.addData("Pinpoint X (in)", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Pinpoint Y (in)", pose.getY(DistanceUnit.INCH));
            telemetry.addData("Pinpoint Heading (deg)", Math.toDegrees(pose.getHeading(AngleUnit.RADIANS)));
        }
        telemetry.update();
    }
    private void handleModeToggle() {
        boolean togglePressed = gamepad1.right_bumper;
        if (togglePressed&&!prevToggle) fieldCentricMode = !fieldCentricMode;
        prevToggle = togglePressed;
    }
    // private void driveRobot(double headingRad) {
    //     double mult;
    //     if (gamepad1.left_bumper) mult = 0.4;
    //     else mult = 1.0+(gamepad1.left_trigger*0.3);
    //     double lat = applyDeadband(gamepad1.left_stick_x)*mult;
    //     double vert = applyDeadband(-gamepad1.left_stick_y)*mult;
    //     double rot = applyDeadband(-gamepad1.right_stick_x)*(mult*0.8);
    //     if (fieldCentricMode&&pinpointReady) drive.fieldCentric(lat, vert, rot, headingRad);
    //     else drive.robotCentric(lat, vert, rot);
    // }
    private void driveRobot(double headingRad) {
        double mult;
        if (gamepad1.left_bumper) mult = 0.4;
        else mult = 1.0+(gamepad1.left_trigger*0.3);
        double lat = applyDeadband(gamepad1.left_stick_x)*mult;
        double vert = applyDeadband(gamepad1.left_stick_y)*mult;
        double rot = applyDeadband(-gamepad1.right_stick_x)*(mult*0.8);
        if (fieldCentricMode&&pinpointReady) drive.fieldCentric(lat, vert, rot, headingRad);
        else drive.robotCentric(lat, vert, rot);
    }
    private double applyDeadband(double x) {return Math.abs(x)>0.05?x:0.0;}
    private void initPinpoint() {
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            configurePinpoint();
            pinpointReady = true;
        } catch (Exception e) {pinpointReady = false;}
    }
    private double getHeading() {
        if (pinpointReady) return pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
        return 0.0;
    }
    private double normalizeRadians(double x) {
        while (x>Math.PI) x-=2*Math.PI;
        while (x<-Math.PI) x+=2*Math.PI;
        return x;
    }
    private void configurePinpoint() {
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }
}
