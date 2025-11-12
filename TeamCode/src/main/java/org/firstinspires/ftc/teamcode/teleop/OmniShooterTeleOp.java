//controller 1 = driver (movement)
//controller 2 = operator (attachment, unused)
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.BallColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentric;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "OmniShooterTeleOp")
public class OmniShooterTeleOp extends OpMode {
    private FieldCentric drive;
    private IMU imu;
    private boolean imuReady = false;
    private boolean fieldCentricMode = true; //start on fieldcentric
    private boolean prevToggle = false;
    private double headingOffsetRad = 0.0;
    @Override
    public void init() {
        drive = new FieldCentric();
        drive.init(hardwareMap);
        initImu();
        headingOffsetRad = imuReady?getYawRadians():0.0;
        telemetry.addLine("Init complete. Field-centric: ON");
        telemetry.update();
    }
    @Override
    public void loop() {
        handleModeToggle();
        double heading = 0.0;
        if (imuReady && fieldCentricMode) {
            heading = getYawRadians() - headingOffsetRad;
            heading = normalizeRadians(heading);
        }
        driveRobot(heading);
        telemetry.addData("Mode", fieldCentricMode ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.update();
    }
    private void handleModeToggle() {
        boolean togglePressed = gamepad1.right_bumper;
        if (togglePressed&&!prevToggle) fieldCentricMode = !fieldCentricMode;
        prevToggle = togglePressed;
    }
    private void driveRobot(double heading) {
        double strafe = applyDeadband(gamepad1.left_stick_x);
        double forward = applyDeadband(-gamepad1.left_stick_y);
        double rotate = applyDeadband(-gamepad1.right_stick_x);
        if (fieldCentricMode && imuReady) drive.driveFieldCentric(strafe, forward, rotate, heading);
        else drive.driveFieldCentric(strafe, forward, rotate, 0.0);
        else drive.driveRobotCentric(strafe, forward, rotate);
    }
    private double applyDeadband(double x) {return Math.abs(x)>0.05?x:0.0;}
    private void initImu() {
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            imu.initialize(params); imu.resetYaw();
            imuReady = true;
        } catch (Exception e) {imuReady = false;}
    }
    private double getYawRadians() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }
    private double normalizeRadians(double x) {
        while (x>Math.PI) x-=2*Math.PI;
        while (x<-Math.PI) x+=2*Math.PI;
        return x;
    }
}