package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IMU.Parameters;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentric;

@TeleOp(name = "OmniShooterTeleOp")
public class OmniShooterTeleOp extends OpMode {
    private FieldCentric drive;
    private IMU imu;
    private double headingOffsetRad = 0.0;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        Parameters params = new Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(params);

        drive = new FieldCentric();
        drive.init(hardwareMap);

        headingOffsetRad = getYawRadians();
    }

    @Override
    public void loop() {
        double strafe = applyDeadband(gamepad1.left_stick_x);
        double forward = applyDeadband(-gamepad1.left_stick_y); // up on the stick -> positive forward
        double rotate = applyDeadband(-gamepad1.right_stick_x);

        double slowMultiplier = gamepad1.left_bumper ? 0.4 : 1.0;
        double fineRotateMultiplier = gamepad1.right_bumper ? 0.6 : 1.0;

        double heading = normalizeRadians(getYawRadians() - headingOffsetRad);

        drive.driveFieldCentric(strafe * slowMultiplier, forward * slowMultiplier, rotate * fineRotateMultiplier, heading);

        if (gamepad1.y) {
            headingOffsetRad = getYawRadians();
        }

        telemetry.addData("Heading (rad)", heading);
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.addData("FL", drive.getLastFL());
        telemetry.addData("FR", drive.getLastFR());
        telemetry.addData("BL", drive.getLastBL());
        telemetry.addData("BR", drive.getLastBR());
        telemetry.update();
    }

    private double applyDeadband(double value) {
        double deadband = 0.05;
        return Math.abs(value) > deadband ? value : 0.0;
    }

    private double getYawRadians() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }
}
