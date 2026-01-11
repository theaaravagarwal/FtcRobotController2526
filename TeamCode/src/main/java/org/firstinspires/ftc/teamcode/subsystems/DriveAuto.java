package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DriveAuto {

    private final Drive drive;
    private final GoBildaPinpointDriver pinpoint;
    private final LinearOpMode opMode;

    private double headingOffset = 0.0;

    public DriveAuto(Drive drive, GoBildaPinpointDriver pinpoint, LinearOpMode opMode) {
        this.drive = drive;
        this.pinpoint = pinpoint;
        this.opMode = opMode;
    }

    public void setHeadingOffset(double off) {
        headingOffset = off;
    }

    private double normalize(double x) {
        while (x > Math.PI) x -= 2*Math.PI;
        while (x < -Math.PI) x += 2*Math.PI;
        return x;
    }

    private double getHeading() {
        double raw = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
        return normalize(raw - headingOffset + Math.PI);   // EXACT TeleOp logic
    }

    public void turnTo(double thd) {
        double kP = 1.5;
        while (opMode.opModeIsActive()) {
            double current = getHeading();
            double error = normalize(thd - current);
            if (Math.abs(error) < Math.toRadians(2)) break;
            drive.robotCentric(0, 0, error * kP);
            opMode.sleep(10);
        }
        drive.stop();
    }

    public void driveTo(double tx, double ty, double v) {
        double kP = 0.06;

        while (opMode.opModeIsActive()) {
            Pose2D pose = updateAndGetPose();

            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);

            double dx = tx - x;
            double dy = ty - y;

            if (Math.hypot(dx, dy) < 1.0) break;

            double mvx = dx * kP;
            double mvy = dy * kP;

            drive.fieldCentric(mvx * v, mvy * v, 0, getHeading());
            opMode.sleep(15);
        }

        drive.stop();
    }

    public void driveRelative(double vert, double lat, double v) {
        Pose2D pose = updateAndGetPose();
        double hd = getHeading();

        double x0 = pose.getX(DistanceUnit.INCH);
        double y0 = pose.getY(DistanceUnit.INCH);

        double tx = x0 + (Math.cos(hd) * vert - Math.sin(hd) * lat);
        double ty = y0 + (Math.sin(hd) * vert + Math.cos(hd) * lat);

        driveTo(tx, ty, v);
    }

    private Pose2D updateAndGetPose() {
        pinpoint.update();
        return pinpoint.getPosition();
    }
}
