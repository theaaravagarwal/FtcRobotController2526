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
    public DriveAuto(Drive drive, GoBildaPinpointDriver pinpoint, LinearOpMode opMode) {
        this.drive = drive;
        this.pinpoint = pinpoint;
        this.opMode = opMode;

    private double normalize(double x) {
        while (x>Math.PI) x-=2*Math.PI;
        while (x<-Math.PI) x+=2*Math.PI;
        return x;
    }
    public void turnTo(double thd) {
        double kP = 1.5;
        while (opMode.opModeIsActive()) {
            Pose2D pose = updateAndGetPose();
            double current = pose.getHeading(AngleUnit.RADIANS);
            double error = normalize(thd-current);
            if (Math.abs(error)<Math.toRadians(2)) break;
            double rot = error*kP;
            drive.robotCentric(0, 0, rot);
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
            double dx = tx-x;
            double dy = ty-y;
            double dist = Math.hypot(dx, dy);
            if (dist<1.0) break;
            double mvx = dx*kP;
            double mvy = dy*kP;
            double hd = pose.getHeading(AngleUnit.RADIANS);
            drive.fieldCentric(mvx*v, mvy*v, 0, hd);
            opMode.sleep(15);
        }
        drive.stop();
    }
    public void driveRelative(double vert, double lat, double v) { //vert and lat are in inches
        Pose2D pose = updateAndGetPose();
        double headingStart = pose.getHeading(AngleUnit.RADIANS);
        double x0 = pose.getX(DistanceUnit.INCH);
        double y0 = pose.getY(DistanceUnit.INCH);
        double tx = x0+(Math.cos(headingStart)*vert-Math.sin(headingStart)*lat);
        double ty = y0+(Math.sin(headingStart)*vert+Math.cos(headingStart)*lat);
        driveTo(tx, ty, v);
    }
    private Pose2D updateAndGetPose() {
        pinpoint.update();
        return pinpoint.getPosition();
    }
}
