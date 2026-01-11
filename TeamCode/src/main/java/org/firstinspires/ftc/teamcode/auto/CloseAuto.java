package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@Autonomous(name = "CloseAuto", group = "Debug")
public class CloseAuto extends LinearOpMode {

    private Drive drive;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        // init drive
        drive = new Drive();
        drive.init(hardwareMap);

        // init pinpoint (optional)
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            configurePinpoint();
        } catch (Exception e) {
            pinpoint = null;
        }

        telemetry.addLine("OtherAuto ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        long start = System.currentTimeMillis();
        long durationMs = 2000; // 2 seconds

        while (opModeIsActive() &&
                System.currentTimeMillis() - start < durationMs) {

            // straight forward, 67% speed
            drive.robotCentric(0, 0.67, 0);

            double x = 0, y = 0, rawHeading = 0, correctedHeading = 0;

            if (pinpoint != null) {
                pinpoint.update();
                Pose2D pose = pinpoint.getPosition();
                x = pose.getX(DistanceUnit.INCH);
                y = pose.getY(DistanceUnit.INCH);
                rawHeading = pose.getHeading(AngleUnit.RADIANS);
                correctedHeading = rawHeading + Math.PI; // flipped heading
            }

            telemetry.addLine("=== OtherAuto ===");
            telemetry.addData("Time (ms)", System.currentTimeMillis() - start);

            telemetry.addLine("Pose:");
            telemetry.addData("X (in)", x);
            telemetry.addData("Y (in)", y);
            telemetry.addData("Raw Heading (deg)", Math.toDegrees(rawHeading));
            telemetry.addData("Corrected Heading (deg)", Math.toDegrees(correctedHeading));

            telemetry.addLine("Powers:");
            telemetry.addData("FL", drive.getLastFL());
            telemetry.addData("FR", drive.getLastFR());
            telemetry.addData("BL", drive.getLastBL());
            telemetry.addData("BR", drive.getLastBR());

            telemetry.update();
            sleep(50);
        }

        drive.stop();
        telemetry.addLine("OtherAuto done");
        telemetry.update();
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
    }
}
