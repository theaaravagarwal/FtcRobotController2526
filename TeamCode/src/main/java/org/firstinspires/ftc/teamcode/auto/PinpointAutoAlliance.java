package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.DriveAuto;

@Autonomous(name="PinpointAutoAlliance")
public class PinpointAutoAlliance extends LinearOpMode {

    private Drive drive;
    private GoBildaPinpointDriver pinpoint;
    private DriveAuto auto;

    private enum Alliance {RED, BLUE}
    private Alliance chosenAlliance = Alliance.RED;

    @Override
    public void runOpMode() {

        drive = new Drive();
        drive.init(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        auto = new DriveAuto(drive, pinpoint, this);

        // Store offset like TeleOp does
        pinpoint.update();
        auto.setHeadingOffset(pinpoint.getPosition().getHeading(AngleUnit.RADIANS));

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) chosenAlliance = Alliance.RED;
            if (gamepad1.b) chosenAlliance = Alliance.BLUE;

            Pose2D pose = pinpoint.getPosition();

            telemetry.addLine("=== Select Alliance ===");
            telemetry.addLine("Press A → RED");
            telemetry.addLine("Press B → BLUE");
            telemetry.addData("Chosen", chosenAlliance);

            telemetry.addData("X (in)", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", pose.getY(DistanceUnit.INCH));
            telemetry.addData("Raw Heading (deg)",
                    Math.toDegrees(pose.getHeading(AngleUnit.RADIANS)));
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        if (chosenAlliance == Alliance.RED) runRedAuto();
        else runBlueAuto();
    }

    private void runRedAuto() {
        auto.turnTo(0);
        auto.driveRelative(24, 0, 0.6);
    }

    private void runBlueAuto() {
        auto.turnTo(0);
        auto.driveRelative(24, 0, 0.6);
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