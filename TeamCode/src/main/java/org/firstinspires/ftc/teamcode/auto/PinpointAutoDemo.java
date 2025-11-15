package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.DriveAuto;

@Autonomous(name="PinpointAutoDemo")
public class PinpointAutoDemo extends LinearOpMode {
    private Drive drive;
    private GoBildaPinpointDriver pinpoint;
    private DriveAuto auto;
    @Override
    public void runOpMode() {
        drive = new Drive();
        drive.init(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();
        auto = new DriveAuto(drive, pinpoint, this);
        waitForStart();
        //auto actions
        auto.turnTo(Math.toRadians(0)); //turn to face 0 deg
        auto.driveTo(24, 0, 1.0); //drive to 24,0 at max speed
        auto.turnTo(Math.toRadians(90)); //turn to face 90 deg
        auto.driveTo(24, 24, 0.8); //drive to 24,24 at 80% max speed
        auto.driveRelative(12, 0, 0.8); //drive 12 inches forward relative to the current hd position
    }
    private void configurePinpoint() {
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }
}
