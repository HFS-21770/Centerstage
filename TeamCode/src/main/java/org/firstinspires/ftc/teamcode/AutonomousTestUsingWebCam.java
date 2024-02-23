package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.template.BaseLinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "Autonomous Webcam")
public class AutonomousTestUsingWebCam extends BaseLinearOpMode {

    private VisionPortal visionPortal;
//    private AprilTagProcessor aprilTagProcessor;

    @Override
    public void runOpMode() {
//        aprilTagProcessor = new AprilTagProcessor.Builder()
//                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
//                .build();

        visionPortal = new VisionPortal
                .Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(true)
//                .addProcessor(aprilTagProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        waitForStart();


        while (opModeIsActive()) {
        }

        visionPortal.close();
    }
}
