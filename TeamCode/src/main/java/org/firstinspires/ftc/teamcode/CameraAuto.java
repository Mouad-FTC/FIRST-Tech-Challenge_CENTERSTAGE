package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous( name = "CameraAuto")
public class CameraAuto extends LinearOpMode {
    OpenCvCamera controlHubCam;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "webCam "), cameraMonitorViewId);

        CameraPipeline detector = new CameraPipeline(telemetry);

        controlHubCam.setPipeline(detector);

        controlHubCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                controlHubCam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }

        });
        waitForStart();
        while (opModeIsActive()){
            switch (detector.getLocation()){
                case RIGHT:
                    // Do something
                    break;
                case LEFT:
                    // Do something
                    break;
                case CENTER:
                    // Do something
            }
        }

    }
}
