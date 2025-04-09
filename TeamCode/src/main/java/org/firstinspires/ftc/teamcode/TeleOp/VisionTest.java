package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Camera to Servo Angle")
public class CameraToServoOpMode extends LinearOpMode {

    private OpenCvWebcam webcam;
    private Servo servo;
    private BlueObjectPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware map
        servo = hardwareMap.get(Servo.class, "servo");

        // Camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        pipeline = new BlueObjectPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double angle = pipeline.getDetectedAngle();

            // Map angle (-90 to +90) → servo position (0.0 to 1.0)
            double servoPos = (angle + 90) / 180.0;
            servoPos = Math.max(0.0, Math.min(1.0, servoPos));  // Clamp to valid range

            servo.setPosition(servoPos);

            telemetry.addData("Angle", angle);
            telemetry.addData("Servo Pos", servoPos);
            telemetry.update();

            sleep(20); // Reduce CPU usage
        }

        webcam.stopStreaming();
    }

    // ===== Pipeline Class =====
    class BlueObjectPipeline extends OpenCvPipeline {

        private double detectedAngle = 0;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Blue HSV range
            Scalar lowerBlue = new Scalar(100, 150, 0);
            Scalar upperBlue = new Scalar(140, 255, 255);

            Mat mask = new Mat();
            Core.inRange(hsv, lowerBlue, upperBlue, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                double maxArea = 0;
                MatOfPoint maxContour = contours.get(0);

                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area > maxArea) {
                        maxArea = area;
                        maxContour = contour;
                    }
                }

                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(maxContour.toArray()));
                detectedAngle = rect.angle;
            }

            return input;
        }

        public double getDetectedAngle() {
            return detectedAngle;
        }
    }
}

