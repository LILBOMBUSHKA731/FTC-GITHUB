package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_SIMPLEX;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Follow", group="Autonomous")
public class Robot extends LinearOpMode {

    // Declare the camera and motors
    private VideoCapture camera;
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() {

        // Load the OpenCV library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // Initialize the camera and motors
        camera = new VideoCapture(0);
        leftMotor = hardwareMap.get(DcMotor.class, "leftf");
        rightMotor = hardwareMap.get(DcMotor.class, "rightf");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {

            // Read a frame from the camera
            Mat frame = new Mat();
            camera.read(frame);

            // Convert the frame to grayscale
            Mat gray = new Mat();
            Imgproc.cvtColor(frame, gray, Imgproc.COLOR_BGR2GRAY);

            // Apply a Gaussian blur to the grayscale image
            Mat blurred = new Mat();
            Imgproc.GaussianBlur(gray, blurred, new Size(11, 11), 0);

            // Threshold the blurred image to create a binary image
            Mat binary = new Mat();
            Imgproc.threshold(blurred, binary, 0, 255, Imgproc.THRESH_BINARY_INV + Imgproc.THRESH_OTSU);

            // Find contours in the binary image
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(binary, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest contour, which should be the user's hand
            double maxArea = -1;
            Rect maxRect = null;
            for (int i = 0; i < contours.size(); i++) {
                Rect rect = Imgproc.boundingRect(contours.get(i));
                double area = Imgproc.contourArea(contours.get(i));
                if (area > maxArea) {
                    maxArea = area;
                    maxRect = rect;
                }
            }

            // Draw a rectangle around the largest contour, if one was found
            if (maxRect != null) {
                Imgproc.rectangle(frame, maxRect.tl(), maxRect.br(), new Scalar(0, 255, 0), 2);

                // Calculate the position of the center of the largest contour
                Point center = new Point(maxRect.x + maxRect.width / 2, maxRect.y + maxRect.height / 2);
                double position = center.x / frame.size().width;

                // Calculate the motor power based on the position of the center of the largest contour
                double motorPower = (position - 0.5) * 2;

                // Set the motor power
                leftMotor.setPower(motorPower);
                rightMotor.setPower(motorPower);
            } else {
                // If no hand was found, stop the motors
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }

            // Show the frame with the detected hand
            Imgproc.putText(frame, "Press 'q' to quit", new Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(0, 255, 0), 2);


            // Wait for a key press or for 10 milliseconds, whichever comes first


            // Release the camera
            camera.release();

            sleep(3000);

            // Stop the motors
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            telemetry.addData("Status", "Done!");
            telemetry.update();

            // Clean up
            System.exit(0);
        }
    }
}