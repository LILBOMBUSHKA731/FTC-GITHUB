package org.firstinspires.ftc.teamcode.Tflite;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous(name = "TensorFlowVuforiaAutonomous", group = "Autonomous")
public class TensorFlowVuforiaAutonomous extends LinearOpMode {

    private static final String VUFORIA_KEY = "AflZoOr/////AAABmWP/64ENLUgbtS99XwB2tYoDX5M8QSB4PsxuiHCTBRzj7NXnknsMLsSJ7YvUgvkF51p6eiSQiNVaoeSnaSiy1s+vcqh2jvX43SP1VqAY5qv9nfbDKr+Yrx/VlQhnfQ0CjNg/p2kzEYs0kfWTMEy6ktsTtM/WfEAkRno7c/Xoi8CQFu0hHjPdVTwVbEGLo9boVoFhjFGkSlYzOqQffF5GUnuXKbz3t4xaXm2sOp9J5stKnM/G5qx4fKifezLggeU/yki1F6+y1UMOTfWDd2lyWJBpxqx49GXOaVb4LwSlhcIYZAk7ZrieQji172ce38KhvqGrMF50dP8tkXvDu+h9WVQ0Xj0vc/XUr0mGQZoHjKeY";
    private static final String TFOD_MODEL_ASSET = "model_unquant.tflite";
    private static final String LABEL_CLASS_A = "0 Class 1\n";
    private static final String LABEL_CLASS_B = "1 Class 2\n";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeHardware();
        initializeVuforia();
        initializeTensorFlow();
        activateTensorFlow();

        waitForStart();

        if (opModeIsActive()) {
            List<Recognition> recognitions = tfod.getUpdatedRecognitions();

            if (recognitions != null) {
                processRecognitions(recognitions);
            }
        }

        // Stop the robot after processing the recognitions
        stopMoving();

        // Cleanup resources
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initializeHardware() {
        // Initialize your hardware here (e.g., motors)
        leftMotor = hardwareMap.get(DcMotor.class, "leftfront");
        rightMotor = hardwareMap.get(DcMotor.class, "rightfront");

        // Set motor directions, if necessary
        if (leftMotor != null) {
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        if (rightMotor != null) {
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    private void initializeVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1"); // Use the webcam
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initializeTensorFlow() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_CLASS_A, LABEL_CLASS_B);
    }

    private void activateTensorFlow() {
        if (tfod != null) {
            tfod.activate();
        }
    }

    private void processRecognitions(List<Recognition> recognitions) {
        boolean foundClassA = false;
        boolean foundClassB = false;

        for (Recognition recognition : recognitions) {
            if (recognition.getLabel().equals(LABEL_CLASS_A)) {
                foundClassA = true;
            } else if (recognition.getLabel().equals(LABEL_CLASS_B)) {
                foundClassB = true;
            }
        }

        if (foundClassA && !foundClassB) {
            moveForward();
        } else if (foundClassB && !foundClassA) {
            moveBackward();
        } else {
            // Handle other cases or implement a fallback behavior
            stopMoving();
        }
    }

    private void moveForward() {
        if (runtime != null && opModeIsActive()) {
            runtime.reset();

            while (runtime.seconds() < 1.0 && opModeIsActive()) {
                // Implement your robot movement logic to move forward
                if (leftMotor != null) {
                    leftMotor.setPower(0.5);
                }

                if (rightMotor != null) {
                    rightMotor.setPower(0.5);
                }
            }

            stopMoving();
        }
    }

    private void moveBackward() {
        if (runtime != null && opModeIsActive()) {
            runtime.reset();

            while (runtime.seconds() < 1.0 && opModeIsActive()) {
                // Implement your robot movement logic to move backward
                if (leftMotor != null) {
                    leftMotor.setPower(-0.5);
                }

                if (rightMotor != null) {
                    rightMotor.setPower(-0.5);
                }
            }

            stopMoving();
        }
    }

    private void stopMoving() {
        // Stop the robot
        if (leftMotor != null) {
            leftMotor.setPower(0);
        }

        if (rightMotor != null) {
            rightMotor.setPower(0);
        }
    }
}






