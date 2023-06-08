/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Tflite;


import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
//@TeleOp(name = "Tflite", group = "Robot")
@Autonomous(name = "Tflite", group = "Robot")
//@Disabled
public class Tflite extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    //private static final String TFOD_MODEL_ASSET = "model_unquant.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_unquant.tflite";

    //labels
/* private static final String[] LABELS = {
        "A",
        "B"
}; */
    private static final String LABEL_CLASS_A = " 0 Class 1 ";
    private static final String LABEL_CLASS_B = " 1 Class 2 ";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     =* Once yXZou've obtained a license key, copy the string from the Vuforia web site
     ]]a|||\[* and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AflZoOr/////AAABmWP/64ENLUgbtS99XwB2tYoDX5M8QSB4PsxuiHCTBRzj7NXnknsMLsSJ7YvUgvkF51p6eiSQiNVaoeSnaSiy1s+vcqh2jvX43SP1VqAY5qv9nfbDKr+Yrx/VlQhnfQ0CjNg/p2kzEYs0kfWTMEy6ktsTtM/WfEAkRno7c/Xoi8CQFu0hHjPdVTwVbEGLo9boVoFhjFGkSlYzOqQffF5GUnuXKbz3t4xaXm2sOp9J5stKnM/G5qx4fKifezLggeU/yki1F6+y1UMOTfWDd2lyWJBpxqx49GXOaVb4LwSlhcIYZAk7ZrieQji172ce38KhvqGrMF50dP8tkXvDu+h9WVQ0Xj0vc/XUr0mGQZoHjKeY";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /* Declare OpMode members. */
    private DcMotor lFront = null;
    private DcMotor rFront = null;
    private DcMotor lBack = null;
    private DcMotor rBack = null;

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double BACKWARD_SPEED = -0.6;

    public static final double STRAFESLOW = 0.4; //slow Strafe
    public static final double STRAFEFAST = 0.9;    //  fastStrafe
    public double strafeSpeed = STRAFESLOW;

    // TIME

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        initHardware();
        activateTensorFlow();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start Autonomous");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedrecognitions = tfod.getUpdatedRecognitions();
            if (updatedrecognitions != null) {
                processRecognitions(updatedrecognitions);
                telemetry.update();
            }
        }
        // Stop and shutdown
        stopMoving();
        tfod.shutdown();
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        //version of Tensor flow model
        tfodParameters.isModelTensorFlow2 = false;
        // Resulution is 300x300 pixels
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABEL_CLASS_A, LABEL_CLASS_B);
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_CLASS_A, LABEL_CLASS_B);
    }

    private void activateTensorFlow() {
/**
 * Activate TensorFlow Object Detection before we wait for the start command.
 * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
 **/
        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
    }

    /**
     * initialize hardware
     */
    private void initHardware() {

        lFront = hardwareMap.get(DcMotor.class, "leftfront");
        rFront = hardwareMap.get(DcMotor.class, "rightfront");
        lBack = hardwareMap.get(DcMotor.class, "leftback");
        rBack = hardwareMap.get(DcMotor.class, "rightback");

        lFront.setDirection(DcMotor.Direction.REVERSE);
        rFront.setDirection(DcMotor.Direction.FORWARD);
        lBack.setDirection(DcMotor.Direction.REVERSE);
        rBack.setDirection(DcMotor.Direction.FORWARD);
    }

    /**
     * Good stuffs
     */
    private void moveLeft() {
        // Implement your robot movement logic to move forward
        telemetry.addLine("Cone");
        lFront.setPower(strafeSpeed);
        lBack.setPower(-strafeSpeed);
        rFront.setPower(-strafeSpeed);
        rBack.setPower(strafeSpeed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
        }
    }

    private void moveRight() {
        // Implement your robot movement logic to move backward
        telemetry.addLine("Pole");
        lFront.setPower(-strafeSpeed);
        lBack.setPower(strafeSpeed);
        rFront.setPower(strafeSpeed);
        rBack.setPower(-strafeSpeed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
        }
    }

    private void stopMoving() {
        // Stop the robot
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
        }
    }

    // processRecognitions is the movement relative to recognition.
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
            moveRight();
        } else if (foundClassB && !foundClassA) {
            moveLeft();
        } else {
            // Handle other cases or implement a fallback behavior
            telemetry.addLine("Failed to find anything!");
            stopMoving();
        }
    }
}