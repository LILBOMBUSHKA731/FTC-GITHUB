package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="CamAuto", group="Robot")

public class CamAuto extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag IDs of sleeve
    int Left = 17;
    int Middle = 18;
    int Right = 19;
    AprilTagDetection tagOfInterest = null;


    /* Declare OpMode members. */
    private DcMotor lFront = null;
    private DcMotor rFront = null;
    private DcMotor lBack = null;
    private DcMotor rBack = null;

    /*public DcMotor lift = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;*/

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
 static final double BACKWARD_SPEED = -0.6;

    public static final double GROUNDGOAL = 150; // height of ground goal in ticks (13.3 inches off floor)
    public static final double LOWGOAL = 1050; // height of low goal in ticks (13.3 inches off floor)
    public static final double MEDIUMGOAL = 1800; // height of low goal in ticks (13.3 inches off floor)
    public static final double HIGHGOAL = 2750; // height of low goal in ticks (13.3 inches off floor)

    public static final double STRAFESLOW = 0.4; //slow Strafe
    public static final double STRAFEFAST = 0.9;    //  fastStrafe
    public double strafeSpeed = STRAFESLOW;

    public double lift_start = 0;

    // CLAW

    public static final double OPEN_CLAW = 1;
    public static final double CLOSE_CLAW = 0;
    public static final double MID_SERVO = 0.5;
    public static final double CLAW_SPEED = 0.02;        // sets rate to move servo
    double clawOffset = 0;
    double liftPosition = 0;

    // STATE MACHINE

    /*public boolean highGoal = false;    //reflects whether the user pressed "Y" on joystick2
    public boolean lowGoal = false;     //reflects whether the user pressed "B" on joystick2
    public boolean mediumGoal = false;   //reflects whether the user pressed "X" on joystick2
    public boolean groundGoal = false;   //reflects whether the user pressed "X" on joystick2
    public boolean release = false;   //reflects whether the user pressed "A" on joystick2 or moved joysticks
    public boolean slowMode = true;   // run slow mode*/

    // TIME

    private ElapsedTime runtime = new ElapsedTime();

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() {

        lFront = hardwareMap.get(DcMotor.class, "leftfront");
        rFront = hardwareMap.get(DcMotor.class, "rightfront");
        lBack = hardwareMap.get(DcMotor.class, "leftback");
        rBack = hardwareMap.get(DcMotor.class, "rightback");

        lFront.setDirection(DcMotor.Direction.REVERSE);
        rFront.setDirection(DcMotor.Direction.FORWARD);
        lBack.setDirection(DcMotor.Direction.REVERSE);
        rBack.setDirection(DcMotor.Direction.FORWARD);

        //lift = hardwareMap.get(DcMotor.class, "lift");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections != null) {

                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == Left || tag.id == Middle || tag.id == Right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        while (opModeIsActive()) {
            /* Actually do something useful */
            if(tagOfInterest == null) {
                telemetry.addLine("Tag not found ;(");

            }else{

            }
            if (tagOfInterest.id == Left) {

                lFront.setPower(strafeSpeed);
                lBack.setPower(-strafeSpeed);
                rFront.setPower(-strafeSpeed);
                rBack.setPower(strafeSpeed);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.5)) {

                }
                lFront.setPower(0);
                lBack.setPower(0);
                rFront.setPower(0);
                rBack.setPower(0);
                
                sleep(1000);

                lBack.setPower(TURN_SPEED);
                rFront.setPower(-TURN_SPEED);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 5.0)) {

                }

                // left code
            } else if (tagOfInterest == null || tagOfInterest.id == Middle) {
                lFront.setPower(FORWARD_SPEED);
                rFront.setPower(FORWARD_SPEED);
                lBack.setPower(FORWARD_SPEED);
                rBack.setPower(FORWARD_SPEED);
                runtime.reset();
                while  (opModeIsActive() && (runtime.seconds() < 2.0)) {

                }
                // middle code
            } else if (tagOfInterest.id == Right)
                // right code
                lFront.setPower(-strafeSpeed);
                lBack.setPower(strafeSpeed);
                rFront.setPower(strafeSpeed);
                rBack.setPower(-strafeSpeed);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 2.0)) {

                }
            }

            lFront.setPower(0);
            lBack.setPower(0);
            rFront.setPower(0);
            rBack.setPower(0);

            sleep(1000);
        }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}