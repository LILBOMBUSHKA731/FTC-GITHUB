/*


package org.firstinspires.ftc.teamcode.OpenCV;


import android.annotation.SuppressLint;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.PoleDetectionPipeline;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


import java.util.ArrayList;
import java.util.List;

@Autonomous(name="PoleAuto", group="Robot")

public class YellowPoleAuto extends LinearOpMode {

    OpenCvCamera camera;

PoleDetectionPipeline poleDetectionPipeline;

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

    // Declare OpMode members.
    private DcMotor lFront = null;
    private DcMotor rFront = null;
    private DcMotor lBack = null;
    private DcMotor rBack = null;

    /*public DcMotor lift = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;*/

/*
    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double BACKWARD_SPEED = -0.6;

    public static final double GROUNDGOAL = 150; // height of ground goal in ticks (13.3 inches off floor)
    public static final double LOWGOAL = 1050; // height of low goal in ticks (13.3 inches off floor)
    public static final double MEDIUMGOAL = 1800; // height of low goal in ticks (13.3 inches off floor)
    public static final double HIGHGOAL = 2750; // height of low goal in ticks (13.3 inches off floor)

    public static Scalar lowerYellow1 = new Scalar(116.2, 204, 177.1);
    public static Scalar upperYellow1 = new Scalar(117.6, 255, 255);
    public static Scalar lowerYellow2 = new Scalar(113.3, 208.3, 177.1);
    public static Scalar upperYellow2 = new Scalar(119, 255, 255);


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

    Mat mask1 = new Mat();
    Mat mask2 = new Mat();
    Mat mask = new Mat();

    // STATE MACHINE

    /*public boolean highGoal = false;    //reflects whether the user pressed "Y" on joystick2
    public boolean lowGoal = false;     //reflects whether the user pressed "B" on joystick2
    public boolean mediumGoal = false;   //reflects whether the user pressed "X" on joystick2
    public boolean groundGoal = false;   //reflects whether the user pressed "X" on joystick2
    public boolean release = false;   //reflects whether the user pressed "A" on joystick2 or moved joysticks
    public boolean slowMode = true;   // run slow mode*/

    // TIME
/*
    private ElapsedTime runtime = new ElapsedTime();

    private Telemetry telemetry;
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
        poleDetectionPipeline = new PoleDetectionPipeline();

        camera.setPipeline(poleDetectionPipeline);
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
            List<MatOfPoint> contours = new ArrayList<>();
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

            @Override
            public Mat processFrame (Mat input) {
                contours.clear();
                Mat hsvImage = new Mat();
                Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);
                Core.inRange(hsvImage, lowerYellow1, upperYellow1, mask1);
                Core.inRange(hsvImage, lowerYellow2, upperYellow2, mask2);
                Core.bitwise_or(mask1, mask2, mask);
                Imgproc.erode(mask, mask, kernel);
                Imgproc.dilate(mask, mask, kernel);
                Mat hierarchy = new Mat();
                Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
                hierarchy.release();
                hsvImage.release();

                return input;
            }
            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        // Update the telemetry
/*
        while (opModeIsActive()) {
            // Actually do something useful

            if (input >= 100) {

                lFront.setPower(strafeSpeed);
                lBack.setPower(-strafeSpeed);
                rFront.setPower(-strafeSpeed);
                rBack.setPower(strafeSpeed);
                runtime.reset();

                while (opModeIsActive() && (runtime.seconds() < 0.5)) {

                    lFront.setPower(0);
                    lBack.setPower(0);
                    rFront.setPower(0);
                    rBack.setPower(0);

                    sleep(1000);
                }
            }
        }
    }
}
*/