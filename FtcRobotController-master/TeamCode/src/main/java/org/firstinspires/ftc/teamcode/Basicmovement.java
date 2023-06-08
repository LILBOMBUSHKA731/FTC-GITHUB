package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="Drive-and-Spin", group="Robot")
public class Basicmovement extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private double targetDistance = 6.0; // inches
    private double wheelDiameter = 4.0; // inches
    private double ticksPerRevolution = 1120.0;
    private double distancePerTick = (wheelDiameter * Math.PI) / ticksPerRevolution;
    private int targetTicks = (int) (targetDistance / distancePerTick);

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.get(DcMotor.class, "leftfront");
        rightMotor = hardwareMap.get(DcMotor.class, "rightfront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftback");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightback");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        // Move forward for 1.5 feet
        leftMotor.setTargetPosition(targetTicks);
        rightMotor.setTargetPosition(targetTicks);
        backLeftMotor.setTargetPosition(targetTicks);
        backRightMotor.setTargetPosition(targetTicks);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        backLeftMotor.setPower(-0.5);
        backRightMotor.setPower(-0.5);

        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Status", "Moving forward...");
            telemetry.update();
            idle();

            sleep(500);
        }
        leftMotor.setPower(FORWARD_SPEED);
        rightMotor.setPower(FORWARD_SPEED);
        backLeftMotor.setPower(FORWARD_SPEED);
        backRightMotor.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        leftMotor.setPower(TURN_SPEED);
        rightMotor.setPower(-TURN_SPEED);
        backLeftMotor.setPower(TURN_SPEED);
        backRightMotor.setPower(-TURN_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Status", "Spinning...");
            telemetry.update();
        }

        sleep(3000);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        telemetry.addData("Status", "Done!");
        telemetry.update();
    }
}