package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name="lilsumtin", group="Robot")
public class DriveDistance extends LinearOpMode {

    // Declare robot hardware variables
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    // Declare constants for driving
    final double COUNTS_PER_REV = 1120; // Number of encoder counts per motor revolution
    final double WHEEL_DIAMETER = 4; // Diameter of the wheels in inches
    final double GEAR_RATIO = 1; // Gear ratio between motor and wheel
    final double COUNTS_PER_INCH = (COUNTS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);

    @Override
    public void runOpMode() {

        // Initialize robot hardware
        leftMotor = hardwareMap.get(DcMotor.class, "leftfront");
        rightMotor = hardwareMap.get(DcMotor.class, "rightfront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftback");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightback");


        // Set direction of motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        // Wait for start button to be pressed
        waitForStart();
        // Set motor modes
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Set target positions for motors
        int leftTarget = (int) (COUNTS_PER_INCH * 18); // 1.5 feet = 18 inches
        int rightTarget = (int) (COUNTS_PER_INCH * 18);
        int backleftTarget = (int) (COUNTS_PER_INCH * 18); // 1.5 feet = 18 inches
        int backrightTarget = (int) (COUNTS_PER_INCH * 18);
        // Set motor powers and target positions
        leftMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        backRightMotor.setPower(0.5);
        leftMotor.setTargetPosition(leftTarget);
        backLeftMotor.setTargetPosition(backleftTarget);
        rightMotor.setTargetPosition(rightTarget);
        backRightMotor.setTargetPosition(backrightTarget);

        // Wait until motors have reached target positions
        while (opModeIsActive() && (leftMotor.isBusy() || rightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy())) {
            telemetry.addData("Status", "Moving forward...");
            telemetry.update();
            idle();
        }
        sleep(3000);

        // Stop motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        telemetry.addData("Status", "Done!");
        telemetry.update();
    }
}