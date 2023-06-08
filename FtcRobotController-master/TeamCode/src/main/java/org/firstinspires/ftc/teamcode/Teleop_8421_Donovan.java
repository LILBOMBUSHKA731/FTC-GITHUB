/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/**
 * This particular OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="CopyCat", group="Robot")
public class Teleop_8421_Donovan extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor lFront = null;
    private DcMotor rFront = null;
    private DcMotor lBack = null;
    private DcMotor rBack = null;

    public DcMotor lift = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;


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

    public boolean highGoal = false;    //reflects whether the user pressed "Y" on joystick2
    public boolean lowGoal = false;     //reflects whether the user pressed "B" on joystick2
    public boolean mediumGoal = false;   //reflects whether the user pressed "X" on joystick2
    public boolean groundGoal = false;   //reflects whether the user pressed "X" on joystick2
    public boolean release = false;   //reflects whether the user pressed "A" on joystick2 or moved joysticks
    public boolean slowMode = true;   // run slow mode

    // TIME

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Define and Initialize Motors
        lFront = hardwareMap.get(DcMotor.class, "leftfront");
        rFront = hardwareMap.get(DcMotor.class, "rightfront");
        lBack = hardwareMap.get(DcMotor.class, "leftback");
        rBack = hardwareMap.get(DcMotor.class, "rightback");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        lFront.setDirection(DcMotor.Direction.REVERSE);
        rFront.setDirection(DcMotor.Direction.FORWARD);
        lBack.setDirection(DcMotor.Direction.REVERSE);
        rBack.setDirection(DcMotor.Direction.FORWARD);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double left;
        double right;
        int one_touch_lift_up = 0;  // move lift up
        int one_touch_lift_down = 0;   //move lift down
        int yButton = 0;    //reflects whether the user pressed "Y"
        int xButton = 0;     //reflects whether the user pressed "Y"

       // int low_goal = 0;    // flag to move lift to low goal position.
       // double ground = 0;
        // double strafe = 0;
        // double liftCurrent = 0.0;
       // double targetPosition = 0.0;
        double speed = 1;

        // ground = lift.getCurrentPosition();


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        strafeSpeed = STRAFEFAST;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)

            // SLOW MODE
            if (gamepad1.left_bumper) {
                strafeSpeed = STRAFESLOW;
                speed = 1.5;
            }
            if (gamepad1.right_bumper) {
                strafeSpeed = STRAFEFAST;
                speed = 1;
            }

            //strafe = gamepad1.right_stick_x;

            if ((gamepad1.left_trigger > 0.5) || (gamepad1.right_trigger > 0.5)) {

                if (gamepad1.right_trigger > 0.5) {
                    lFront.setPower(strafeSpeed);
                    lBack.setPower(-strafeSpeed);
                    rFront.setPower(-strafeSpeed);
                    rBack.setPower(strafeSpeed);
                }
                if (gamepad1.left_trigger > 0.5) {
                    lFront.setPower(-strafeSpeed);
                    lBack.setPower(strafeSpeed);
                    rFront.setPower(strafeSpeed);
                    rBack.setPower(-strafeSpeed);
                }
                left = 0;
                right = 0;
                //     *****************  End strafe code *****************************************
            } else {
                left = -(gamepad1.left_stick_y / speed);
                right = -(gamepad1.right_stick_y / speed);
                if (((left > 0.7) || (left < -0.7)) && ((right > 0.7) || (right < -0.7))) {
                    left = (left / 1.0);    // fast modee
                    right = (right / 1.0);
                } else {
                    left = (left / 2);      // slow mode
                    right = (right / 2);
                }
                lFront.setPower(left);
                lBack.setPower(left);
                rFront.setPower(right);
                rBack.setPower(right);
            }
/*
            // Use gamepad left & right Triggers to open and close the claw

            if (gamepad2.right_trigger > 0.5) {
                // move to open claw.
                leftClaw.setPosition(OPEN_CLAW);
                rightClaw.setPosition(OPEN_CLAW);
                telemetry.update();
            } else if (gamepad2.left_trigger > 0.5) {
                // move to close claw.
                leftClaw.setPosition(CLOSE_CLAW);
                rightClaw.setPosition(CLOSE_CLAW);
            } else {
            }
            //
            //   this resets the ground  because the lift rotation counter keeps drifting.
            //
            if (gamepad2.right_bumper) {
                ground = lift.getCurrentPosition();
            }

            if (gamepad2.a) {
                lowGoal = true;
                mediumGoal = false;
                highGoal = false;
                targetPosition = LOWGOAL;
            }
            if (gamepad2.y) {
                mediumGoal = false;
                lowGoal = false;
                highGoal = true;
                targetPosition = HIGHGOAL;
            }
            if (gamepad2.x) {
                mediumGoal = true;
                lowGoal = false;
                highGoal = false;
                lift.setPower(0);
            }
            if (!mediumGoal && !lowGoal && !highGoal) {
                if (gamepad2.right_stick_y < -0.7) {
                    lift.setPower(0.99);  // lift up
                } else if (gamepad2.right_stick_y > 0.7) {
                    lift.setPower(-0.99);
                } else {
                    lift.setPower(0.07);
                }
            }
            //


            //   Move lift down 6 inches

            if (gamepad2.b) {
                lift.setPower(-0.7);
                runtime.reset();
                while ((runtime.seconds() < .04)) {
                    telemetry.addData("lift moving down", "Leg 1: %4d S Elapsed", lift.getCurrentPosition());
                    telemetry.update();
                }
                lift.setPower(0);
                targetPosition = lift.getCurrentPosition();
                lowGoal = false;
                mediumGoal = false;
                highGoal = false;

            }
            // medium goal
            if (mediumGoal) {

                if (lift.getCurrentPosition() < (Math.abs(ground + MEDIUMGOAL))) {
                    //  lift too low, go up
                    lift.setPower(0.95);

                } else if (lift.getCurrentPosition() > (Math.abs(ground + MEDIUMGOAL + 100))) {
                    // lift too high
                    lift.setPower(-0.4);

                } else {
                    // lift in sweety spot
                    mediumGoal = false;
                    lowGoal = false;
                    mediumGoal = false;
                    highGoal = false;
                    lift.setPower(0.05);

                }
                targetPosition = lift.getCurrentPosition();
            }
            // high goal
            if (highGoal) {

                if (lift.getCurrentPosition() < (Math.abs(ground + HIGHGOAL))) {
                    //  lift too low, go up
                    lift.setPower(0.95);
                    telemetry.addData("lift moving up", "Leg 1: %4d S Elapsed", lift.getCurrentPosition());
                    telemetry.update();
                } else if (lift.getCurrentPosition() > (Math.abs(ground + HIGHGOAL + 100))) {
                    // lift too high
                    lift.setPower(-0.4);
                    telemetry.addData("lift moving DOWN", "Leg 1: %4d S Elapsed", lift.getCurrentPosition());
                    telemetry.update();
                } else {
                    // lift in sweet spot
                    highGoal = false;
                    lowGoal = false;
                    mediumGoal = false;
                    highGoal = false;
                    lift.setPower(0.05);

                }
                targetPosition = lift.getCurrentPosition();
            }
            // low goal
            if (lowGoal) {

                if (lift.getCurrentPosition() < (Math.abs(ground + LOWGOAL))) {
                    //  lift too low, go up
                    lift.setPower(0.95);
                    telemetry.addData("lift moving up", "Leg 1: %4d S Elapsed", lift.getCurrentPosition());
                    telemetry.update();
                } else if (lift.getCurrentPosition() > (Math.abs(ground + LOWGOAL + 100))) {
                    // lift too high
                    lift.setPower(-0.4);
                    telemetry.addData("lift moving up", "Leg 1: %4d S Elapsed", lift.getCurrentPosition());
                    telemetry.update();
                } else {
                    // lift in sweety spot
                    lowGoal = false;
                    lowGoal = false;
                    mediumGoal = false;
                    highGoal = false;
                    lift.setPower(0.05);

                }
                targetPosition = lift.getCurrentPosition();
            }*/


            // Pace this loop so jaw action is reasonable speed.
            sleep(1);
        }
    }
}