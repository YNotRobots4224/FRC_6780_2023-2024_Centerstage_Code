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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Score, Park Right, Red Close", group="Robot")
//@Disabled
public class ScoreParkRightRedClose extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;


    public DcMotor elevatorMotor = null;
    public DcMotor winchMotor = null;
    public DcMotor intakeMotor = null;
    public Servo bucketServo = null;



    // Intake
    public boolean isIntakeOn = false;
    public boolean isIntakePressed = false;


    // Winch
    public int targetWinchPosition;

    // Elevator
    public int targetElevatorPosition;
    public double elevatorPower = 1;
    public final double MAX_ELEVATOR_POWER = 1;
    public final double SLOW_ELEVATOR_POWER = 0.5;



    // CONST variables.
    public final double MOVEMENT_SPEED = 0.5;

    public final double BUCKET_UP_POSITION = 0.68;
    public final double BUCKET_DOWN_POSITION = 1;
    public double winchPower = 0.5;

    @Override
    public void runOpMode() {

        // Define and Inited, most robots need the motor on one side to be ralize Motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");

        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator");
        winchMotor = hardwareMap.get(DcMotor.class, "left_winch");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        bucketServo = hardwareMap.get(Servo.class, "bucket");


        bucketServo.setPosition(0.4);

        // To drive forwareversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels. Gear Reduction or 90 Deg drives may require direction flips
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        winchMotor.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setDirection(DcMotor.Direction.REVERSE);

        // ENCODER
        winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // BREAKS
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();
        MoveWinch(350);

        while (runtime.seconds() < 2) {

        }

        runtime.reset();
        while (runtime.seconds() < 0.2) {
            MoveRobot(-1, 0, 0);
        }
        MoveRobot(0, 0, 0);

        runtime.reset();
        while (runtime.seconds() < 0.2) {

        }

        runtime.reset();
        while (runtime.seconds() < 0.265) {
            MoveRobot(0, 1, 0);
        }
        MoveRobot(0, 0, 0);

        runtime.reset();
        while (runtime.seconds() < 0.2) {

        }

        runtime.reset();
        while (runtime.seconds() < 0.35) {
            MoveRobot(-1, 0, 0);
        }
        MoveRobot(0, 0, 0);

        runtime.reset();
        while (runtime.seconds() < 0.2) {

        }

        runtime.reset();
        while (runtime.seconds() < 1.5) {
            MoveWinch(650);
        }

        runtime.reset();
        while (runtime.seconds() < 2.5) {
            MoveElevator(3000);
        }

        runtime.reset();
        while (runtime.seconds() < 1.25) {
            bucketServo.setPosition(BUCKET_DOWN_POSITION);
        }

        runtime.reset();
        while (runtime.seconds() < 0.5) {
            bucketServo.setPosition(BUCKET_UP_POSITION);
        }

        runtime.reset();
        while (runtime.seconds() < 2.5) {
            MoveElevator(0);
        }

        runtime.reset();
        while (runtime.seconds() < 1.5) {
            MoveWinch(350);
        }

        runtime.reset();
        while (runtime.seconds() < 0.5) {
            MoveRobot(1, 0, 0);
        }
        MoveRobot(0, 0, 0);

        runtime.reset();
        while (runtime.seconds() < 0.30) {
            MoveRobot(0, 1, 0);
        }
        MoveRobot(0, 0, 0);

        runtime.reset();
        while (runtime.seconds() < 0.5) {
            MoveWinch(0);
        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        sleep(30000);
    }

    private void MoveRobot(double movementX, double movementZ, double rotationY)
    {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(movementZ) + Math.abs(movementX) + Math.abs(rotationY), 1);
        double backLeftPower = (movementZ - movementX + rotationY) / denominator;
        double frontLeftPower = (movementZ + movementX + rotationY) / denominator;
        double frontRightPower = (movementZ - movementX - rotationY) / denominator;
        double backRightPower = (movementZ + movementX - rotationY) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
    public void MoveWinch(int targetWinchPosition) {
        // Determine new target position, and pass to motor controller
        winchMotor.setTargetPosition(targetWinchPosition);

        // Turn On RUN_TO_POSITION
        winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        winchMotor.setPower(winchPower);

        if (!winchMotor.isBusy()) {
            // Stop all motion;
            winchMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void MoveElevator(int targetElevatorPosition) {
        // Determine new target position, and pass to motor controller
        elevatorMotor.setTargetPosition(targetElevatorPosition);

        // Turn On RUN_TO_POSITION
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        elevatorMotor.setPower(elevatorPower);

        if (!elevatorMotor.isBusy()) {
            // Stop all motion;
            elevatorMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
