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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Interview Teleop", group="Robot")
//@Disabled
public class InterviewTeleop extends OpMode {

    // Declare OpMode members.
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor elevatorMotor = null;
    public DcMotor rightWinchMotor = null;
    public DcMotor leftWinchMotor = null;
    public DcMotor intakeMotor = null;

    public Servo bucketServo = null;
    public Servo droneServo = null;


    // Drivers
    private boolean isEncoderDriverDriving = true;

    // Intake
    public boolean isIntakeOn = false;
    public boolean isIntakePressed = false;


    // Winch
    public int targetWinchPosition;
    public boolean shouldWinchGoFullyUp = false;
    public boolean isWinchPressed = false;

    // Elevator
    public int targetElevatorPosition;
    public double elevatorPower = 1;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Define and Inited, most robots need the motor on one side to be ralize Motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator");
        leftWinchMotor = hardwareMap.get(DcMotor.class, "left_winch");
        rightWinchMotor = hardwareMap.get(DcMotor.class, "right_winch");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");


        // To drive forwareversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels. Gear Reduction or 90 Deg drives may require direction flips
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftWinchMotor.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setDirection(DcMotor.Direction.REVERSE);

        // ENCODER
        leftWinchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWinchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // ENCODER
        rightWinchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWinchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // BREAKS
        leftWinchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWinchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        bucketServo = hardwareMap.get(Servo.class, "bucket");
        droneServo = hardwareMap.get(Servo.class, "drone");


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        targetWinchPosition = MotorPositions.WINCH_HOVER_POSITION;
        targetElevatorPosition = MotorPositions.ELEVATOR_IN_POSITION;
        bucketServo.setPosition(MotorPositions.BUCKET_UP_POSITION);
        droneServo.setPosition(MotorPositions.DRONE_START_POSITION);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        DetermainDriver();
        RunDriver1Code();
        RunDriver2Code();

        telemetry.addData("Winchs Running to", "%.2f", (double) targetWinchPosition);
        telemetry.addData("left Winch Currently at", "%.2f", (double) leftWinchMotor.getCurrentPosition());
        telemetry.addData("right Winch Currently at", "%.2f", (double) rightWinchMotor.getCurrentPosition());

        telemetry.addData("BucketServo", "%.2f", bucketServo.getPosition());
        telemetry.addData("DroneServo", "%.2f", droneServo.getPosition());
        telemetry.addData("Elevator Running to", "%.2f", (double)targetElevatorPosition);
        telemetry.addData("Elevator Currently at", "%.2f", (double) elevatorMotor.getCurrentPosition());
        int i = 0;
        if (shouldWinchGoFullyUp) i = 0;
        else i = 1;
        telemetry.addData("Should Winch Go fully Up", "%.2f", (double) i);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    private  void DetermainDriver()
    {
        if (gamepad1.start)
        {
            isEncoderDriverDriving = true;

            // ENCODER
            leftWinchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftWinchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightWinchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightWinchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if (gamepad2.start)
        {
            isEncoderDriverDriving = false;


            leftWinchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rightWinchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void RunDriver1Code() {
        if (isEncoderDriverDriving) {

            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad1.y)
                bucketServo.setPosition(MotorPositions.BUCKET_UP_POSITION);
            else if (gamepad1.x)
                bucketServo.setPosition(MotorPositions.BUCKET_DOWN_POSITION);


            if (leftWinchMotor.getCurrentPosition() > MotorPositions.WINCH_HALF_UP_POSITION - 50) {
                if (gamepad1.dpad_down) // In
                {
                    targetElevatorPosition = MotorPositions.ELEVATOR_IN_POSITION;
                    bucketServo.setPosition(MotorPositions.BUCKET_UP_POSITION);
                    elevatorPower = MotorPositions.MAX_ELEVATOR_POWER;
                } else if (gamepad1.dpad_left) // little out
                {
                    targetElevatorPosition = MotorPositions.ELEVATOR_OUT1_POSITION;
                    elevatorPower = MotorPositions.MAX_ELEVATOR_POWER;
                } else if (gamepad1.dpad_up) // mid out
                {
                    targetElevatorPosition = MotorPositions.ELEVATOR_OUT2_POSITION;
                    elevatorPower = MotorPositions.MAX_ELEVATOR_POWER;
                } else if (gamepad1.dpad_right) // Far out
                {
                    targetElevatorPosition = MotorPositions.ELEVATOR_OUT3_POSITION;
                    elevatorPower = MotorPositions.MAX_ELEVATOR_POWER;
                }
            }


            if (gamepad1.left_trigger > 0.5) {
                targetElevatorPosition = MotorPositions.ELEVATOR_OUT3_POSITION;
                bucketServo.setPosition(MotorPositions.BUCKET_UP_POSITION);
                elevatorPower = MotorPositions.SLOW_ELEVATOR_POWER;
            }

            if (gamepad1.b) {
                droneServo.setPosition(MotorPositions.DRONE_LAUNCH_POSITION);
            }

            if (gamepad1.right_trigger > 0.5) {
                if (!isWinchPressed) {
                    isWinchPressed = true;
                    if (shouldWinchGoFullyUp) {
                        shouldWinchGoFullyUp = false;
                        targetWinchPosition = MotorPositions.WINCH_UP_POSITION;
                    } else {
                        shouldWinchGoFullyUp = true;
                        targetWinchPosition = MotorPositions.WINCH_HALF_UP_POSITION;
                    }
                }
            } else if (gamepad1.right_bumper) {
                if (elevatorMotor.getCurrentPosition() < MotorPositions.ELEVATOR_IN_POSITION + 100) {
                    // Use Right trigger and bumper to use the winch
                    targetWinchPosition = MotorPositions.WINCH_HOVER_POSITION;
                    isWinchPressed = false;
                    shouldWinchGoFullyUp = false;
                }
            } else {
                isWinchPressed = false;
            }

            // Winch Movement
            {
                // Determine new target position, and pass to motor controller
                leftWinchMotor.setTargetPosition(targetWinchPosition);
                rightWinchMotor.setTargetPosition(targetWinchPosition);

                // Turn On RUN_TO_POSITION
                leftWinchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightWinchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftWinchMotor.setPower(0.5);
                rightWinchMotor.setPower(0.5);

                if (!leftWinchMotor.isBusy()) {
                    // Stop all motion;
                    leftWinchMotor.setPower(0);
                    rightWinchMotor.setPower(0);

                    // Turn off RUN_TO_POSITION
                    leftWinchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightWinchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }

            // Elevator Movement
            {
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


            if (gamepad1.a && elevatorMotor.getCurrentPosition() < 200) {
                if (!isIntakePressed) {
                    if (isIntakeOn) {
                        // turn intake off
                        intakeMotor.setPower(0);
                        isIntakeOn = false;
                        if (targetWinchPosition != MotorPositions.WINCH_UP_POSITION) {
                            targetWinchPosition = MotorPositions.WINCH_HOVER_POSITION;
                        }
                    } else {
                        // turn intake on
                        intakeMotor.setPower(0.75);
                        isIntakeOn = true;
                        if (targetWinchPosition != MotorPositions.WINCH_UP_POSITION) {
                            targetWinchPosition = MotorPositions.WINCH_DOWN_POSITION;
                        }
                    }
                    isIntakePressed = true;
                }
            } else
                isIntakePressed = false;
        }
    }

    private  void RunDriver2Code()
    {
        if (!isEncoderDriverDriving) {

            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad2.y)
                bucketServo.setPosition(MotorPositions.BUCKET_UP_POSITION);
            else if (gamepad2.x)
                bucketServo.setPosition(MotorPositions.BUCKET_DOWN_POSITION);

            if (gamepad2.b) {
                droneServo.setPosition(MotorPositions.DRONE_LAUNCH_POSITION);
            }

            if (gamepad2.right_trigger > 0.3)
            {
                leftWinchMotor.setPower(gamepad2.right_trigger / 2f);
                rightWinchMotor.setPower(gamepad2.right_trigger / 2f);
            }
            else if (gamepad2.right_bumper)
            {
                leftWinchMotor.setPower(-0.5);
                rightWinchMotor.setPower(-0.5);
            }
            else
            {
                leftWinchMotor.setPower(0);
                rightWinchMotor.setPower(0);
            }

            if (gamepad2.left_trigger > 0.25)
            {
                elevatorMotor.setPower(gamepad2.left_trigger);
            }
            else if (gamepad2.left_bumper)
            {
                elevatorMotor.setPower(-1);
            }
            else
            {
                elevatorMotor.setPower(0);
            }

            if (gamepad2.a)
            {
                intakeMotor.setPower(1);
            }
            else
            {
                intakeMotor.setPower(0);
            }
        }
    }


}
