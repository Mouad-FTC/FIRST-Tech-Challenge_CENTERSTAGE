package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp ( name = "TeleOpMode")
public class TeleOpMode extends LinearOpMode {

    //Declaring the actuators
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor motorSlide;
    DcMotor motorArm;
    Servo servoClawRot;
    Servo servoClaw1;
    Servo servoClaw2;

    //Declaring the timer
    ElapsedTime timer;
    // Define states for the Finite State Machine
    public enum State {
        IDLE,
        ACTIVE,
        COMPLETED

    }
    private State currentState;

    @Override
    public void runOpMode() {
        // Initialization block
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        motorSlide = hardwareMap.get(DcMotor.class, "motorArm2");

        servoClawRot = hardwareMap.get(Servo.class, "servo0");
        servoClaw1 = hardwareMap.get(Servo.class, "servo1");
        servoClaw2 = hardwareMap.get(Servo.class, "servo2");

        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);

        motorSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        servoClaw1.setPosition(0);
        servoClaw2.setPosition(0);
        servoClawRot.setPosition(0);

        timer = new ElapsedTime();
        // Initialize the FSM to the IDLE state
        currentState = State.IDLE;

        waitForStart();

        while (opModeIsActive()) {

            // Controlling the motors of the driveTrain
            rightMotor.setPower(gamepad1.left_stick_y);
            leftMotor.setPower(gamepad1.left_stick_y);

            rightMotor.setPower(+gamepad1.right_stick_x);
            leftMotor.setPower(-gamepad1.right_stick_x);

            rightMotor.setPower(gamepad2.left_stick_y /2);
            leftMotor.setPower(gamepad2.left_stick_y /2);

            rightMotor.setPower(gamepad2.right_stick_x /2);
            leftMotor.setPower(-gamepad2.right_stick_x /2);

            switch (currentState) {
                case IDLE:
                    // Wait for the 'A' button to be pressed to activate the slide and servo
                    if (gamepad1.a) {
                        motorSlide.setPower(1);
                        sleep(500);
                        motorArm.setPower(1);
                        servoClawRot.setPosition(1);
                        sleep(500);
                        servoClaw1.setPosition(1);
                        servoClaw2.setPosition(1);
                        timer.reset();
                        currentState = State.ACTIVE;
                    }
                    break;

                case ACTIVE:
                    if (timer.seconds() >= 4) {
                        motorSlide.setPower(-1);
                        sleep(500);
                        motorArm.setPower(-1);
                        sleep(500);
                        motorSlide.setPower(0);
                        motorArm.setPower(0);
                        currentState = State.COMPLETED;
                    }
                    break;

                case COMPLETED:
                    if (gamepad1.b) {
                        servoClawRot.setPosition(0);
                        sleep(500);
                        servoClaw1.setPosition(0);
                        servoClaw2.setPosition(0);
                        currentState = State.IDLE;
                    }
                    break;
                }
            }

            // Telemetry for debugging
            telemetry.addData("Current State", currentState.toString());
            telemetry.addData("Timer", timer.seconds());
            telemetry.update();

            sleep(1000);
        }
    }



