package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Angle_PID_Constants.PIDConstants;

@TeleOp (name = "The TeleOp")
public class The_TeleOp extends LinearOpMode {
    // Declaring the motors
    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor motorArm;
    DcMotor motorArm1;
    Servo servo0;
    Servo servo1;
    Servo servo2;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initializing the actuators
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        motorArm1 = hardwareMap.get(DcMotor.class, "motorArm2");

        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");


        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArm.setDirection(DcMotorSimple.Direction.REVERSE);

        servo0.setPosition(0);
        servo1.setPosition(0);
        servo2.setPosition(0);

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
            // Controlling the arm Motor

            if (gamepad1.right_bumper) {
                motorArm.setPower(-0.5);
            }
            if (gamepad1.left_bumper) {
                motorArm.setPower(0.5);
            }

            else {
                motorArm.setPower(0);
            }

            if (gamepad2.right_bumper) {
                motorArm1.setPower(1);
            }
            if (gamepad2.left_bumper) {
                motorArm1.setPower(-1);
            }
            else {
                motorArm1.setPower(0);
            }
//             Controlling the servos of the hand and the claw
            if (gamepad1.a) {
                servo0.setPosition(1);
            }
            if (gamepad1.a) {
                servo1.setPosition(1);
            }
            if (gamepad1.a) {
                servo2.setPosition(1);
            }

            }

        }

    }