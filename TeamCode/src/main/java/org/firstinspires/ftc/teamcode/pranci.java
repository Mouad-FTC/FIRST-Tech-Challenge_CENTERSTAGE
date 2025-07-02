package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Angle_PID_Constants.PIDConstants;


@Autonomous( name = "pranci")
public class pranci extends LinearOpMode {

    //Declaring the actuators
    DcMotor leftMotor;
    DcMotor rightMotor;
    Servo servo0;
    Servo servo1;
    DcMotor motorArm;
    double integralSum = 0;

    //Declaring the PID constants
    double Kd = PIDConstants.Kd;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    private double lastError = 0;

    //Declaring the timer
    ElapsedTime timer = new ElapsedTime();
    BHI260IMU imu;
    static  int MOTOR_COUNTS_PER_REV = 288;
    public static double COUNTS_PER_MOTOR_REV = 533;    // Encoder counts per revolution
    public static double WHEEL_DIAMETER_CM = 9.0;      // Wheel diameter in centimeters
    public static double COUNTS_PER_CM = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_CM * Math.PI);

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialization block
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");

        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(Servo.class, "servo1");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        imu =  hardwareMap.get(BHI260IMU.class, "IMU");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize();

        servo0.setPosition(0.5);
        servo1.setPosition(0.3);


        double desiredAngle = Math.toRadians(90);

        waitForStart();
        while(opModeIsActive()){

            telemetry.addData("Target IMU angle", desiredAngle);
            telemetry.addData("Current IMU angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            double power = PIDController( desiredAngle, imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);

            moveRobot(power/9, 76, 76);
            sleep(30000);
            stopMotors();

            telemetry.update();

        }
    }

    public double PIDController (double reference , double state){

        double error = angleWrap(reference - state);
        integralSum = error * timer.seconds();
        double derivative = (error - lastError)/timer.seconds();
        timer.reset();
        double output = (error * Kp) + (integralSum * Ki) + (derivative * Kd);
        return output;

    }

    public double angleWrap (double radians){

        while( radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while( radians < Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;

    }

    public void moveRobot (double speed, double leftCM, double rightCM) {
        int newLeftTarget;
        int newRightTarget;

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftCM * COUNTS_PER_CM);
        newRightTarget = rightMotor.getCurrentPosition() + (int)(rightCM * COUNTS_PER_CM);
        leftMotor.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time and start motion.
        leftMotor.setPower(Math.abs(speed/2));
        rightMotor.setPower(Math.abs(speed/2));

    }
    public int degreesToRotateToCounts(int degrees) {
        return (int) (degrees * (MOTOR_COUNTS_PER_REV / 360.0));
    }

    public void stopMotors(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
