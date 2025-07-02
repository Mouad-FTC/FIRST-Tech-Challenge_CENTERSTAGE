package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Angle_PID_Constants.PIDConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous( name = "AprilTag")
public class AprilTagCam extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    Servo servo0;
    Servo servo1;
    DcMotor motorArm;
    static  int MOTOR_COUNTS_PER_REV = 288;
    TouchSensor touchSensor;

    public static double COUNTS_PER_MOTOR_REV = 533;    // Encoder counts per revolution
    public static double WHEEL_DIAMETER_CM = 9.0;      // Wheel diameter in centimeters
    public static double COUNTS_PER_CM = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_CM * Math.PI);
    private static final double COUNTS_PER_REV = 1440; // This depends on the encoder type
    private static final double GEAR_RATIO = 0.08; // Adjust if using gears
    private static final double ARM_LENGTH_CM = 40.0; // Length of arm in centimeters


    double integralSum = 0;
    double Kd = PIDConstants.Kd;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    BHI260IMU imu;
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "webCam"))
                .setCameraResolution(new Size(640, 480))
                .build();

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");

        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(Servo.class, "servo1");

        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BHI260IMU.class, "IMU");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize();

        servo0.setPosition(0);
        servo1.setPosition(0);

        double desiredAngle = Math.toRadians(90);

        double power = PIDController(desiredAngle, imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {


            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData(" Tag id = ", tag.id);
                telemetry.addData(" Translation x", tag.ftcPose.x);
                telemetry.addData(" Translation y", tag.ftcPose.y);
                telemetry.addData(" Translation z", tag.ftcPose.z);
                telemetry.addData(" Rotation roll", tag.ftcPose.roll);
                telemetry.addData(" Rotation pitch", tag.ftcPose.pitch);
                telemetry.addData(" Rotation yaw", tag.ftcPose.yaw);

                telemetry.update();

                if (tag.id == 1) {

                    moveRobot(power / 8, -20, -20);
                    sleep(30000);

                }
                else if (tag.id == 2) {

                    moveRobot(power / 8, -20, -20);
                    sleep(30000);

                }
                 else if (tag.id == 3) {

                    moveRobot(power / 8, -20, -20);
                    sleep(30000);

                }

            }


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

    private void moveArmToAngle(double targetAngle) {
        // Convert angle to encoder counts
        double targetPosition = degreesToCounts(targetAngle);

        // Set target position and mode
        motorArm.setTargetPosition((int) targetPosition);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power and wait until arm reaches target position
        motorArm.setPower(0.5);
        // Adjust power as needed
        sleep(2000);
        // Stop the motor
        motorArm.setPower(0);
    }
    private double degreesToCounts(double degrees) {
        // Convert arm length to meters
        double armLengthM = ARM_LENGTH_CM / 100.0;

        // Calculate linear distance traveled by the arm tip
        double linearDistance = Math.toRadians(degrees) * armLengthM;

        // Calculate encoder counts for the linear distance
        double counts = (linearDistance / (2 * Math.PI)) * COUNTS_PER_REV * GEAR_RATIO;
        return counts;
    }

    public void stopMotors(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
