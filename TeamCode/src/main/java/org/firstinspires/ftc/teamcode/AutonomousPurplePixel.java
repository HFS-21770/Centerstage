package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// NoamLandau21770's Code
@Autonomous(name = "Autonomous Purple Pixel", preselectTeleOp = "Manual")
public class AutonomousPurplePixel extends LinearOpMode {
    /* Declare OpMode members. */
    private DcMotor frontLeft0;
    private DcMotor frontRight3;
    private DcMotor backLeft1;
    private DcMotor backRight2;

    private DistanceSensor distanceSensorRight;      // The Actual distance sensor on the RIGHT SIDE
    private DistanceSensor distanceSensorLeft;     // The Actual distance sensor on the LEFT SIDE

    private ElapsedTime runtime = new ElapsedTime();
    private ServoController servoController;
    private DcMotor arm1;
    private DcMotor arm2;
    private Servo claw;
    private Servo angle;

    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: Core Hex HD Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 15.0;     // 3 * 5 = 15
    static final double WHEEL_DIAMETER_INCHES = 2.95275591;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415); // 446.86
    static final double DRIVE_SPEED = 0.5;

    public State curState;

    @Override
    public void runOpMode() throws InterruptedException
    {
        frontLeft0 = hardwareMap.get(DcMotorEx.class, "frontLeft0");
        frontRight3 = hardwareMap.get(DcMotorEx.class, "frontRight3");
        backLeft1 = hardwareMap.get(DcMotorEx.class, "backLeft1");
        backRight2 = hardwareMap.get(DcMotorEx.class, "backRight2");

        frontLeft0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontLeft0.setDirection(DcMotor.Direction.REVERSE);
        backLeft1.setDirection(DcMotor.Direction.REVERSE);
        backRight2.setDirection(DcMotor.Direction.FORWARD);
        frontRight3.setDirection(DcMotor.Direction.FORWARD);

        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");

        curState = State.DRIVE;

        // Get Control Hub ServoController
        servoController = hardwareMap.get(ServoController.class, "Control Hub");

        // Get Arm Motors
        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
        arm2 = hardwareMap.get(DcMotorEx.class, "arm2");

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1.setDirection(DcMotor.Direction.REVERSE);
        arm2.setDirection(DcMotor.Direction.FORWARD);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Get servos
        claw = hardwareMap.get(Servo.class, "claw");
        angle = hardwareMap.get(Servo.class, "angle");

        servoController.pwmEnable();

        claw.scaleRange(0, 0.5);
        angle.scaleRange(0, 1);

        claw.setPosition(0);
        angle.setPosition(0);
        waitForStart();
        while(opModeIsActive())
        {
            switch (curState)
            {
                case DRIVE:

                    encoderDrive(DRIVE_SPEED, 24, 24, 24,24); // 6.
                    telemetry.addData("State","Moved");
                    curState = State.SEARCH;
                    break;

                case SEARCH:

                    telemetry.addData("State", "Searching");
                    if (distanceSensorRight.getDistance(DistanceUnit.MM) < 300)
                    {
                        telemetry.addData("State", "Found it on Right side");
                        curState = State.TURN_RIGHT;
                    }
                    else if (distanceSensorLeft.getDistance(DistanceUnit.MM) < 300)
                    {
                        telemetry.addData("State", "Found it on Left side");
                        curState = State.TURN_LEFT;
                    }
                    else
                    {
                        telemetry.addData("State", "Found it upfront");
                        curState = State.DONT_TURN;
                    }
                    break;

                case TURN_LEFT:
                    //-----------------------------------------------------------------------------------------------------
                    telemetry.addData("State","Turning Left");
                    encoderDrive(DRIVE_SPEED, 9, 9, 9,9);
                    encoderDrive(DRIVE_SPEED, -18, 18, 18,-18);
                    curState = State.PUT_LEFT;
                    break;
                //-----------------------------------------------------------------------------------------------------
                case TURN_RIGHT:
                    //-----------------------------------------------------------------------------------------------------
                    telemetry.addData("State","Turning Right");
                    encoderDrive(DRIVE_SPEED, 9, 9, 9,9);
                    encoderDrive(DRIVE_SPEED, 18, -18,-18, 18);
                    curState = State.PUT_RIGHT;
                    break;
                //-----------------------------------------------------------------------------------------------------
                case DONT_TURN:
                    //-----------------------------------------------------------------------------------------------------
                    telemetry.addData("State","Walking");
                    curState = State.PUT_MIDDLE;
                    break;
                //-----------------------------------------------------------------------------------------------------
                case STOP:
                    telemetry.addData("State","STOPPING");
                default:
                    telemetry.addData("State not valid, something went wrong", String.valueOf(curState));
            }
        }

    }

    public void encoderDrive(double speed, double frontLeftInches, double frontRightInches,double backRightInches,double backLeftInches) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive())
        {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = (int)(frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget =  (int)(frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = (int)(backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = (int)(backRightInches * COUNTS_PER_INCH);

            telemetry.addData("Running to:", newFrontLeftTarget);

            frontLeft0.setTargetPosition(newFrontLeftTarget);
            backLeft1.setTargetPosition(newBackLeftTarget);
            frontRight3.setTargetPosition(newFrontRightTarget);
            backRight2.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft0.setPower(Math.abs(speed));
            backLeft1.setPower(Math.abs(speed));
            frontRight3.setPower(Math.abs(speed));
            backRight2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (frontLeft0.isBusy() && backLeft1.isBusy() && frontRight3.isBusy() && backRight2.isBusy())) // runtime.seconds() < timeoutS
            {
                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newFrontLeftTarget,  newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        frontLeft0.getCurrentPosition(), backRight2.getCurrentPosition());
                telemetry.addData("Sensor",distanceSensorLeft.getDistance(DistanceUnit.MM));
                telemetry.update();
            }

            // Stop all motion;
            frontLeft0.setPower(0);
            backLeft1.setPower(0);
            frontRight3.setPower(0);
            backRight2.setPower(0);

            // Reset Encoder
            frontLeft0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            frontLeft0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public enum State
    {
        DRIVE, SEARCH, TURN_LEFT,TURN_RIGHT,DONT_TURN, PUT_LEFT,PUT_RIGHT,PUT_MIDDLE,STOP;
    }
}