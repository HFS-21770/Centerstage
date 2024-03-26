package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Autonomous Close Red", preselectTeleOp = "Manual")

public class AutonomosCloseRed extends LinearOpMode
{
    /* Declare OpMode members. */
    private DcMotor frontLeft0;
    private DcMotor frontRight3;
    private DcMotor backLeft1;
    private DcMotor backRight2;

    private DistanceSensor distanceSensorRight;
    private DistanceSensor distanceSensorLeft;

    private ElapsedTime runtime = new ElapsedTime();
    private ServoController servoController;
    private DcMotor arm1;
    private DcMotor arm2;

    private Servo claw;
    private Servo angle;

    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: Core Hex HD Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 15.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 2.95275591;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415); // 446.86
    static final double DRIVE_SPEED = 0.5;
    static final double ARM_SPEED = 0.7;

    public State curState;
    @Override
    public void runOpMode() {
        // Get Motors and config them
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

        // reset servos
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

                    encoderDrive(DRIVE_SPEED, 10,10,10,10);
                    encoderDrive(DRIVE_SPEED, -4,4,-4,4);
                    encoderDrive(DRIVE_SPEED, 14, 14, 14,14); // E4
                    telemetry.addData("State","Moved");
                    curState = State.SEARCH;
                    break;

                case SEARCH:

                    telemetry.addData("State", "Searching");
                    if (distanceSensorRight.getDistance(DistanceUnit.MM) < 300) // SEARCH WHERE THE TEAM PROP IS
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

                    telemetry.addData("State","Turning Left");
                    encoderDrive(DRIVE_SPEED, 4, 4, 4,4);               // WALK A LITTLE BT TO TURN
                    encoderDrive(DRIVE_SPEED, -18, 18, 18,-18);         // TURN 90
                    encoderDrive(DRIVE_SPEED, 4.5, 4.5,4.5, 4.5);       // PUT PIXEL ON MARK
                    encoderDrive(DRIVE_SPEED, -4.5, -4.5, -4.5,-4.5);   // GO BACK
                    curState = State.PUT_LEFT;
                    break;

                case TURN_RIGHT:

                    telemetry.addData("State","Turning Right");
                    encoderDrive(DRIVE_SPEED, 4, 4, 4,4);               // WALK A LITTLE BT TO TURN
                    encoderDrive(DRIVE_SPEED, 18, -18,-18, 18);         // TURN 90
                    encoderDrive(DRIVE_SPEED, 4, 4, 4,4);               // PUT PIXEL ON MARK
                    encoderDrive(DRIVE_SPEED, -6, -6, -6,-6);           // GO BACK
                    curState = State.PUT_RIGHT;
                    break;

                case DONT_TURN:

                    telemetry.addData("State","Walking");
                    encoderDrive(DRIVE_SPEED, 5.75, 5.75, 5.75,5.75);   // PUT PIXEL ON MARK
                    encoderDrive(DRIVE_SPEED, -5, -5, -5,-5);           // GO BACK
                    curState = State.PUT_MIDDLE;
                    break;

                case PUT_LEFT:


                    encoderDrive(DRIVE_SPEED, -19, -19, -19,-19);       // E5
                    encoderDrive(DRIVE_SPEED, 35.5, -35.5, -35.5,35.5); // TURN 180
                    encoderDrive(DRIVE_SPEED,15,15,15,15);              // GO TO BACKDROP
                    encoderDrive(DRIVE_SPEED,-3,3,-3,3);                // LITTLE ADJUSTMENT
                    encoderArm(ARM_SPEED,100,100);                                                     // ARM UP
                    encoderDrive(DRIVE_SPEED,3,3,3,3);                  // MAKE CONTACT WITH THE BACKDROP
                    OpenClaw();                                                                                              // OPEN CLAW
                    CloseClaw();                                                                                             //CLOSE CLAW
                    encoderDrive(DRIVE_SPEED,-3,-3,-3,-3);              // BACK AWAY FROM BACKDROP
                    telemetry.addData("State","Done_Right");
                    curState = State.PARK_RIGHT;
                    break;

                case PUT_RIGHT:

                    encoderDrive(DRIVE_SPEED,-11,11,-11,11);                    // D4
                    encoderDrive(DRIVE_SPEED,27,27,27,27);                      // D5
                    encoderDrive(DRIVE_SPEED,13,-13,13,-13);                    // E5
                    encoderArm(ARM_SPEED,100,100);                                                             // ARM UP
                    encoderDrive(DRIVE_SPEED,10,10,10,10);                      // MAKE CONTACT WITH BOARD
                    OpenClaw();                                                                                                      // OPEN CLAW
                    CloseClaw();                                                                                                     // CLOSE CLAW
                    encoderDrive(DRIVE_SPEED,-3,-3,-3,-3);                      // BACK AWAY FROM BACKDROP
                    telemetry.addData("State","Done_Left");
                    curState = State.PARK_LEFT;
                    break;

                case PUT_MIDDLE:

                    encoderDrive(DRIVE_SPEED, 17.75, -17.75, -17.75,17.75);     // TURN 90
                    encoderDrive(DRIVE_SPEED,33,33,33,33);                      // E5 - E6
                    encoderDrive(DRIVE_SPEED,-2.75,2.75,-2.75,2.75);                        // MINOR ADJUSTMENTS
                    encoderArm(ARM_SPEED,100,100);                                                             // ARM UP
                    encoderDrive(DRIVE_SPEED,3,3,3,3);                          // MAKE CONTACT WITH BOARD
                    OpenClaw();                                                                                                      // OPEN CLAW
                    CloseClaw();                                                                                                     // CLOSE CLAW
                    encoderDrive(DRIVE_SPEED,-3,-3,-3,-3);                      // BACK AWAY FROM BACKDROP
                    telemetry.addData("State","Done_Middle");
                    curState = State.PARK_MIDDLE;
                    break;

                case PARK_LEFT:

                    encoderDrive(DRIVE_SPEED,20,-20,20,-20);                    // F5
                    encoderDrive(DRIVE_SPEED,10,10,10,10);                      // F6
                    curState = State.STOP;
                    break;

                case PARK_RIGHT:

                    encoderDrive(DRIVE_SPEED,35,-35,35,-35);                    // F5
                    encoderDrive(DRIVE_SPEED,10,10,10,10);                      // F6
                    curState = State.STOP;
                    break;

                case PARK_MIDDLE:

                    encoderDrive(DRIVE_SPEED,25,-25,25,-25);                    // F5
                    encoderDrive(DRIVE_SPEED,9,9,9,9);                          // F6
                    curState = State.STOP;
                    break;

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
    public void encoderArm(double speed, double arm1Position,double arm2Position){
        if(opModeIsActive())
        {
            arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            arm1.setTargetPosition((int)arm1Position);
            arm2.setTargetPosition((int)arm2Position);

            arm1.setPower(speed);
            arm2.setPower(speed);

            arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(opModeIsActive() && arm1.isBusy() && arm2.isBusy())
            {
                telemetry.addData("arm1","Moving to %7d",arm1.getCurrentPosition());
                telemetry.addData("arm2","Moving to %7d",arm2.getCurrentPosition());
                telemetry.update();
            }

            arm1.setPower(0);
            arm2.setPower(0);

            arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void CloseClaw()
    {
        angle.setPosition(0);
        runtime.reset();
        while(runtime.seconds() < 1)
        {
            telemetry.addData("WAIT","close");
        }
        claw.setPosition(0); // close the claw
    }
    public void OpenClaw()
    {
        angle.setPosition(0.4);
        runtime.reset();
        while(runtime.seconds() < 1)
        {
            telemetry.addData("WAIT","open");
        }
        claw.setPosition(0.75); // open the claw
    }

    public enum State
    {
        DRIVE, SEARCH, TURN_LEFT,TURN_RIGHT,DONT_TURN, PUT_LEFT,PUT_RIGHT,PUT_MIDDLE,PARK_LEFT,PARK_RIGHT,PARK_MIDDLE,STOP;
    }
}
