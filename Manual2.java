package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "Mecanum Shlomi")
public class Manual2 extends LinearOpMode {

    private ServoController ServoController;
    private DcMotor arm1;
    private DcMotor arm2;
    private Servo claw;
    private Servo plane;
    private Servo angle;
    private DcMotor frontLeft0;
    private DcMotor frontRight3;
    private DcMotor backLeft1;
    private DcMotor backRight2;
    private DcMotor suspension;
    private DcMotor armSpin;

    private boolean angleChange;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Get Control Hub ServoController
        ServoController = hardwareMap.get(ServoController.class, "Control Hub");

        // Get Arm Motors
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");


        // Get servos
        claw = hardwareMap.get(Servo.class, "claw");
        angle = hardwareMap.get(Servo.class, "angle");

        // Plane launcher
        plane = hardwareMap.get(Servo.class, "plane");

        // Get Wheel Motors
        frontLeft0 = hardwareMap.get(DcMotor.class, "frontLeft0");
        frontRight3 = hardwareMap.get(DcMotor.class, "frontRight3");
        backLeft1 = hardwareMap.get(DcMotor.class, "backLeft1");
        backRight2 = hardwareMap.get(DcMotor.class, "backRight2");

        // Enable Control Hub Servos;
        ServoController.pwmEnable();

        // Set Servos to starting positions;
        claw.setPosition(0);
//        angle.setPosition(0);
        plane.setPosition(1);

        // Configure Max Angles for the servos that need it;
        claw.scaleRange(0, 0.5);

        angle.scaleRange(0.2, 0.8);

        // Configure DcMotor Directions correctly;
        // Wheels;
        frontLeft0.setDirection(DcMotor.Direction.REVERSE);
        backLeft1.setDirection(DcMotor.Direction.REVERSE);
        backRight2.setDirection(DcMotor.Direction.FORWARD);
        frontRight3.setDirection(DcMotor.Direction.FORWARD);

        // configure Arm brakes; to counteract Arm not being able to hold itself;
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setDirection(DcMotor.Direction.REVERSE);

        // rope Motor
        suspension = hardwareMap.get(DcMotor.class, "suspension");
        suspension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armSpin = hardwareMap.get(DcMotor.class,"armSpin");
        armSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Update on Driver Hub that Initialization finished;
        telemetry.addData("Initialized", "Finished with no errors");

        waitForStart();
        //------------------------------------------------------------




        while (opModeIsActive())
        {
            // Reverse stick value to fix forward/backward movements;
            double forward = -gamepad1.left_stick_y / 1.5;



            // Factor to counteract imperfect strafing;
            double turn = gamepad1.left_stick_x * 1.1 / 2.5;
            double strafe = gamepad1.right_stick_x / 2;

            // Arm speed;
            double arm = gamepad2.left_stick_y;

            // Power for Arm Motors (-1 to 1);

            // ternary conditional operator;
            // example: true ? "this" : "not this"; || false ? "not this" : "this";
            // syntax: condition ? true value/code : false value/code;
            // better: (condition) ? true value/code : false value/code;
            // Should only be used for small things
            double power = (arm != 0) ? 0.5 * arm : 0;

            motorPower(forward, strafe, turn);

            // Change arm Lift speed;
            arm1.setPower(power);
            arm2.setPower(power);

            // Control Claw
            if (gamepad2.left_trigger != claw.getPosition()) {
                claw.setPosition(gamepad2.left_trigger / 2);
            }


            if(gamepad2.left_bumper)
            {
                armSpin.setTargetPosition(144);
                armSpin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSpin.setPower(0.3);
                telemetry.addData("SPIN: ", armSpin.getCurrentPosition());
            } else if (gamepad2.right_bumper) {

                armSpin.setTargetPosition(0);
                armSpin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSpin.setPower(-0.3);
                telemetry.addData("SPIN: ", armSpin.getCurrentPosition());
            }


            // Claw Angle
            if (gamepad2.dpad_down)
            {
                angle.setPosition(angle.getPosition() + 0.1);
            }
            else if (gamepad2.dpad_up)
            {
                angle.setPosition(angle.getPosition() - 0.1);
            }

            if(gamepad2.a)
            {
                suspension.setPower(1);
            }
            else if (gamepad2.y)
            {
                suspension.setPower(-1);
            }
            else{
                suspension.setPower(0);
            }

            //Launch plane
            if (gamepad1.x){
                plane.setPosition(0);
            }



        }
        plane.setPosition(1);
    }

    // Utility functions
    public void motorPower(final double forward, final double strafe, final double turn) {
        // Denominator is the largest motor power;
        double denominator =JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(forward) + Math.abs(turn) + Math.abs(strafe), 1));

        // change motor speeds for wheels;
        frontLeft0.setPower((forward + turn + strafe) / denominator);
        backLeft1.setPower((forward + turn - strafe) / denominator);
        backRight2.setPower((forward - turn + strafe) / denominator);
        frontRight3.setPower((forward - turn - strafe) / denominator);
    }
}
