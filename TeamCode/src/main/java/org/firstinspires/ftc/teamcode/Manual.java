package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Manual")
// @Disabled
public class Manual extends OpMode {

    private ServoController servoController;
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
    @Override
    public void init() {
        // Get Control Hub ServoController
        servoController = hardwareMap.get(ServoController.class, "Control Hub");

        // Get Arm Motors
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");

        // Get Suspension Motor
        suspension = hardwareMap.get(DcMotor.class, "suspension");

        // Get servos
        claw = hardwareMap.get(Servo.class, "claw");
        angle = hardwareMap.get(Servo.class, "angle");

        // Plane launcher
        plane = hardwareMap.get(Servo.class, "plane");

        // Get Wheel Motors
        frontLeft0 = hardwareMap.get(DcMotor.class, "frontLeft0");
        backLeft1 = hardwareMap.get(DcMotor.class, "backLeft1");
        backRight2 = hardwareMap.get(DcMotor.class, "backRight2");
        frontRight3 = hardwareMap.get(DcMotor.class, "frontRight3");

        // Enable Control Hub Servos;
        servoController.pwmEnable();

        // Configure Max Angles for the servos that need it;
        claw.scaleRange(0, 0.5);
        angle.scaleRange(0, 0.55);

        // Set Servos to starting positions;
        claw.setPosition(0);
        plane.setPosition(1);

        // Configure DcMotor Directions correctly;
        // Wheels;
        frontLeft0.setDirection(DcMotor.Direction.REVERSE);
        backLeft1.setDirection(DcMotor.Direction.REVERSE);
        backRight2.setDirection(DcMotor.Direction.FORWARD);
        frontRight3.setDirection(DcMotor.Direction.FORWARD);
        // Arm;
        arm1.setDirection(DcMotor.Direction.FORWARD);
        arm2.setDirection(DcMotor.Direction.REVERSE);
        suspension.setDirection(DcMotor.Direction.REVERSE);

        // Configure Wheels to Brake when provided no Power;
        frontLeft0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // configure Arm brakes; to counteract Arm not being able to hold itself;
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Configure suspension Zero Power Behavior;
        suspension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Update on Driver Hub that Initialization finished;
        telemetry.addData("Initialized", "Finished with no errors");
    }

    @Override
    public void loop() {
        // Reverse stick value in order to flip forward and backward movements to be correct;
        double forward = -gamepad1.left_stick_y / 1.3;

        // Factor to counteract imperfect strafing;
        double turn = gamepad1.left_stick_x / 1.45;
        double strafe = gamepad1.right_stick_x / 1.3;

        if (gamepad1.right_trigger == 1) {
            forward /= 3;
            strafe /= 3;
            turn /= 3;
        }
        
        // Arm up/down speed;
        double arm = gamepad2.left_stick_y * 0.5;

        // Power for Arm Motors (-1 to 1);
        motorPower(forward, strafe, turn);

        // Change arm Lift speed;
        arm1.setPower(arm);
        arm2.setPower(arm);

        // Control Claws
        claw.setPosition(gamepad2.left_trigger / 2);

        // Claw Angle
        if (gamepad2.dpad_down) {
            angle.setPosition(angle.getPosition() + 0.1);
        } else if (gamepad2.dpad_up) {
            angle.setPosition(angle.getPosition() - 0.1);
        }

        if (gamepad2.a) {
            suspension.setPower(1);
        } else if (gamepad2.y) {
            suspension.setPower(-1);
        } else {
            suspension.setPower(0);
        }

        // Launch plane
        if (gamepad1.x) {
            plane.setPosition(0);
        }
    }

    @Override
    public void stop() {
        // Prepare Plane servo for reloading;
        plane.setPosition(1);
    }

    // Utility functions
    public void motorPower(final double forward, final double strafe, final double turn) {
        // Denominator is the largest motor power;
        double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(forward) + Math.abs(turn) + Math.abs(strafe), 1));

        // change motor speeds for wheels;
        frontLeft0.setPower((forward + turn + strafe) / denominator);
        backLeft1.setPower((forward + turn - strafe) / denominator);
        backRight2.setPower((forward - turn + strafe) / denominator);
        frontRight3.setPower((forward - turn - strafe) / denominator);
    }
}
