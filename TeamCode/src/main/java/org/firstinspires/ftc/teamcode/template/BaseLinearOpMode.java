package org.firstinspires.ftc.teamcode.template;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public abstract class BaseLinearOpMode extends LinearOpMode {
    // BaseOpMode for Avoiding Repetition and Boilerplate; (this was made too late to avoid the repetition)

    // Control Hub servo controller;
    private ServoController CH_ServoController = hardwareMap.get(ServoController.class, "Control Hub");

    // DcMotors Initialization;
    // Arm;
    public final DcMotor arm1           = hardwareMap.get(DcMotor.class, "arm1");
    public final DcMotor arm2           = hardwareMap.get(DcMotor.class, "arm2");
    // For lifting off the ground;
    public final DcMotor suspension     = hardwareMap.get(DcMotor.class, "suspension");
    // Spinning Claw;
    public final DcMotor armSpin        = hardwareMap.get(DcMotor.class, "armSpin");

    // Motors for wheels;
    public final DcMotor frontLeft0     = hardwareMap.get(DcMotor.class, "frontLeft0");
    public final DcMotor backLeft1      = hardwareMap.get(DcMotor.class, "backLeft1");
    public final DcMotor backRight2     = hardwareMap.get(DcMotor.class, "backRight2");
    public final DcMotor frontRight3    = hardwareMap.get(DcMotor.class, "frontRight3");

    // Servo initialization;
    public final Servo claw             = hardwareMap.get(Servo.class, "claw");
    public final Servo plane            = hardwareMap.get(Servo.class, "plane");
    public final Servo angle            = hardwareMap.get(Servo.class, "angle");
    public final DistanceSensor distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
    public final DistanceSensor distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");

    public final BaseLinearOpMode.MotorControl MotorController = new BaseLinearOpMode.MotorControl();

    // Utility Methods;
    private class MotorControl {
        public void wheelPower(final double frontLeft, final double backLeft, final double backRight, final double frontRight) {
            frontLeft0.setPower(frontLeft);
            backLeft1.setPower(backLeft);
            backRight2.setPower(backRight);
            frontRight3.setPower(frontRight);
        }

        public void wheelPower(final double forward, final double strafe, final double turn) {
            // Denominator is the largest motor power;
            double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(forward) + Math.abs(turn) + Math.abs(strafe), 1));

            // update wheel powers based on denominator and the inputs;
            frontLeft0.setPower((forward + turn + strafe) / denominator);
            backLeft1.setPower((forward + turn - strafe) / denominator);
            backRight2.setPower((forward - turn + strafe) / denominator);
            frontRight3.setPower((forward - turn - strafe) / denominator);

        }

        public void armPower(final double power) {
            arm1.setPower(power);
            arm2.setPower(power);
        }
    }



}
