package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Calibration")
public class Calibration extends OpMode {

    private DcMotor suspension;
    private DcMotor arm1;
    private DcMotor arm2;

    @Override
    public void init() {
        suspension = hardwareMap.get(DcMotor.class, "suspension");
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        hardwareMap.get(Servo.class, "plane").setPosition(1);
        arm2.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        telemetry.addData("Calibration Controls", "");
        telemetry.addData("GPad 1/2 Dpad Up/Down", "spin suspension Motor in certain direction");
        telemetry.addData("GPad 1/2 Right Stick Up/Down", "Move Arm Up/Down");

        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            suspension.setPower(0.1);
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            suspension.setPower(-0.1);
        } else {
            suspension.setPower(0);
        }

        double arm = gamepad2.right_stick_y;
        double power = (arm != 0) ? 0.5 * arm : 0;

        arm1.setPower(power);
        arm2.setPower(power);
    }
}
