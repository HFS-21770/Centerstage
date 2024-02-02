package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AutonomousTest")
@Disabled // - Disable When Not Needed
public class AutonomousTest extends OpMode {

    private DistanceSensor distanceSensor;      // The Actual distance sensor
    private NormalizedColorSensor colorSensor;  // The Actual color sensor
    private TouchSensor touchSensor;            // The Actual touch sensor
    private DcMotor armAngle;                   // DcMotor for spining the claw
    private final int i = 50;                   // number of times the armAngle spins


    @Override
    public void init() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        armAngle = hardwareMap.get(DcMotor.class, "armAngle");
        armAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        telemetry.addData("Distance", String.format("%.2f", distanceSensor.getDistance(DistanceUnit.MM)));

        NormalizedRGBA colors = colorSensor.getNormalizedColors(); // the colors of colorSensor
        telemetry.addLine("Color")
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
    
            telemetry.addData("Touch Sensor", touchSensor.isPressed() ? "Is pressed" : "Is NOT pressed");
        
        int amount = 0;
        while(amount <= i)
        {
            armAngle.setPower(1);
            amount++;
        }
}
