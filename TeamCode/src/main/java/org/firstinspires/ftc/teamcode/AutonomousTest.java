package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AutonomousTest")
public class AutonomousTest extends OpMode {

    private DistanceSensor distanceSensor;
    private NormalizedColorSensor colorSensor;
    private HardwareMap hwMap;
    @Override
    public void init() {
        hwMap = hardwareMap;
        distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
    }

    @Override
    public void loop() {
        telemetry.addData("Distance", String.format("%.2f", distanceSensor.getDistance(DistanceUnit.MM)));

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        telemetry.addLine("Color")
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
    }
}
