package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AutonomousTest")
// @Disabled // - Disable When Not Needed
public class AutonomousTest extends OpMode {

    private DistanceSensor distanceSensor;      // The Actual distance sensor
    private NormalizedColorSensor colorSensor;  // The Actual color sensor
    private TouchSensor touchSensor;            // The Actual touch sensor
    private boolean touchSensorState;           // Checking if touchSensor is Pressed
    @Override
    public void init() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");  
    }

    @Override
    public void loop() {
        telemetry.addData("Distance", String.format("%.2f", distanceSensor.getDistance(DistanceUnit.MM)));

        NormalizedRGBA colors = colorSensor.getNormalizedColors(); // the colors of colorSensor
        telemetry.addLine("Color")
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
    
        touchSensorState = touchSensor.isPressed();
        if(touchSensorState)
        {
            telemetry.addData("Touch Sensor:", "Is pressed");
        }
        else
        {
            telemetry.addData("Touch Sensor:", "Is NOT pressed");
        }
    }
}
