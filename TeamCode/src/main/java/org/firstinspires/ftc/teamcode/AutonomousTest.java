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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.Convert;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Autonomous(name = "Auto test 4.2.24")
public class autoTest extends OpMode{
    private DistanceSensor distanceSensor;      // The Actual distance sensor
    private DistanceSensor distanceSensorB;
    private NormalizedColorSensor colorSensor;  // The Actual color sensor
    private TouchSensor touchSensor;            // The Actual touch sensor
    private DcMotor armAngle;                   // DcMotor for spining the claw
    private int i = 0;                   // number of times the armAngle spins
    private DcMotor frontLeft0;
    private DcMotor frontRight3;
    private DcMotor backLeft1;
    private DcMotor backRight2;
    private double h = 0;
    private double r = 0;
    private double g = 0;
    private double b = 0;
    private boolean timerStarted; // to reset the timer
    private boolean walk; // if the robot walked a tile to detect the cube
    private boolean detect; // if the robot detected the cube
    private boolean see;
    public State curState;

    private ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        distanceSensorB = hardwareMap.get(DistanceSensor.class, "distanceSensorB");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
//        armAngle = hardwareMap.get(DcMotor.class, "armAngle");
//        armAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft0 = hardwareMap.get(DcMotor.class, "frontLeft0");
        frontRight3 = hardwareMap.get(DcMotor.class, "frontRight3");
        backLeft1 = hardwareMap.get(DcMotor.class, "backLeft1");
        backRight2 = hardwareMap.get(DcMotor.class, "backRight2");

        frontLeft0.setDirection(DcMotor.Direction.REVERSE);
        backLeft1.setDirection(DcMotor.Direction.REVERSE);
        backRight2.setDirection(DcMotor.Direction.FORWARD);
        frontRight3.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Initialized", "Finished with no errors");

        timer.reset();

        timerStarted = true;
        walk = false;
        detect = true;
        see = false;
        curState = State.DRIVE;
    }

    @Override
    public void loop()
    {

        telemetry.addData("Distance", String.format("%.2f", distanceSensor.getDistance(DistanceUnit.MM)));
        telemetry.addData("Distance", String.format("%.2f", distanceSensorB.getDistance(DistanceUnit.MM)));


        NormalizedRGBA colors = colorSensor.getNormalizedColors(); // the colors of colorSensor
        telemetry.addLine("Color")
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);

        telemetry.addData("Touch Sensor", touchSensor.isPressed() ? "Is pressed" : "Is NOT pressed");

        telemetry.addData("Timer2", timer.seconds());


        switch (curState){
            case DRIVE:
                if(timerStarted)
                {
                    timer.reset();
                    timerStarted = false;
                }
                if(timer.seconds() < 2)
                {
                    frontLeft0.setPower(0.5);
                    backRight2.setPower(0.5);
                    frontRight3.setPower(0.5);
                    backLeft1.setPower(0.5);
                }
                curState = State.SEARCH;
                telemetry.addData("curState:",curState);
                break;


            case SEARCH:
                if(distanceSensor.getDistance(DistanceUnit.MM) < 300)
                {
                    SearchAction(State.TURNLEFT, 1);
                }
                else if (distanceSensorB.getDistance(DistanceUnit.MM) < 300)
                {
                    SearchAction(State.TURNRIGHT, 2);
                }
                else if (distanceSensorB.getDistance(DistanceUnit.MM) > 300 && distanceSensor.getDistance(DistanceUnit.MM) > 300)
                {
                    SearchAction(State.DONTTURN, 3);
                }
                else
                {
                    telemetry.addData("State","ERROR!!!!!");
                }
                break;

            case TURNLEFT:
                if(timer.seconds() < 0.5)
                {
                    frontLeft0.setPower(0.5);
                    backRight2.setPower(-0.5);
                    frontRight3.setPower(-0.5);
                    backLeft1.setPower(0.5);
                }
                curState = State.PUT;
                break;

            case TURNRIGHT:
                if(timer.seconds() < 0.5){
                    frontLeft0.setPower(-0.5);
                    backRight2.setPower(0.5);
                    frontRight3.setPower(0.5);
                    backLeft1.setPower(-0.5);
                }
                else{
                    curState = State.PUT;

                }
                break;
            case DONTTURN:
                if(timer.seconds() < 0.5){
                    frontLeft0.setPower(0.5);
                    backRight2.setPower(0.5);
                    frontRight3.setPower(0.5);
                    backLeft1.setPower(0.5);
                }
                else {
                    curState = State.PUT;
                }
                break;
            case PUT:
                frontLeft0.setPower(0);
                backRight2.setPower(0);
                frontRight3.setPower(0);
                backLeft1.setPower(0);
        }


//        if(!detect && distanceSensor.getDistance(DistanceUnit.MM) < 300)
//        {
//            telemetry.addData("Cube:","1");
//            frontLeft0.setPower(0.5);
//            backRight2.setPower(-0.5);
//            frontRight3.setPower(-0.5);
//            backLeft1.setPower(0.5);
//            detect = true;
//            see = true;
//        }
//        else if(!detect && distanceSensorB.getDistance(DistanceUnit.MM) < 300)
//        {
//            telemetry.addData("Cube:","2");
//            frontLeft0.setPower(-0.5);
//            backRight2.setPower(0.5);
//            frontRight3.setPower(0.5);
//            backLeft1.setPower(-0.5);
//            see = true;
//            detect = true;
//
//        }
//        else if(!detect && distanceSensorB.getDistance(DistanceUnit.MM) > 300 && distanceSensor.getDistance(DistanceUnit.MM) > 300)
//        {
//            telemetry.addData("Cube:","3");
//            frontLeft0.setPower(0);
//            backRight2.setPower(0);
//            frontRight3.setPower(0);
//            backLeft1.setPower(0);
//            see = true;
//            detect = true;
//        }
    }
    public enum State {
        DRIVE, SEARCH, TURNLEFT,TURNRIGHT,DONTTURN, PUT
    }

    public void SearchAction(State state, int value) {
        telemetry.addData("Cube:", String.valueOf(value));
        curState = state;
        timer.reset();
    }
}
