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
@Autonomous(name = "AutonomousTest")
public class AutonomousTest extends OpMode {
    private DistanceSensor distanceSensor;      // The Actual distance sensor on the Right Side
    private DistanceSensor distanceSensorB;     // The Actual distance sensor on the Left Side

    private DcMotor frontLeft0;
    private DcMotor frontRight3;
    private DcMotor backLeft1;
    private DcMotor backRight2;

    public State curState;

    private boolean timerStarted; // to reset the timer
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init()
    {
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");

        frontLeft0 = hardwareMap.get(DcMotor.class, "frontLeft0");
        frontRight3 = hardwareMap.get(DcMotor.class, "frontRight3");
        backLeft1 = hardwareMap.get(DcMotor.class, "backLeft1");
        backRight2 = hardwareMap.get(DcMotor.class, "backRight2");

        frontLeft0.setDirection(DcMotor.Direction.REVERSE);
        backLeft1.setDirection(DcMotor.Direction.REVERSE);
        backRight2.setDirection(DcMotor.Direction.FORWARD);
        frontRight3.setDirection(DcMotor.Direction.FORWARD);

        timer.reset();
        timerStarted = true;

        curState = State.DRIVE;

        telemetry.addData("Initialized", "Finished with no errors");
    }

    @Override
    public void loop()
    {
telemetry.addData("Distance", String.format("%.2f", distanceSensorRight.getDistance(DistanceUnit.MM)));
        telemetry.addData("Distance", String.format("%.2f", distanceSensorLeft.getDistance(DistanceUnit.MM)));

        telemetry.addData("Timer2", timer.seconds());

        switch (curState)
        {
            case DRIVE:
                if(timerStarted)
                {
                    // resetting the timer
                    timer.reset();
                    timerStarted = false;
                }
                if(timer.seconds() < 1.2)
                {
                    MotorPower(0.4 ,0.4 ,0.4 ,0.4);
                }
                else
                {
                    // 0.94 seconds passed - move to next stat
                    curState = State.SEARCH;
                }
                telemetry.addData("curState:",curState);

                break;

            case SEARCH:
                MotorPower(0,0,0,0);
                telemetry.addData("Searching","...");

                if (distanceSensorRight.getDistance(DistanceUnit.MM) < 300)
                {
                    telemetry.addData("Found it on", " Right side"); // NEEDS TO BE CHANGED DEPENDING ON THE SIDES!!!
                    SearchAction(State.TURNLEFT, 3); // the timer resets here
                }
                else if (distanceSensorLeft.getDistance(DistanceUnit.MM) < 300)
                {
                    telemetry.addData("Found it on", " Left side"); // NEEDS TO BE CHANGED DEPENDING ON THE SIDES!!!
                    SearchAction(State.TURNRIGHT, 9); // the timer resets here
                }
                else
                {
                    telemetry.addData("Found it on", " upfront");
                    SearchAction(State.DONTTURN, 12); // the timer resets here
                }
                break;

            case TURNLEFT:
                if(timer.seconds() < 0.6)
                {
                    MotorPower(0.5,-0.5,-0.5,0.5);
                }
                else
                {
                    telemetry.addData("Turned Left", "Stopping robot");
                    timer.reset();
                    curState = State.PUT;
                }
                break;

            case TURNRIGHT:
                if(timer.seconds() < 0.6)
                {
                    MotorPower(-0.5,0.5,0.5,-0.5);
                }
                else
                {
                    telemetry.addData("Turned Right", "Stopping robot");
                    timer.reset();
                    curState = State.PUT;
                }
                break;
            case DONTTURN:
                if(timer.seconds() < 0.5)
                {
                    MotorPower(0.5,0.5,0.5,0.5);
                }
                if(timer.seconds() < 0.9)
                {
                    MotorPower(-0.5,-0.5,-0.5,-0.5);
                }
                else
                {
                    telemetry.addData("Moved Backwards", "Stopping robot");
                    timer.reset();
                    curState = State.PUT;
                }
                break;
            case PUT:
                if(timer.seconds() < 0.1)
                {
                    MotorPower(0.5,0.5,0.5,0.5);
                }
                else
                {
                    MotorPower(0,0,0,0);
                }
                break;

            default:
                telemetry.addData("State not valid, something went wrong", String.valueOf(curState));
        } 
    }
    public enum State
    {
        DRIVE, SEARCH, TURNLEFT,TURNRIGHT,DONTTURN, PUT
    }
    public void SearchAction(State state, int value) {
        telemetry.addData("Found In:", String.valueOf(value));
        curState = state;
        timer.reset();
    }
    public void MotorPower(double p1, double p2, double p3,double p4)
    {
        frontLeft0.setPower(p1);
        backRight2.setPower(p2);
        frontRight3.setPower(p3);
        backLeft1.setPower(p4);
    }
}
