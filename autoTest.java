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
    private DistanceSensor distanceSensor;      // The Actual distance sensor on the ____
    private DistanceSensor distanceSensorB;     // The Actual distance sensor on the ____

    private DcMotor frontLeft0;
    private DcMotor frontRight3;
    private DcMotor backLeft1;
    private DcMotor backRight2;

    public State curState;

    private boolean timerStarted; // to reset the timer
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        distanceSensorB = hardwareMap.get(DistanceSensor.class, "distanceSensorB");

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

        telemetry.addData("Distance", String.format("%.2f", distanceSensor.getDistance(DistanceUnit.MM)));
        telemetry.addData("Distance", String.format("%.2f", distanceSensorB.getDistance(DistanceUnit.MM)));

        telemetry.addData("Timer2", timer.seconds());

        switch (curState)
        {
            case DRIVE:
                if (timerStarted)
                {
                    timer.reset();
                    timerStarted = false;
                }
                if (timer.seconds() < 0.94)
                {
                    frontLeft0.setPower(0.5);
                    backRight2.setPower(0.5);
                    frontRight3.setPower(0.5);
                    backLeft1.setPower(0.5);
                }
                else
                {
                    // 0.94 seconds passed - move to next state
                    curState = State.SEARCH;
                }
                telemetry.addData("curState:", curState);

                break;

            case SEARCH:
                telemetry.addData("Searching", "...");
                if (distanceSensor.getDistance(DistanceUnit.MM) < 300)
                {
                    telemetry.addData("Found it on", " Right side"); // NEEDS TO BE CHANGED DEPENDING ON THE SIDES!!!
                    SearchAction(State.TURNLEFT, 3); // the timer resets here
                }
                else if (distanceSensorB.getDistance(DistanceUnit.MM) < 300)
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
                if (timer.seconds() < 0.5)
                {
                    frontLeft0.setPower(0.5);
                    backRight2.setPower(-0.5);
                    frontRight3.setPower(-0.5);
                    backLeft1.setPower(0.5);
                }
                else
                {
                    telemetry.addData("Turned Left", "Stoping robot");
                    curState = State.PUT;
                }
                break;

            case TURNRIGHT:
                if (timer.seconds() < 0.5)
                {
                    frontLeft0.setPower(-0.5);
                    backRight2.setPower(0.5);
                    frontRight3.setPower(0.5);
                    backLeft1.setPower(-0.5);
                }
                else
                {
                    telemetry.addData("Turned Right", "Stoping robot");
                    curState = State.PUT;

                }
                break;
            case DONTTURN:
                if (timer.seconds() < 0.5)
                {
                    frontLeft0.setPower(-0.5);
                    backRight2.setPower(-0.5);
                    frontRight3.setPower(-0.5);
                    backLeft1.setPower(-0.5);
                }
                else
                {
                    telemetry.addData("Moved Backwords", "Stoping robot");
                    curState = State.PUT;
                }
                break;
            case PUT:
                frontLeft0.setPower(0);
                backRight2.setPower(0);
                frontRight3.setPower(0);
                backLeft1.setPower(0);
                break;

            default:
            {
                telemetry.addData("State not valid, somthing went wrong", String.valueOf(curState));
            }

        }
    }
    public enum State {
        DRIVE, SEARCH, TURNLEFT,TURNRIGHT,DONTTURN, PUT
    }

    public void SearchAction(State state, int value) {
        telemetry.addData("Found in:", String.valueOf(value));
        curState = state;
        timer.reset();
    }
}
