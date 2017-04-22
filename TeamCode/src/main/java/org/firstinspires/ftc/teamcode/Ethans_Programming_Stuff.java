package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by AlbrightIT on 10/4/2016.
 */

@Disabled
@Autonomous (name = "Ethans_Programming_Stuff", group = "DRSS")

public class Ethans_Programming_Stuff extends OpMode {

    //Declare variables here

    private DcMotor andymotor1;
    private Servo servo1;

    @Override
    public void init(){

        //Ran once when "init" is pressed on the robot controller.

        andymotor1 = hardwareMap.dcMotor.get("motor1");
        servo1 = hardwareMap.servo.get("servo1");
        //Setup your hardware (motors, Servos, and sensors)


    }

    @Override
    public void init_loop() {
       //Ran while the "init" mode is active.

        //Set servo position, or anything else that needs to be done in init.

    }

    @Override
    public void start(){
        //Ran once when played by the robot controller

        //Anything that needs to be done as soon as the robot starts up.//
    }

    @Override
    public void loop() {

        //Ran while the robot is running.

        //Anything you want to be able to do while drivign your robot.

        andymotor1.setPower(1.0);
        if (servo1.getPosition() == 1.0) {
            servo1.setPosition(-1.0);
        } else {
            servo1.setPosition(1.0);
        }
    }
    }
