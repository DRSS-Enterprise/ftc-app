package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.text.SimpleDateFormat;
import java.util.Date;

@Autonomous(name = "Concept:DRSS TestBoard", group = "DRSS")

/**
 * Created by AlbrightIT on 10/2/2016.
 */

public class DRSS_TestBoard extends OpMode {

    private DcMotor myMotor1 = null;
    private DcMotor myMotor2 = null;
    private DcMotorController topController = null;
    private DcMotorController bottomController = null;
    private ServoController servoCtrl = null;
    private DeviceInterfaceModule devIntMod = null;
    private Servo myServo = null;
    //private ColorSensor myColorSensor = null;
    private ModernRoboticsI2cColorSensor myColorSensor = null;
    private ModernRoboticsAnalogOpticalDistanceSensor myOpticalSensor = null;
    private ModernRoboticsI2cRangeSensor myRangeSensor = null;
    private ModernRoboticsI2cGyro myGyro = null;

    private ElapsedTime runtime = new ElapsedTime();

    int xVal, yVal, zVal = 0;     // Gyro rate Values
    int heading = 0;              // Gyro integrated heading
    int angleZ = 0;
    int red, green, blue, alpha = 0;

    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

            try {
            topController = hardwareMap.dcMotorController.get("MotorControllerTop");
        } catch (Exception eControllerTop) {
            topController = null;
        }

        try {
            myMotor1 = hardwareMap.dcMotor.get("motor1");
        } catch (Exception eTopMotor) {
            myMotor1 = null;
        }

        try {
            bottomController = hardwareMap.dcMotorController.get("MotorControllerBottom");
        } catch (Exception eControllerBottom) {
            bottomController = null;
        }

        try {
            myMotor2 = hardwareMap.dcMotor.get("motor2");
        } catch (Exception eBottomMotor) {
            myMotor2 = null;
        }

        try {
            servoCtrl = hardwareMap.servoController.get("ServoController");
        } catch (Exception eServoCtrl) {
            servoCtrl = null;
        }

        try {
            myServo = hardwareMap.servo.get("servo1");
        } catch (Exception eServo) {
            myServo = null;
        }

        try {
           devIntMod = hardwareMap.deviceInterfaceModule.get("DeviceInterfaceModule");
        } catch (Exception eDevIntMod) {
            devIntMod = null;
        }

        try {
            myColorSensor = (ModernRoboticsI2cColorSensor)hardwareMap.colorSensor.get("ColorSensor");
        } catch (Exception eColorSensor) {
            myColorSensor = null;
        }

        try {
            myGyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("GyroSensor");
        } catch (Exception eGyroSensor) {
            myGyro = null;
        }

        try {
            myRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeSensor"); //There is no harwareMap.ModernRoboticsI2cRangeSensor
        } catch (Exception eRangeSensor) {
            myRangeSensor = null;
        }

        try {
            myOpticalSensor = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "OpticalSensor");
        } catch (Exception eOpticalSensor) {
            myOpticalSensor = null;
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();
    }

    /*
       * Code to run when the op mode is first enabled goes here
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
       */
    @Override
    public void init_loop() {
        myColorSensor.enableLed(true);
    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());


       if (myMotor1 != null) {
            myMotor1.setPower(1.0);
        }
        if (myMotor2 != null) {
            myMotor2.setPower(1.0);  
        }

        if (myColorSensor != null) {
            myColorSensor.enableLed(false);
            telemetry.addData("Flag 1", 1);
        }

        // get the x, y, and z values (rate of change of angle).
        xVal = myGyro.rawX();
        yVal = myGyro.rawY();
        zVal = myGyro.rawZ();

        /*
        red = myColorSensor.red();
        blue = myColorSensor.blue();
        green = myColorSensor.green();
        alpha = myColorSensor.alpha();
        */


        // get the heading info.
        // the Modern Robotics' gyro sensor keeps
        // track of the current heading for the Z axis only.
        heading = myGyro.getHeading();
        angleZ  = myGyro.getIntegratedZValue();

        if(gamepad1.a) {
            myServo.setPosition(1.0);
        } else {
            myServo.setPosition(-1.0);
        }



//        myColorSensor.enableLed(true);

        telemetry.addData("0", "Heading %03d", heading);
        telemetry.addData("1", "Int. Ang. %03d", angleZ);
        telemetry.addData("2", "X av. %03d", xVal);
        telemetry.addData("3", "Y av. %03d", yVal);
        telemetry.addData("4", "Z av. %03d", zVal);
        telemetry.addData("5", "Red %03d", red);
        telemetry.addData("6", "Blue %03d", blue);
        telemetry.addData("7", "Green %03d", green);
        telemetry.addData("8", "Alpha %03d", alpha);
        telemetry.addData("9", "Raw ultrasonic distance %03d", myRangeSensor.rawUltrasonic());
        telemetry.addData("10", "Raw optical distance %03d", myRangeSensor.rawOptical());
        telemetry.addData("11", "Light level detected %03f", myOpticalSensor.getLightDetected());

    }

    public void pause (int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
            }
    }
}
