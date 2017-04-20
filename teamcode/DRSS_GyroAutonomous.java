
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "DRSS GyroAutonomous", group = "DRSS")

public class DRSS_GyroAutonomous extends LinearOpMode{

    private DcMotor myMotor1 = null;
    private DcMotor myMotor2 = null;
    private DcMotorController topController = null;
    private DcMotorController bottomController = null;
    private ServoController servoCtrl = null;
    private DeviceInterfaceModule devIntMod = null;
    private Servo myServo = null;
    // private ColorSensor myColorSensor = null;
    private ModernRoboticsI2cColorSensor myColorSensor = null;
    private ModernRoboticsAnalogOpticalDistanceSensor myOpticalSensor = null;
    private ModernRoboticsI2cRangeSensor myRangeSensor = null;
    private ModernRoboticsI2cGyro myGyro = null;

    private ElapsedTime runtime = new ElapsedTime();

    private boolean isFinished = false;

    int xVal, yVal, zVal = 0;     // Gyro rate Values
    int heading;              // Gyro integrated heading
    int lastHeading;
    int angleZ = 0;
    int red, green, blue, alpha = 0;
    int lTarget;
    int rTarget;
    int target;
    int lPosition;
    int rPosition;
    int currentStep;

    double error;

    public void runOpMode() throws InterruptedException {


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
            myColorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "ColorSensor");
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

        currentStep = 1;

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");

        telemetry.addData("Achieved", 0);

        telemetry.update();

        waitForStart();

        target = 42000;

        while(!isFinished){
            telemetry.addData("Achieved", 2);
            telemetry.update();
            heading = myGyro.getHeading();
            heading--;
            if(currentStep == 1){
                driveStraight(target);
                telemetry.addData("Motor1", myMotor1.getCurrentPosition());
                telemetry.addData("Motor2", myMotor2.getCurrentPosition());
                telemetry.addData("current positon", currentStep);
                telemetry.update();
                
            } else if(currentStep == 2){
                myMotor1.setPower(0.0);
                myMotor2.setPower(0.0);
                
            }

        }


    }

    private void driveStraight(int target){

        int targetAngle = heading;
        error = getError(targetAngle);

        while(currentStep == 1){
            heading = myGyro.getHeading();
            heading--;
            if(error <= 0) {
                myMotor1.setPower((error / 180) + 0.8);
                myMotor2.setPower((error / 180) + 0.8);
            } else {
                myMotor1.setPower(0.8 - (error/180));
                myMotor2.setPower(0.8 - (error/180));
            }
            error = getError(targetAngle);
            telemetry.addData("Motor1", myMotor1.getCurrentPosition());
            telemetry.addData("Motor2", myMotor2.getCurrentPosition());
            telemetry.addData("current position", currentStep);
            telemetry.addData("error", error);
            telemetry.addData("Initial Z:", targetAngle);
            telemetry.addData("Current Z:", heading);
            telemetry.update();
            if(myMotor1.getCurrentPosition() >= target){
                currentStep++;
            }

        }




    }

    private double getError(int targetAngle) {

        double robotError;
        int currentHeading =  myGyro.getHeading();
        currentHeading--;

        robotError = targetAngle - myGyro.getHeading();
        return robotError;

    }

}
