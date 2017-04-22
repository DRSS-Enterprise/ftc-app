package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "DRSS_OpticalTest", group = "DRSS")
public class DRSS_OpticalTest extends OpMode {

    private OpticalDistanceSensor opticalDistanceSensor = null;
    private Servo gateServo;

    @Override
    public void init(){

        try{
            opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("opticalSensor");
        } catch(Exception eOpticalDistanceSensor){
            telemetry.addData("Exception", "opticalDistanceSensor not found");
        }

        try{
            gateServo = hardwareMap.servo.get("cannonSeal");
        } catch(Exception eOpticalDistanceSensor){
            telemetry.addData("Exception", "servo not found");
        }

    }

    @Override
    public void loop(){
/*
        if(gamepad1.a){
            buggyServo.setPower(1.0);
        } else if(gamepad1.b){
            buggyServo.setPower(-1.0);
        } else{
            buggyServo.setPower(0.0);
        }
*/
      telemetry.addData("Light Detected", opticalDistanceSensor.getLightDetected());
      telemetry.addData("Raw Light Detected", opticalDistanceSensor.getRawLightDetected());


        Boolean moveServo = (opticalDistanceSensor.getLightDetected() <= 0.0015);
      telemetry.addData("Move Servo?", moveServo);

        if(moveServo){
            gateServo.setPosition(0);
            telemetry.addData("runningtest",1);
        } else{
            gateServo.setPosition(90);
            telemetry.addData("runningtest",2);
        }

    }
}
