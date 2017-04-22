package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.safehardware.DummyCRServo;
import org.firstinspires.ftc.teamcode.safehardware.DummyColorSensor;
import org.firstinspires.ftc.teamcode.safehardware.DummyDcMotor;
import org.firstinspires.ftc.teamcode.safehardware.DummyDcMotorController;
import org.firstinspires.ftc.teamcode.safehardware.DummyOpticalDistanceSensor;
import org.firstinspires.ftc.teamcode.safehardware.DummyServo;
import org.firstinspires.ftc.teamcode.safehardware.DummyServoController;
/**
 * Created by AlbrightIT on 4/11/2017.
 */

@TeleOp(name = "TonyTestOp", group= "DRSS")
public class TeleOp_Sandbox extends OpMode {


    //Hardware must be declared outside of any methods (init, start, loop)
    DcMotor leftMotor;
    DcMotor rightMotor;
    CRServo pushRod;
    Servo cannonSeal;
    double leftPower = 0.0;
    boolean aLastPressed = false;
    boolean driving = false;
    boolean slowOn = false;

    @Override
    public void init() {

        //Hardware must be assigned within a method (init, start, loop)
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        pushRod = hardwareMap.crservo.get("pushRod");
        cannonSeal = hardwareMap.servo.get("cannonSeal");

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    @Override
    public void loop() {

        //cannonSeal.setPosition(double)
        //0.0 is 0 degrees, 1.0 is 180 degrees

        //pushRod.setPower(double)
        //works like a motor

        //leftMotor.getCurrentPosition
        //rightMotor.getCurrentPositon
        //Holds the current postion of the encoder. 1440 units per revolution

        //telemetry.addData("Title", "message")
        //Sends data to the driver phone. The message can be any variable or an number, or just about anything
/*

        if(gamepad2.a && !aLastPressed){
            driving = !driving;
            aLastPressed = true;
        } else if(!gamepad2.a && aLastPressed){
            aLastPressed = false;
        }



       if(driving){
           leftMotor.setPower(1);
       } else{
           leftMotor.setPower(0);
       }
*/
        telemetry.addData("Example", aLastPressed);
        telemetry.addData("Is driving", driving);

        /*
        if(gamepad1.right_stick_y >= 0.5){
            rightMotor.setPower(1);
        } else
            if (gamepad1.right_stick_y < -0.5){
            rightMotor.setPower(-1);
        } else {rightMotor.setPower(0);

        if(gamepad1.left_stick_y >= 0.5){
            leftMotor.setPower(1);
            telemetry.addData("LeftForward", "leftforward");
        } else
            if (gamepad1.left_stick_y < -0.5){
                leftMotor.setPower(-1);
                telemetry.addData("LeftBackward", "leftbackward");
        } else {leftMotor.setPower(0);
        if(gamepad1.right_stick_y >= 0.5 && gamepad1.left_stick_y >= 0.5){
            rightMotor.setPower(1);
            leftMotor.setPower(1);
        } else if(gamepad1.right_stick_y < 0.5 && gamepad1.left_stick_y < 0.5){
            rightMotor.setPower(-1);
            leftMotor.setPower(-1);
        }
        */

        if (gamepad1.right_bumper) slowOn = !slowOn;

        if (gamepad1.left_stick_y >= 0.5 || gamepad1.left_stick_y <= -0.5 && !slowOn) {
            leftMotor.setPower(gamepad1.left_stick_y);
        }

        if (gamepad1.right_stick_y >= 0.5 || gamepad1.right_stick_y <= -0.5 && !slowOn) {
            if (gamepad1.right_stick_y >= 0.5 || gamepad1.right_stick_y <= -0.5) {
                rightMotor.setPower(gamepad1.right_stick_y);
            }

            if (gamepad1.left_stick_y >= 0.5 || gamepad1.left_stick_y <= -0.5 && slowOn) {

            }



        /*
        if(gamepad1.right_bumper) {
            leftMotor.setPower(gamepad1.left_stick_y * 0.5);
            rightMotor.setPower(gamepad1.right_stick_y*0.5);
        }
        else {
            leftMotor.setPower(gamepad1.left_stick_y);
            rightMotor.setPower(gamepad1.right_stick_y);
        }
        */










    /*
      if(gamepad1.a) {
            leftMotor.setPower(leftPower);

            if (leftPower == 0) {
                leftPower = 1;
            } else {
                leftPower = 0;
            }

        }
     */

        }
    }
}