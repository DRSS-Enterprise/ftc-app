package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
/**
 * Created by AlbrightIT on 4/20/2017.
 */

@TeleOp(name = "EthanTestOp", group= "DRSS")
public class DRSS_Ethan_TeleOp extends OpMode {

    //Hardware must be declared outside of any methods (init, start, loop)
    DcMotor leftMotor;
    DcMotor rightMotor;
    CRServo pushRod;
    Servo cannonSeal;
    boolean xLastPressed = false;
    boolean pushOut = false;
    boolean yLastPressed = false;
    boolean guardDown = false;

    public void init(){

        //Hardware must be assigned within a method (init, start, loop)
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        pushRod = hardwareMap.crservo.get("pushRod");
        cannonSeal = hardwareMap.servo.get("cannonSeal");

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void loop(){



/*
        if (gamepad1.x) {
            if (pushRod.getPower() == -1){
                pushRod.setPower(1);

            } else {
                pushRod.setPower(-1);
            }
        }
*/
        if(gamepad1.x && !xLastPressed){
            pushOut = !pushOut;
            xLastPressed = true;
        } else if(!gamepad1.x && xLastPressed){
            xLastPressed = false;
        }

        if(pushOut){
            pushRod.setPower(1.0);
        } else{
            pushRod.setPower(-1.0);
        }


        if(gamepad1.y && !yLastPressed){
            guardDown = !guardDown;
            yLastPressed = true;
        } else if(!gamepad1.y && yLastPressed){
            yLastPressed = false;
        }

        if(guardDown){
            cannonSeal.setPosition(0.5);
        } else{
            cannonSeal.setPosition(0);
        }

        //Sets the power for the left and right drive motors to the y position of their respective sticks.

        leftMotor.setPower(-gamepad1.left_stick_y);
        rightMotor.setPower(-gamepad1.right_stick_y);

    }
}
