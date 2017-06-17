package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
/**
 * Created by AlbrightIT on 4/11/2017.
 */

/**
 * Created by AlbrightIT on 4/20/2017.
 */

@TeleOp(name = "BabyBotTele", group= "DRSS")
public class BabyBot_program extends OpMode {

    //Hardware must be declared outside of any methods (init, start, loop)
    DcMotor leftMotor;
    DcMotor rightMotor;
    ColorSensor colorSensor;


    public void init(){

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        colorSensor = hardwareMap.colorSensor.get("colorSensor");


        //gamepad1.setJoystickDeadzone((float)0.2);
    }

    public void loop(){
        leftMotor.setPower(gamepad1.left_stick_y);
        rightMotor.setPower(gamepad1.right_stick_y);

        if(gamepad1.a){
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
        }
    }
}
