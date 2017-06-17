package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.safehardware.DummyDcMotor;

/**
 * Created by AlbrightIT on 5/9/2017.
 */
@TeleOp(name = "Holonomic", group = "HOLO")
public class DRSS_Holnomic_Test extends OpMode {
private DcMotor rearLeftMotor, rearRightMotor, frontLeftMotor, frontRightMotor = null;

    public void init() {

        try {
            rearLeftMotor = hardwareMap.dcMotor.get("RearLeftMotor");
            rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            rearLeftMotor = new DummyDcMotor();

        }
        try {
            rearRightMotor = hardwareMap.dcMotor.get("RearRightMotor");
            rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            rearRightMotor = new DummyDcMotor();

        }
        try {
            frontLeftMotor = hardwareMap.dcMotor.get("FrontLeftMotor");
            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            frontLeftMotor = new DummyDcMotor();

        }
        try {
            frontRightMotor = hardwareMap.dcMotor.get("FrontRightMotor");
            frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            frontRightMotor = new DummyDcMotor();

        }

    }

    public void loop(){

/*
    rearLeftMotor.setPower(Math.sin( Math.asin(gamepad1.left_stick_y)));
    frontLeftMotor.setPower(Math.sin( Math.asin(gamepad1.left_stick_y)));
    rearRightMotor.setPower(Math.sin( Math.asin(gamepad1.left_stick_x)));
    rearLeftMotor.setPower(Math.sin( Math.asin(gamepad1.left_stick_x)));
  */
        //rearLeftMotor.setPower(Math.sqrt((Math.pow(gamepad1.left_stick_y,2))/(Math.pow(gamepad1.left_stick_x,2))));
    frontLeftMotor.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x, -1.0, 1.0));
    frontRightMotor.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x,-1.0,1.0));
    rearRightMotor.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x,-1.0,1.0));
    rearLeftMotor.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x,-1.0,1.0));

    }

}
