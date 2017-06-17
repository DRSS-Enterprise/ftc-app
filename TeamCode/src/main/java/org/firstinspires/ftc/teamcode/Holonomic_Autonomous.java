package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.safehardware.DummyDcMotor;

/**
 * Created by AlbrightIT on 5/11/2017.
 */

@Autonomous(name = "AutoHolo", group = "DRSS")

public class Holonomic_Autonomous extends LinearOpMode {
    int forward;
    private DcMotor rearLeftMotor, rearRightMotor, frontLeftMotor, frontRightMotor = null;

    public void runOpMode() throws InterruptedException{

        try {
            rearLeftMotor = hardwareMap.dcMotor.get("RearLeftMotor");

        } catch(Exception E){
            rearLeftMotor = new DummyDcMotor();
        }
        try {
            rearRightMotor = hardwareMap.dcMotor.get("RearRightMotor");

        } catch(Exception E){
            rearRightMotor = new DummyDcMotor();
        }
        try {
            frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");

        } catch(Exception E){
            frontRightMotor = new DummyDcMotor();
        }
        try {
            frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");

        } catch(Exception E){
            frontLeftMotor = new DummyDcMotor();
        }

        waitForStart();


        opModeIsActive();
        while(opModeIsActive()) {
            frontLeftMotor.setPower(-1.0);
            frontRightMotor.setPower(1.0);
            rearLeftMotor.setPower(-1.0);
            rearRightMotor.setPower(1.0);
        }
    }

}
