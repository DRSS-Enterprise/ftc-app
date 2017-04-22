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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by AlbrightIT on 10/13/2016.
 */

@TeleOp(name = "DRSS TeleOp2", group = "DRSS")
@Deprecated
@Disabled
public class DRSS_TeleOp2 extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor particleAccelerator;
    private DcMotor particleLift;

    private Servo backFlap;

    private boolean toggleParticleAccelerator = false;
    private boolean toggleBackFlap = false;
    private boolean toggleParticleLift = false;
    private boolean resetBackFlap = false;
    private boolean resetParticleAccelerator = false;
    private boolean flipBackFlap = false;
    private boolean isBlue = true;
    private boolean isRed = false;

    private DcMotorController DriveMotorController;
    private DcMotorController ShooterMotorController;

    private ColorSensor colorSensor;

    private double leftMotorPower;
    private double rightMotorPower;
    private double runTimeTarget = 1e+12;

    private ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    @Override
    public void init(){

        try{
            leftMotor = hardwareMap.dcMotor.get("leftMotor");
//            leftMotor.setDirection(DcMotor.Direction.REVERSE);
        } catch(Exception eLeftMotor){
            telemetry.addData("Exception: ", "leftMotor not found");
            leftMotor = null;
        }

        try{
            DriveMotorController = hardwareMap.dcMotorController.get("DriveMotorController");
        } catch(Exception eDriveMotorController){
            DriveMotorController = null;
            telemetry.addData("Exception: ", "DriveMotorController not found");
        }

        try{
            ShooterMotorController = hardwareMap.dcMotorController.get("ShooterMotorController");
        } catch(Exception eShooterMotorController){
            ShooterMotorController = null;
            telemetry.addData("Exception: ", "ShooterMotorController not found");
        }

        try {
            colorSensor = hardwareMap.colorSensor.get("colorSensor");

        } catch(Exception eColorSensor){
            colorSensor = null;
            telemetry.addData("Exception: ", "colorSensor not found");
        }

        try{
            rightMotor = hardwareMap.dcMotor.get("rightMotor");
//            rightMotor.setDirection(DcMotor.Direction.FORWARD);
        } catch(Exception eRightMotor){
            telemetry.addData("Exception: ", "rightMotor not found");
            rightMotor = null;
        }

        try{
            particleAccelerator = hardwareMap.dcMotor.get("particleAccelerator");
//            particleAccelerator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch(Exception eParticleAccelerator){
            telemetry.addData("Exception: ", "particleAccelerator not found");
            particleAccelerator = null;
        }

        try{
            backFlap = hardwareMap.servo.get("backFlap");
//            backFlap.setDirection(Servo.Direction.FORWARD);
        } catch(Exception eBackFlap){
            telemetry.addData("Exception: ", "backFlap really not found");
            backFlap = null;
        }

        try{
            particleLift = hardwareMap.dcMotor.get("particleLift");
        } catch(Exception eParticleLift){
            telemetry.addData("Exception: ", "particleLift not found");
            particleLift = null;
        }

        if (leftMotor != null) {
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
        }
        if (rightMotor != null) {
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        if (particleAccelerator != null) {
            particleAccelerator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (backFlap != null) {
            backFlap.setDirection(Servo.Direction.FORWARD);
        }
    }

    @Override
    public void start() {
        runTime.reset();
    }

    @Override
    public void loop(){

        if (colorSensor != null) {
            if (colorSensor.blue() >= 3) {
                isBlue = true;
            } else {
                isBlue = false;
            }

            if (colorSensor.red() >= 3) {
                isRed = true;
            } else {
                isRed = false;
            }
        }

        leftMotorPower = Range.clip(Math.pow(gamepad1.left_stick_y, 3), -1, 1);
        rightMotorPower = Range.clip(Math.pow(gamepad1.right_stick_y, 3), -1, 1);

        if(gamepad2.a){
            //           toggleBackFlap = !toggleBackFlap;
            backFlap.setPosition(0.0);
        }
        //else{
        //    backFlap.setPosition(1.0);
        //}

        if(gamepad2.b){
//            toggleBackFlap = !toggleBackFlap;
//            backFlap.setPosition(-1.0);
//        } else{
            backFlap.setPosition(1.0);
        }

        if(gamepad2.right_bumper){
            particleAccelerator.setPower(1.0);
        } else{
            particleAccelerator.setPower(0.0);
        }

        if(gamepad2.left_bumper){
            particleLift.setPower(1.0);
        } else if(gamepad2.left_trigger >= 20){
            particleLift.setPower(-1.0);
        } else{
            particleLift.setPower(0.0);
        }

        /*if(toggleBackFlap) {

            if(flipBackFlap) {
                backFlap.setPosition(-1.0);
                if(backFlap.getPosition() <= -0.8) flipBackFlap = !flipBackFlap;

            } else{
                backFlap.setPosition(1.0);
                if(backFlap.getPosition() >= 0.8) flipBackFlap = !flipBackFlap;

            }


        }else{
            backFlap.setPosition(-1.0);

        }
*/


        telemetry.addData("Left Joy Y", gamepad1.left_stick_y);
        telemetry.addData("Right Joy Y", gamepad1.right_stick_y);
        telemetry.addData("leftpower", leftMotor.getPower());
        telemetry.addData("rightpower", rightMotor.getPower());
        telemetry.addData("isRed", isRed);
        telemetry.addData("isBlue", isBlue);
        telemetry.addData("toggle back flap", toggleBackFlap);
        telemetry.addData("flip back flap", flipBackFlap);
        telemetry.update();

        if (leftMotor != null) {
            leftMotor.setPower(leftMotorPower);
        }
        if (rightMotor != null) {
            rightMotor.setPower(rightMotorPower);
        }

    }
    @Override
    public void stop(){
        if (leftMotor != null) {
            leftMotor.setPower(0.0);
        }
        if (rightMotor != null) {
            rightMotor.setPower(0.0);
        }
        //particleAccelerator.setPower(0.0);
        //backFlap.setPower(0.0);
        //particleLift.setPower(0.0);
    }

    private boolean estimateEquals(int value, int target){
        if((value >= target - 50) && (value <= target + 50)){
            return true;
        } else{
            return false;
        }
    }
}
