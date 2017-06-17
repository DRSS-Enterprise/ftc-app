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

@TeleOp(name = "DRSS TeleOp1", group = "DRSS")
@Deprecated
@Disabled
public class DRSS_TeleOp1 extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor particleAccelerator;
    private DcMotor particleLift;

    private Servo backFlap;

    private CRServo pushRod;

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

    private ServoController servoController;

    private ColorSensor colorSensor;

    private double leftMotorPower;
    private double rightMotorPower;
    private double driveClip = 1.0;

    private int driveModifier = 1;

    private ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    //Declare all variables that will be used by the program.

    @Override
    public void init() {

        /*The following series of try/catch statements attempt to assign each declared hardware
         *device to a physical hardware device on the robot
         */

        try {
            leftMotor = hardwareMap.dcMotor.get("leftMotor");
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception eLeftMotor) {
            telemetry.addData("Exception ", "leftMotor not found");
            leftMotor = null;
        }


        try {
            DriveMotorController = hardwareMap.dcMotorController.get("DriveMotorController");
        } catch (Exception eDriveMotorController) {
            DriveMotorController = null;
            telemetry.addData("Exception ", "DriveMotorController not found");
        }

        try {
            ShooterMotorController = hardwareMap.dcMotorController.get("ShooterMotorController");
        } catch (Exception eShooterMotorController) {
            ShooterMotorController = null;
            telemetry.addData("Exception ", "ShooterMotorController not found");
        }

        try {
            colorSensor = hardwareMap.colorSensor.get("colorSensor");

        } catch (Exception eColorSensor) {
            colorSensor = null;
            telemetry.addData("Exception ", "colorSensor not found");
        }

        try {
            rightMotor = hardwareMap.dcMotor.get("rightMotor");
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception eRightMotor) {
            telemetry.addData("Exception ", "rightMotor not found");
            rightMotor = null;
        }

        try {
            particleAccelerator = hardwareMap.dcMotor.get("particleAccelerator");
            particleAccelerator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception eParticleAccelerator) {
            telemetry.addData("Exception ", "particleAccelerator not found");
            particleAccelerator = null;
        }

        try {
            backFlap = hardwareMap.servo.get("backFlap");
            backFlap.setDirection(Servo.Direction.FORWARD);
        } catch (Exception eBackFlap) {
            telemetry.addData("Exception ", "backFlap not found");
            backFlap = null;
        }

        try {
            pushRod = hardwareMap.crservo.get("pushRod");
        } catch (Exception ePushRod) {
            telemetry.addData("Exception ", "pushRod not found");
            pushRod = null;
        }

        try {
            servoController = hardwareMap.servoController.get("ServoController");
        }catch (Exception eServoController){
                telemetry.addData("Exception: ", "ServoController not found");
                servoController = null;
        }

        try {
            particleLift = hardwareMap.dcMotor.get("particleLift");
        } catch (Exception eParticleLift) {
            telemetry.addData("Exception ", "particleLift not found");
            particleLift = null;
        }

    }

    @Override
    public void start() {
        runTime.reset();
        //Reset the internal clock when the start button is pressed
    }

    @Override
    public void loop() {

        /*
        if(colorSensor.blue() >= 3){
            isBlue = true;
        } else{
            isBlue = false;
        }

        if(colorSensor.red() >= 3){
            isRed = true;
        } else{
            isRed = false;
        }

        Debugging code used for testing the color sensor. It does not affect the control of the
        robot in any way.
        */

        if (gamepad1.x) {
            driveClip = 1.0;
        }
        /* When driver 1 presses x, set the clip so the maximum and minimum drive power are 1.0 and
        * -1.0 respectively
        */

        if (gamepad1.y) {
            driveClip = 0.5;
        /* When driver 1 presses y, set the clip so the maximum and minimum drive power are 0.5 and
        * -0.5 respectively
        */

        }

        if(gamepad1.right_bumper){
            driveModifier = -driveModifier;
            //When driver 1 presses the right bumper, reverse the drive controls
        }

        leftMotorPower = driveModifier * Range.clip(Math.pow(gamepad1.left_stick_y, 3) +
                        (Math.pow(gamepad1.left_stick_y * 0.02, 2) + 0.9 * gamepad1.left_stick_y),
                -driveClip, driveClip);
        rightMotorPower = driveModifier * Range.clip(Math.pow(gamepad1.right_stick_y, 3) +
                        (Math.pow(gamepad1.right_stick_y * 0.02, 2) + 0.9 * gamepad1.right_stick_y),
                -driveClip, driveClip);
        /* Determine the power going to the motors based upon the cubic function
         * f(x) = x^3 + 0.02x^2 + 0.9x and clip the output to the current threshold (aka: driveClip)
         */


        if (leftMotor != null) {
            leftMotor.setPower(leftMotorPower);
            //If leftMotor has been assigned, have it run at the power calculated previously
        }


        if (rightMotor != null) {
            rightMotor.setPower(rightMotorPower);
            //If rightMotor has been assigned, have it run at the power calculated previously
        }



        if (gamepad2.a) {
            backFlap.setPosition(0.0);
            //When driver 2 presses a, open the back flap.
        }

        if (gamepad2.b) {
            backFlap.setPosition(1.0);
            //When driver 2 presses b, close the back flap.
        }

        if (gamepad2.right_bumper) {
            particleAccelerator.setPower(1.0);
        } else {
            particleAccelerator.setPower(0.0);
        }
        /*When driver 2 holds down the right bumper, run the particle accelerator, otherwise, stop
        * the particle accelerator
        */

        if (gamepad2.left_bumper) {
        } else if (gamepad2.left_trigger >= 0.4) {
            particleLift.setPower(-1.0);
        } else {
            particleLift.setPower(0.0);
        }
        /*When driver 2 holds down the left bumper, run the particle lift forwards, or when
         * driver holds down the left trigger a little under halfway down, run the particle lift in
         * reverse. If neither happens, stop the particle lift.
         */

        if (!gamepad2.right_bumper && gamepad2.right_trigger >= 0.5){
            particleAccelerator.setPower(-1.0);
            /* If driver 2 is not holding down the right bumper (which fires the cannon) and is
             * holding down the cannon at least halfway, reverse the cannon. This does not have any
             * function for competition, but is used for repair.
             */
        }

        if (gamepad2.left_stick_y >= 0.5) {
            pushRod.setPower(1.0);
            /* If driver 2 tilts the left stick upwards at least halfway, make the push rod go
             * forwards at full power
             */

        } else if (gamepad2.left_stick_y <= -0.5) {
            pushRod.setPower(-1.0);
            /* If driver 2 tilts the left stick downwards at least halfway, make the push rod go in
             * reverse at full power
             */

        } else {
            pushRod.setPower(0.0);
            //If driver 2 is not doing either of these things, stop the push rod.
        }

        telemetry.addData("Left Joy Y", gamepad1.left_stick_y);
        telemetry.addData("Right Joy Y", gamepad1.right_stick_y);
        telemetry.addData("leftpower", leftMotor.getPower());
        telemetry.addData("rightpower", rightMotor.getPower());
        telemetry.update();
        /*Print out information about the driver's vertical joystick coordinates and the power being
        * sent to both drive motors
        */

    }

    @Override
    public void stop() {

        if(leftMotor != null){
            leftMotor.setPower(0.0);
        }

        if(rightMotor != null){
            rightMotor.setPower(0.0);
        }

        if(particleAccelerator != null){
            particleAccelerator.setPower(0.0);
        }

        if(particleLift != null){
            particleLift.setPower(0.0);
        }

    }
    //When a stop has been requested, stop all assigned motors
}