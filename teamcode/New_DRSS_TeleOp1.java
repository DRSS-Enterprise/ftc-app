package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
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
 * Created by AlbrightIT on 10/13/2016.
 */

@TeleOp(name = "New DRSS TeleOp1", group = "DRSS")

public class New_DRSS_TeleOp1 extends OpMode {

    private enum DriveMode{

        DRIVE_NORMAL,
        DRIVE_SLOW,
        DRIVE_GLACIAL
    }

    private DriveMode currentDriveMode = DriveMode.DRIVE_NORMAL;

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor particleAccelerator;
    private DcMotor particleLift;
    private DcMotor capBall;
	//private DcMotor bottomCapBall;
	//private DcMotor topCapBall;

    private Servo backFlap;
	//private Servo topToggle;
    private Servo clawTrigger;
    private Servo cannonSeal;
    private Servo capballGrip;

    private CRServo pushRod;

    private boolean tonySoftLock = false;
    private boolean centerLastPressed = false;
    private boolean isBlue = true;
    private boolean isRed = false;
	private boolean slowMode = false;
    private boolean upLastPressed = false;
    private boolean autoGate = false;
    private boolean capballGripDown = true;
    private boolean moveServo = false;
    private boolean capBallAutoTrigger = false;
    private boolean capBallAutoDown = false;


    private DcMotorController driveMotorController;
    private DcMotorController shooterMotorController;
    private DcMotorController capBallMotorController;

    private ServoController servoController;
    private ServoController servoControllerButtonPusher;

    private ColorSensor colorSensor;
    private OpticalDistanceSensor opticalDistanceSensor;

    private double leftMotorPower;
    private double rightMotorPower;
    private double gateopen = 0;
    private double gateclosed = 0.5;

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
            leftMotor = new DummyDcMotor();
        }


        try {
            driveMotorController = hardwareMap.dcMotorController.get("DriveMotorController");
        } catch (Exception eDriveMotorController) {
            driveMotorController = new DummyDcMotorController();
            telemetry.addData("Exception ", "DriveMotorController not found");
        }

        try {
            shooterMotorController = hardwareMap.dcMotorController.get("ShooterMotorController");
        } catch (Exception eShooterMotorController) {
            shooterMotorController = new DummyDcMotorController();
            telemetry.addData("Exception ", "ShooterMotorController not found");
        }

        try {
            colorSensor = hardwareMap.colorSensor.get("colorSensor");
        } catch (Exception eColorSensor) {
            colorSensor = new DummyColorSensor();
            telemetry.addData("Exception ", "colorSensor not found");
        }

        try {
            opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("opticalSensor");
        } catch (Exception eColorSensor) {
            opticalDistanceSensor = new DummyOpticalDistanceSensor();
            telemetry.addData("Exception ", "opticalSensor not found");
        }

        try {
            rightMotor = hardwareMap.dcMotor.get("rightMotor");
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception eRightMotor) {
            telemetry.addData("Exception ", "rightMotor not found");
            rightMotor = new DummyDcMotor();
        }

        try {
            particleAccelerator = hardwareMap.dcMotor.get("particleAccelerator");
        } catch (Exception eParticleAccelerator) {
            telemetry.addData("Exception ", "particleAccelerator not found");
            particleAccelerator = new DummyDcMotor();
        }

        try {
            backFlap = hardwareMap.servo.get("backFlap");
            backFlap.setDirection(Servo.Direction.FORWARD);
        } catch (Exception eBackFlap) {
            telemetry.addData("Exception ", "backFlap not found");
            backFlap = new DummyServo();
        }

        try {
            capballGrip = hardwareMap.servo.get("capballGrip");
            capballGrip.setDirection(Servo.Direction.FORWARD);
            capballGrip.setPosition(0);
        } catch (Exception eBackFlap) {
            telemetry.addData("Exception ", "capballGrip not found");
            capballGrip = new DummyServo();
        }

        try {
            pushRod = hardwareMap.crservo.get("pushRod");
        } catch (Exception ePushRod) {
            telemetry.addData("Exception ", "pushRod not found");
            pushRod = new DummyCRServo();
        }

        try {
            servoController = hardwareMap.servoController.get("ServoController");
        } catch (Exception eServoController){
            telemetry.addData("Exception: ", "ServoController not found");
            servoController = new DummyServoController();
        }

        try {
            servoControllerButtonPusher = hardwareMap.servoController.get("Servo Controller button pusher");
        } catch (Exception eServoController){
            telemetry.addData("Exception: ", "ServoController not found");
            servoController = new DummyServoController();
        }

        try {
            particleLift = hardwareMap.dcMotor.get("particleLift");
        } catch (Exception eParticleLift) {
            telemetry.addData("Exception ", "particleLift not found");
            particleLift = new DummyDcMotor();
        }

        try{
            capBall = hardwareMap.dcMotor.get("capBall");
        } catch (Exception eCapBall){
            telemetry.addData("Exception ", "capBall not found");
            capBall = new DummyDcMotor();
        }

        try{
            clawTrigger = hardwareMap.servo.get("clawTrigger");
            clawTrigger.setPosition(0.0);
        } catch(Exception eClawTrigger){
            telemetry.addData("Exception ", "clawTrigger not found");
            clawTrigger = new DummyServo();
        }

        try{
            capBallMotorController = hardwareMap.dcMotorController.get("CapBallMotorController");
        } catch(Exception eCapBalMotorController){
            telemetry.addData("Exception ", "CapBallMotorController not found");
            capBallMotorController = new DummyDcMotorController();
        }

        try{
            cannonSeal = hardwareMap.servo.get("cannonSeal");
            cannonSeal.setPosition(gateclosed);
        } catch(Exception eCannonSeal){
            telemetry.addData("Exception ", "cannonSeal not found");
            cannonSeal = new DummyServo();
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
            currentDriveMode = DriveMode.DRIVE_NORMAL;
        }
        /* When driver 1 presses x, set the clip so the maximum and minimum drive power are 1.0 and
        * -1.0 respectively
        */

        if (gamepad1.y) {
            currentDriveMode = DriveMode.DRIVE_SLOW;
        /* When driver 1 presses y, set the clip so the maximum and minimum drive power are 0.5 and
        * -0.5 respectively
        */

        }

        if(gamepad1.a){
            currentDriveMode = DriveMode.DRIVE_GLACIAL;
        }

        if(gamepad1.right_bumper){
            driveModifier = -driveModifier;
            //When driver 1 presses the right bumper, reverse the drive controls
        }

        if(gamepad1.left_bumper && gamepad1.dpad_down){
            tonySoftLock = false;
        }

        if(gamepad2.guide && !centerLastPressed){
            tonySoftLock = !tonySoftLock;
            centerLastPressed = true;
        } else if(!gamepad2.guide && centerLastPressed){
            centerLastPressed = false;
        }

            if(currentDriveMode == DriveMode.DRIVE_NORMAL && !tonySoftLock){

                leftMotorPower = driveModifier * Range.clip(Math.pow(gamepad1.left_stick_y * 1.69, 3) -
                        0.26 * gamepad1.left_stick_y, -1.0, 1.0);
                rightMotorPower = driveModifier * Range.clip(Math.pow(gamepad1.right_stick_y * 1.69, 3) -
                        0.26 * gamepad1.right_stick_y, -1.0, 1.0);

            } else if(currentDriveMode == DriveMode.DRIVE_SLOW && !tonySoftLock){

                leftMotorPower = driveModifier * Range.clip(Math.pow(gamepad1.left_stick_y, 3) +
                                (Math.pow(gamepad1.left_stick_y * 0.02, 2) + 0.9 * gamepad1.left_stick_y),
                        -0.5, 0.5);
                rightMotorPower = driveModifier * Range.clip(Math.pow(gamepad1.right_stick_y, 3) +
                                (Math.pow(gamepad1.right_stick_y * 0.02, 2) + 0.9 * gamepad1.right_stick_y),
                        -0.5, 0.5);

            } else if(currentDriveMode == DriveMode.DRIVE_GLACIAL && !tonySoftLock){
                //  leftMotorPower = driveModifier * Range.clip(Math.pow(gamepad1.left_stick_y * 0.38, 3) +
                //          0.0307 * gamepad1.left_stick_y, -0.35, 0.35);
                //  rightMotorPower = driveModifier * Range.clip(Math.pow(gamepad1.right_stick_y * 0.38, 3) +
                //          0.0307 * gamepad1.left_stick_y, -0.35, 0.4);
                leftMotorPower = driveModifier * Range.clip(Math.pow(gamepad1.left_stick_y, 3) +
                                (Math.pow(gamepad1.left_stick_y * 0.02, 2) + 0.9 * gamepad1.left_stick_y),
                        -0.35, 0.35);
                rightMotorPower = driveModifier * Range.clip(Math.pow(gamepad1.right_stick_y, 3) +
                                (Math.pow(gamepad1.right_stick_y * 0.02, 2) + 0.9 * gamepad1.right_stick_y),
                        -0.35, 0.35);
            } else{
                leftMotor.setPower(0.0);
                rightMotor.setPower(0.0);
            }
      
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

        if (gamepad2.right_bumper && gamepad2.right_trigger < 0.5) {
            particleAccelerator.setPower(1.0);
            if(!autoGate)
                cannonSeal.setPosition(gateclosed);
        } else {
            particleAccelerator.setPower(0.0);
            if(!autoGate)
                cannonSeal.setPosition(gateopen);
        }
        /*When driver 2 holds down the right bumper, run the particle accelerator, otherwise, stop
        * the particle accelerator
        */

        if (gamepad2.left_bumper) {
            particleLift.setPower(1.0);
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


        if (gamepad2.dpad_left) {
            pushRod.setPower(1.0);
            /* If driver presses dpad left make the push rod go
             * forwards at full power
             */

        } else if (gamepad2.dpad_right) {
            pushRod.setPower(-1.0);
            /* If driver presses dpad right make the push rod go in
             * reverse at full power
             */

        } else {
            pushRod.setPower(0.0);
            //If driver 2 is not doing either of these things, stop the push rod.
        }

        if(gamepad2.back){
            clawTrigger.setPosition(1.0);
        }

        if(gamepad1.back){
            clawTrigger.setPosition(0.2);
        }

        if(gamepad2.x){
            autoGate = true;
            //turns autogate mode on
        }

        if(gamepad2.y){
            autoGate = false;
            //turn autogate mode off
        }

        if(gamepad2.dpad_up && !upLastPressed){

            if(capballGripDown)
                capballGrip.setPosition(1);
            else
                capballGrip.setPosition(0);
            //toggles the cap ball gripping servo up or down when the driver presses dpad up.
            capballGripDown = !capballGripDown;
            upLastPressed = true;
        } else if (!gamepad2.dpad_up && upLastPressed){
            upLastPressed = false;
        }



        if(gamepad2.right_stick_y >= 0.5) {
            capBall.setPower(1.0);

        } else if(gamepad2.right_stick_y <= -0.5){
            capBall.setPower(-1.0);

        } else{
            capBall.setPower(0.0);

        }

        if(autoGate){
            moveServo = (opticalDistanceSensor.getLightDetected() <= 0.0015);
            telemetry.addData("Move Servo?", moveServo);
            if(moveServo){
                cannonSeal.setPosition(gateopen);
            } else{
                cannonSeal.setPosition(gateclosed);
            }
        }

        /*
        if(gamepad2.dpad_down && !capBallAutoTrigger){
            capBallAutoTrigger = true;
        }

        if(capBallAutoTrigger && estimateEquals(gamepad2.right_stick_y, 0, 0.5)){

            capBall.setPower(capBallAutoDown ? -1.0:1.0);

            if(capBall.getTargetPosition() >= 1440){
                capBallAutoDown = true;
            } else if(capBall.getCurrentPosition() <= 0.0) {
                capBallAutoTrigger = false;
                capBall.setPower(0.0);
            }

        }

/*
        if(gamepad2.right_stick_y >= 0.5 && !gamepad2.right_stick_button){
           // if(liftSwitch.getVoltage() > 0.1){
            if(!flipCapBall){
                bottomCapBall.setPower(-1.0);
                topCapBall.setPower(1.0);
            } else{
                topCapBall.setPower(-1.0);
                bottomCapBall.setPower(0.0);
            }

        } else if(gamepad2.right_stick_y <= -0.5 && !gamepad2.right_stick_button){
            //if(liftSwitch.getVoltage() > 0.1){
            if(!flipCapBall){
                bottomCapBall.setPower(1.0);
                topCapBall.setPower(-1.0);
            } else{
                topCapBall.setPower(1.0);
                bottomCapBall.setPower(0.0);
            }
        } else{
            bottomCapBall.setPower(0.0);
            topCapBall.setPower(0.0);
        }

        if(gamepad2.left_stick_y <= -0.5){
            bottomToggle.setPower(1.0);
        } else if(gamepad2.left_stick_y >= 0.5){
            bottomToggle.setPower(-1.0);
        } else{
            bottomToggle.setPower(0.0);
        }
/*
        if(gamepad2.right_stick_button && !aLastPressed) {
            bottomToggle.setPosition((bottomToggle.getPosition() == 1.0 ? 0.4 : 1.0));
            aLastPressed = true;
        } else if(!gamepad2.right_stick_button){
            aLastPressed = false;
        }
*/



        telemetry.addData("leftJoy Y 2", gamepad2.left_stick_y);
        telemetry.addData("rightJoy Y 2", gamepad2.right_stick_y);
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

    private boolean estimateEquals(double value1, double value2, double threshold){
        if((value1 >= value2 - threshold) && (value1 <= value2 + threshold)){
            return true;
        } else{
            return false;
        }
    }
}