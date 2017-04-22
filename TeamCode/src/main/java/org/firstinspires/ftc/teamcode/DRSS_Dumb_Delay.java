
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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name = "DRSS Dumb_Delay", group = "DRSS")

public class DRSS_Dumb_Delay extends LinearOpMode{

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor particleAccelerator = null;
    private DcMotor particleLift = null;
    //Declare all motors being used

    private DcMotorController driveMotorController = null;
    private DcMotorController shooterMotorController = null;
    //Declare all motor controllers being used

    private CRServo pushRod = null;
    //Declare the a CRSevo for the push rod on the back of the robot

    private ServoController servoController = null;
    //Declare a servo controller

    private ModernRoboticsI2cColorSensor colorSensor = null;
    private ModernRoboticsI2cGyro gyro = null;
    //Declare the gyro and color sensor we will be using

    private ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    /*Create a new instance of runTime with a resolution of milliseconds.
     *We don't need nanosecond precision
     */

    private boolean isFinished = false;
    //Declare a boolean stating if the robot has completed its route

    private boolean hasReset = false;
    //Declare a boolean stating if a reset cycle has been completed

    private int heading;
    //Declare an integer holding the heading of the gyro

    private int red, green, blue, alpha = 0;
    //Declare integers for each of the colo channels for the gyro

    private int lTarget;
    private int rTarget;
    //Only used for deprecated versions of the turn() method.

    private int target;
    //Declare an integer to contain the target encoder position when driving straight

    private int gyroTarget;
    //Declare an integer to contain the target gyro position when turning

    private int currentStep = 1;
    //Declare an integer to contain which the step the robot is currently running

    private int error;
    //Only used for future proportional control logic when driving forward.

    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing");
        //Output that the robot is initializing.

        try {
            leftMotor = hardwareMap.dcMotor.get("leftMotor");
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception eTopMotor) {
            leftMotor = null;
        }

        try {
            rightMotor = hardwareMap.dcMotor.get("rightMotor");
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception eBottomMotor) {
            rightMotor = null;
        }

        try {
            colorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("colorSensor");
        } catch (Exception eColorSensor) {
            colorSensor = null;
            telemetry.addData("Warning: ", "Color Sensor could not be found");
        }

        try {
            driveMotorController = hardwareMap.dcMotorController.get("DriveMotorController");
        } catch (Exception eDriveController) {
            driveMotorController = null;
        }

        try{
            particleAccelerator = hardwareMap.dcMotor.get("particleAccelerator");
            particleAccelerator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception eParticleAccelerator){
            particleAccelerator = null;
        }

        try{
            particleLift = hardwareMap.dcMotor.get("particleLift");
        } catch ( Exception eParticleLift){
            particleLift = null;
        }

        try{
            shooterMotorController = hardwareMap.dcMotorController.get("ShooterMotorController");
        } catch(Exception eShooterController){
            shooterMotorController = null;
        }

        try {
            gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        } catch (Exception eGyroSensor) {
            gyro = null;
        }

        try{
            pushRod = hardwareMap.crservo.get("pushRod");
        } catch ( Exception ePushRod){
            pushRod = null;
        }

        try{
            servoController = hardwareMap.servoController.get("servoController");
        } catch(Exception eServoController){
            servoController = null;
        }

        pushRod.setPower(-1.0);

        gyro.calibrate();
/*
        if (gyro != null) {
            while (gyro.isCalibrating()) {
                Thread.sleep(50);
                idle();
            }
        } else {
            telemetry.addData("ERROR", "GYRO NOT INTIALIZED");
        }
*/
        telemetry.addData(">", "Gyro Calibrated.  Press Start.");

        telemetry.update();

        waitForStart();

        runTime.reset();
        while(runTime.time() < 10000){
            //do nothing
        }

        target = 2800;

        while (!isFinished && !isStopRequested()) {
            telemetry.update();
            heading = gyro.getHeading();
            heading--;
            if (currentStep == 1) {
                reset();
                particleLift.setPower(1.0);
                driveStraight(target);
                telemetry.addData("Motor1", leftMotor.getCurrentPosition());
                telemetry.addData("Motor2", rightMotor.getCurrentPosition());
                telemetry.addData("current positon", currentStep);
                telemetry.addData("error", error);
                telemetry.addData("left Power", leftMotor.getPower());
                telemetry.addData("right Power", rightMotor.getPower());
                telemetry.addData("Current Z:", heading);
                telemetry.update();

            } else if(currentStep == 2){
                reset();
                leftMotor.setPower(0.0);
                rightMotor.setPower(0.0);
                shoot();
                telemetry.addData("Runtime", runTime);


           /* } else if(currentStep == 3) {
                target = 7000;
                reset();
                particleLift.setPower(0.0);
                driveStraight(target);
                telemetry.addData("Error", "Finished Early");
                telemetry.addData("lMotor", leftMotor.getCurrentPosition());
                telemetry.addData("rMotor", rightMotor.getCurrentPosition());
                telemetry.addData("lTarget", lTarget);
                telemetry.addData("rTarget", rTarget);

          /*  }else if(currentStep == 4){
                gyroTarget = 90;
                reset();
                turn(gyroTarget);

            } else if (currentStep == 5) {
                target = 7000;
                reset();
                driveStraight(target);
                telemetry.addData("lMotor", leftMotor.getCurrentPosition());
                telemetry.addData("rMotor", rightMotor.getCurrentPosition());
                telemetry.addData("lTarget", lTarget);
                telemetry.addData("rTarget", rTarget);

/*
            } else if (currentStep == 6) {
                gyroTarget = -90;
                reset();
                telemetry.addData("lMotor", leftMotor.getCurrentPosition());
                telemetry.addData("rMotor", rightMotor.getCurrentPosition());
                telemetry.addData("lTarget", lTarget);
                telemetry.addData("rTarget", rTarget);
                turn(gyroTarget);


            } else if (currentStep == 7) {
                reset();
                leftMotor.setPower(0.0);
                rightMotor.setPower(0.0);
                detectColor();
                telemetry.addData("red", red);
                telemetry.addData("blue", blue);
                telemetry.update();

            } else if (currentStep == 8) {
                target = 4000;
                reset();
                driveStraight(target);

            } else if( currentStep == 9){
                reset();
                leftMotor.setPower(0.0);
                rightMotor.setPower(0.0);
                detectColor();
                telemetry.addData("red", red);
                telemetry.addData("blue", blue);
                telemetry.update();

            } else if(currentStep == 10){
                gyroTarget = -114;
                reset();
                turn(gyroTarget);

            } else if(currentStep == 11){
                target = 8000;
                reset();
                driveStraight(target);

  */           } else{
                leftMotor.setPower(0.0);
                rightMotor.setPower(0.0);
                particleLift.setPower(0.0);

            }
        }
    }


    private void driveStraight(int target){

        leftMotor.setPower(0.6);
        rightMotor.setPower(0.6);

        telemetry.addData("Motor1", leftMotor.getCurrentPosition());
        telemetry.addData("Motor2", rightMotor.getCurrentPosition());
        telemetry.addData("current position", currentStep);
        telemetry.addData("Current Z:", heading);
        telemetry.update();

        if(leftMotor.getCurrentPosition() >= target){
            // && rightMotor.getCurrentPosition()>= target) {
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
            currentStep++;
            hasReset = false;
        }

    }

    private void turn(int gyroTarget){

        double lPower  = Range.clip(2 * ((gyro.getIntegratedZValue() - 1) - gyroTarget), -0.6, 0.6);
        double rPower = - Range.clip(2* ((gyro.getIntegratedZValue() - 1) - gyroTarget), -0.6, 0.6);

        leftMotor.setPower(lPower);
        rightMotor.setPower(rPower);

        telemetry.addData("lPower", lPower);
        telemetry.addData("rPower", rPower);
        telemetry.addData("Integrated Z", gyro.getIntegratedZValue()- 1);
        telemetry.addData("gyroTarget", gyroTarget);

        if(estimateEquals(gyro.getIntegratedZValue() - 1, gyroTarget, 3)){
            currentStep++;
            hasReset = false;
        }
    }

    @Deprecated
    private void turn(int lTarget, int rTarget){

        //GYRO TURN VALUES 37, 15, -119

        boolean lAchieved = false;
        boolean rAchieved = false;

        double lPower = Range.clip(-0.05 * ((leftMotor.getCurrentPosition() - lTarget)), -0.8, 0.8);

        double rPower = Range.clip(-0.05 * ((rightMotor.getCurrentPosition() - rTarget)), -0.8, 0.8);

        telemetry.addData("lAchieve", estimateEquals(leftMotor.getCurrentPosition(), lTarget, 100));
        telemetry.addData("rAchieve", estimateEquals(rightMotor.getCurrentPosition(), rTarget, 100));
        telemetry.addData("lPower", lPower);
        telemetry.addData("rPower", rPower);
        telemetry.update();

        if(!estimateEquals(leftMotor.getCurrentPosition(), lTarget, 100)){
            leftMotor.setPower(lPower);
        } else{
            leftMotor.setPower(0.0);
            lAchieved = true;
        }

        if(!estimateEquals(rightMotor.getCurrentPosition(), rTarget, 100)){
            rightMotor.setPower(rPower);
        } else{
            rightMotor.setPower(0.0);
            rAchieved = true;
        }

        if(lAchieved && rAchieved){
            currentStep++;
            hasReset = false;

        }
    }

    private void detectColor(){

        red = colorSensor.red();
        blue = colorSensor.blue();
        green = colorSensor.green();
        telemetry.addData("Blue value", blue);
        telemetry.addData("Red value", red);
        telemetry.addData("Green value", green);
        telemetry.addData("Current step", currentStep);

        if(runTime.time() < 2000){
            if(blue >= 2){
                telemetry.addData("Blue Achieved", true);
                //pushRod.setPower(1.0);
            } else if(red >= 2){
                telemetry.addData("Red Achieved", true);
                pushRod.setPower(1.0);
            }

        } else{
            pushRod.setPower(0.0);
            currentStep++;
        }

    }

    private void shoot(){
        if(runTime.time() < 7000) {
            particleAccelerator.setPower(1.0);
        } else {
            particleAccelerator.setPower(0.0);
            currentStep++;
        }

    }

    private void reset(){
        if(!hasReset) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            particleAccelerator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            particleAccelerator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            runTime.reset();
            gyro.resetZAxisIntegrator();
            hasReset = true;
        }

    }

    private int getError(int targetAngle) {

        int robotError;
        int currentHeading =  gyro.getHeading();
        currentHeading--;

        robotError = targetAngle - currentHeading;

        if(robotError >= 5 || robotError <= -5){
            return  robotError;
        } else{
            return 0;
        }

    }

    private boolean estimateEquals(int value, int target, int threshold){
        if((value >= target - threshold) && (value <= target + threshold)){
            return true;
        } else{
            return false;
        }
    }

}
