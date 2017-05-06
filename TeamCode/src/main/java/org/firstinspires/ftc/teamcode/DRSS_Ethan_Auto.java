 package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.safehardware.DummyCRServo;
import org.firstinspires.ftc.teamcode.safehardware.DummyDcMotor;
import org.firstinspires.ftc.teamcode.safehardware.DummyDcMotorController;
import org.firstinspires.ftc.teamcode.safehardware.DummyOpticalDistanceSensor;
import org.firstinspires.ftc.teamcode.safehardware.DummyServo;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

/**
 * Created by AlbrightIT on 4/22/2017.
 */

@Autonomous(name = "Ethan Autonomous", group = "DRSS")
public class DRSS_Ethan_Auto extends OpMode {

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor particleAccelerator = null;
    private DcMotor particleLift = null;
    //Declare all motors being used

    private DcMotorController driveMotorController = null;
    private DcMotorController shooterMotorController = null;
    //Declare all motor controllers being used

    private int gyroStart = 0;

    private CRServo pushRod = null;
    //Declare the a CRSevo for the push rod on the back of the robot

    private Servo clawTrigger = null;
    private Servo cannonSeal = null;

    private OpticalDistanceSensor opticalDistanceSensor = null;

    private ServoController servoController = null;
    //Declare a servo controller

    //  private ModernRoboticsI2cColorSensor colorSensor = null;
    private ColorSensor colorSensor = null;
    private ModernRoboticsI2cGyro gyro = null;
    //Declare the gyro and color sensor we will be using

    private ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    /*Create a new instance of runTime with a resolution of milliseconds.
     *We don't need nanosecond precision
     */

    private int currentStep = 1;


    public void init() {

        telemetry.addData("Status", "Initializing");

        /*The following series of try/catch statements attempt to assign each declared hardware
         *devices to a physical hardware device on the robot
         */

        try {
            leftMotor = hardwareMap.dcMotor.get("leftMotor");
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
//      leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception eTopMotor) {
            leftMotor = new DummyDcMotor();
        }

        try {
            rightMotor = hardwareMap.dcMotor.get("rightMotor");
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
//      rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception eBottomMotor) {
            rightMotor = new DummyDcMotor();
        }

        if ((leftMotor != null) && rightMotor != null) {
            leftMotor.setMode(STOP_AND_RESET_ENCODER);
            rightMotor.setMode(STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        try {
            particleAccelerator = hardwareMap.dcMotor.get("particleAccelerator");
        } catch (Exception eParticleAccelerator) {
            particleAccelerator = new DummyDcMotor();
        }

        try {
            particleLift = hardwareMap.dcMotor.get("particleLift");
        } catch (Exception eParticleLift) {
            particleLift = new DummyDcMotor();
        }

        try {
            shooterMotorController = hardwareMap.dcMotorController.get("ShooterMotorController");
        } catch (Exception eShooterController) {
            shooterMotorController = new DummyDcMotorController();
            telemetry.addData("Warning: ", "ShooterMotorController could not be found");
        }

        try {
            //colorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("colorSensor");
            colorSensor = hardwareMap.colorSensor.get("colorSensor");
            colorSensor.enableLed(true);
        } catch (Exception eColorSensor) {
            colorSensor = new ModernRoboticsI2cColorSensor(null, 0);
            telemetry.addData("Warning: ", "Color Sensor could not be found");
        }

        try {
            driveMotorController = hardwareMap.dcMotorController.get("DriveMotorController");
        } catch (Exception eDriveController) {
            driveMotorController = new DummyDcMotorController();
        }

        try {
            gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        } catch (Exception eGyroSensor) {
            gyro = null;
        }

        try {
            pushRod = hardwareMap.crservo.get("pushRod");
        } catch (Exception ePushRod) {
            pushRod = new DummyCRServo();
        }

        try {
            clawTrigger = hardwareMap.servo.get("clawTrigger");
        } catch (Exception eClawTrigger) {
            telemetry.addData("Exception ", "clawTrigger not found");
            clawTrigger = new DummyServo();
        }

        try {
            cannonSeal = hardwareMap.servo.get("cannonSeal");
        } catch (Exception eCannonSeal) {
            telemetry.addData("Exception ", "cannonSeal not found");
            cannonSeal = new DummyServo();
        }

        try {
            opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("opticalSensor");
        } catch (Exception eOpticalDistanceSensor) {
            telemetry.addData("Exception", "opticalSenor not found");
            opticalDistanceSensor = new DummyOpticalDistanceSensor();

        }

        if (pushRod != null) {
            pushRod.setPower(-1.0);
            //If the push rod is assigned, suck it into the robot

        }
    }



    public void loop(){

        if(currentStep == 1){ //Go forwards to the center vortex
            leftMotor.setPower(1.0);
            rightMotor.setPower(1.0);
            if(leftMotor.getCurrentPosition() >= 2880 && rightMotor.getCurrentPosition() >= 2880){
                currentStep++;
            }
        } else if(currentStep == 2){ //Turn left about 45 degrees
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.5);
            telemetry.addData("rotation",rightMotor.getCurrentPosition());
            if (rightMotor.getCurrentPosition() >= 4320) {
                currentStep++;

            }
        } else if(currentStep == 3){ //Go forwards a bit to avoid the center
            leftMotor.setPower(1.0);
            rightMotor.setPower(1.0);
            if (leftMotor.getCurrentPosition() >= 5760 && rightMotor.getCurrentPosition() >= 7200){
                currentStep++;
            }
        } else if (currentStep == 4){ //Turn right about 110 degrees
            leftMotor.setPower(1.0);
            rightMotor.setPower(0.0);
            if (leftMotor.getCurrentPosition() >= 9000 && rightMotor.getCurrentPosition() >= 7200){
                currentStep++;
            }
        } else if (currentStep == 5){ //Go forwards all the way to the blue ramp
            leftMotor.setPower(1.0);
            rightMotor.setPower(1.0);
            if (leftMotor.getCurrentPosition() >= 17640 && rightMotor.getCurrentPosition() >= 15840){
                currentStep++;
            }
        } else if (currentStep == 6) {
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
            particleAccelerator.setPower(1.0);
            if (particleAccelerator.getCurrentPosition() == 1440) {

            }

        }




    }

    public void stopMotors(){
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

    public void resetMotors(){
        leftMotor.setMode(STOP_AND_RESET_ENCODER);
        rightMotor.setMode(STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveForward(int goal){
        resetMotors();
        while ((leftMotor.getCurrentPosition()< goal && rightMotor.getCurrentPosition()< goal)){
            rightMotor.setPower(1.0);
            leftMotor.setPower(1.0);
        }
        stopMotors();
    }
}
