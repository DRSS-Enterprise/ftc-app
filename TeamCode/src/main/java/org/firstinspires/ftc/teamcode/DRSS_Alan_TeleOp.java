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
 * Created by AlbrightIT on 4/20/2017.
 */
@TeleOp(name = "AlanTestOp", group= "DRSS")
public class DRSS_Alan_TeleOp extends OpMode{


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

    public void loop(){



        if (gamepad1.right_trigger>.5){
            rightMotor.setPower(gamepad1.right_stick_y+gamepad1.right_stick_x);
            leftMotor.setPower(gamepad1.right_stick_y-gamepad1.right_stick_x);
        } else {
            rightMotor.setPower(0);
            leftMotor.setPower(0);
        }
    }


}