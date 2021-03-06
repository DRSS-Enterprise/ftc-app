package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.safehardware.DummyCRServo;
import org.firstinspires.ftc.teamcode.safehardware.DummyDcMotor;
import org.firstinspires.ftc.teamcode.safehardware.DummyDcMotorController;
import org.firstinspires.ftc.teamcode.safehardware.DummyOpticalDistanceSensor;
import org.firstinspires.ftc.teamcode.safehardware.DummyServo;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

@Disabled
@Autonomous(name = "DRSS CaseSwitchBlueAutonomous", group = "DRSS")

public class DRSS_CaseSwitchBlueAutonomous extends LinearOpMode{

    private enum Color{
        RED,
        BLUE,
        OTHER
    }

    private Color colorFound = Color.OTHER;

    private final int ROTATION_TO_ENCODER = 100/11;

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

    private boolean isFinished = false;
    //Declare a boolean stating if the robot has completed its route

    private boolean resetRequested = true;
    //Declare a boolean stating if a reset cycle has been requested

    private int heading;
    //Declare an integer holding the heading of the gyro

    private int red, green, blue, alpha = 0;
    //Declare integers for each of the color channels for the gyro

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
            idle();
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        try{
            particleAccelerator = hardwareMap.dcMotor.get("particleAccelerator");
        } catch (Exception eParticleAccelerator){
            particleAccelerator = new DummyDcMotor();
        }

        try{
            particleLift = hardwareMap.dcMotor.get("particleLift");
        } catch ( Exception eParticleLift){
            particleLift = new DummyDcMotor();
        }

        try{
            shooterMotorController = hardwareMap.dcMotorController.get("ShooterMotorController");
        } catch(Exception eShooterController){
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

        try{
            pushRod = hardwareMap.crservo.get("pushRod");
        } catch ( Exception ePushRod){
            pushRod = new DummyCRServo();
        }

        try{
            clawTrigger = hardwareMap.servo.get("clawTrigger");
        } catch(Exception eClawTrigger){
            telemetry.addData("Exception ", "clawTrigger not found");
            clawTrigger = new DummyServo();
        }

        try{
            cannonSeal = hardwareMap.servo.get("cannonSeal");
        } catch(Exception eCannonSeal){
            telemetry.addData("Exception ", "cannonSeal not found");
            cannonSeal = new DummyServo();
        }

        try {
            opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("opticalSensor");
        } catch(Exception eOpticalDistanceSensor){
            telemetry.addData("Esception", "opticalSenor not found");
            opticalDistanceSensor = new DummyOpticalDistanceSensor();

        }

        if(pushRod != null) {
            pushRod.setPower(-1.0);
            //If the push rod is assigned, suck it into the robot
        }

        if (gyro != null) {
            gyro.calibrate();
            while (gyro.isCalibrating()) {
                Thread.sleep(50);
                idle();
                //If the gyro is assigned, calibrate and do not do anything else until it finishes
            }
            telemetry.addData(">", "Gyro Calibrated.  Press Start.");

        } else {
            telemetry.addData("ERROR", "GYRO NOT INTIALIZED");
            //If the gyro is not assigned, declare so.
        }

        telemetry.update();
        //Send telemetry data to robot controller

        waitForStart();
        //Do not run the following code until that start button is pressed.

        cannonSeal.setPosition(0.0);

        switch(currentStep){
            case 0:
                driveStraight(0);
                if (!opModeIsActive());
                    break;
            case 1:
                shoot();
                if (!opModeIsActive());
                    break;
            case 2:
                driveStraight(0);
                if (!opModeIsActive());
                    break;
            case 3:
                encodedgpsturn(0);
                if (!opModeIsActive());
                    break;
            case 4:
                driveStraight(0);
                if (!opModeIsActive());
                break;


        }

        stop();
    }


    /**
     * Drives the robot forward given a set number of encoder counts. Driving in reverse is not
     * currently supported, so it is recommended to make the robot turn 180 degrees first and then
     * drive forward. In addition, due to a disconnect of the right motor's encoder wire, only the
     * left motor is used to gauge distance. When the target is achieved, the drive motors reset.
     *
     * @param target The linear distance to move in encoder counts
     */

    //TODO Implement proportional control to ensure the robot drives in a straight line

    private void driveStraight(int target){

        if((target > 0) && opModeIsActive()) {
            //leftMotor.setPower(Range.clip( target * 0.5 / (leftMotor.getCurrentPosition() + 1), 0.1, 0.3));
            //rightMotor.setPower(Range.clip( target * 0.5 / (leftMotor.getCurrentPosition() + 1), 0.1, 0.3));
            if (opModeIsActive()) {
                leftMotor.setPower(0.2);
                rightMotor.setPower(0.2);
            }

            if((leftMotor.getCurrentPosition() >= target) && opModeIsActive()){
                //        && rightMotor.getCurrentPosition()>= target) {
                currentStep++;
                leftMotor.setPower(0.0);
                rightMotor.setPower(0.0);
                reset();
                //When the target is reached, increment the current step by one, stop both drive motors,
                //and request a reset.

            }

        }else if((target < 0) && opModeIsActive()){

            //leftMotor.setPower(Range.clip( target * -0.5 / (leftMotor.getCurrentPosition() + 1), -0.3, -0.1));
            //rightMotor.setPower(Range.clip( target * -0.5 / (leftMotor.getCurrentPosition() + 1), -0.3, -0.1));
            if (opModeIsActive()) {
                leftMotor.setPower(-0.2);
                rightMotor.setPower(-0.2);
            }
            if((leftMotor.getCurrentPosition() <= target && rightMotor.getCurrentPosition()<= target) && opModeIsActive()) {
                currentStep++;
                leftMotor.setPower(0.0);
                rightMotor.setPower(0.0);
                reset();
                //When the target is reached, increment the current step by one, stop both drive motors,
                // and request a reset.

            }
        }
        //Drive both motors forward a little over half power



    }

    public void reset(){
        leftMotor.setMode(STOP_AND_RESET_ENCODER);
        rightMotor.setMode(STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //gyro.resetZAxisIntegrator();
        pushRod.setPower(-1.0);
        runTime.reset();
    }

    public void encodedturn(DcMotor blah, double degrees) {


        DcMotor.RunMode startMode = blah.getMode();

        blah.setMode(STOP_AND_RESET_ENCODER);
        blah.setMode(RUN_TO_POSITION);

        blah.setTargetPosition((int)(degrees*(100/11)));

        blah.setMode(startMode);


    }


    public void encodedgpsturn(int degrees) {


        DcMotor.RunMode startMode = leftMotor.getMode();


        gyroStart = gyro.getHeading();
        int goal = degrees - gyroStart;

        leftMotor.setMode(STOP_AND_RESET_ENCODER);
        leftMotor.setMode(RUN_TO_POSITION);
        rightMotor.setMode(STOP_AND_RESET_ENCODER);
        rightMotor.setMode(RUN_TO_POSITION);

        if(opModeIsActive()) {
            leftMotor.setTargetPosition((int) (goal * (100 / 11)));
            rightMotor.setTargetPosition(-(int) (goal * (100 / 11)));
        }

        while((leftMotor.isBusy() || rightMotor.isBusy()) && opModeIsActive()){

        }

        leftMotor.setMode(startMode);
        rightMotor.setMode(startMode);


    }



    /**
     * Turns the robot according to distance left and right to move both motors. This has much more
     * inaccuracy than turn(int gyroTurn) and is harder to use as well. The method is kept in for
     * legacy reasons, but should not be used if possible. Being unfinalized code, it is
     * disorganized and difficult to read at points.
     *
     * @param lTarget The distance in encoder counts to move the left motor
     * @param rTarget The distance in encoder counts to move the right motor
     */


    /**
     * This method uses the data from the color sensor to tell what what color emitted light is.
     * Two units of either red or blue need to be detected by the color sensor to be accurately read
     * as red or blue. This condition will be checked for two seconds before the method stops
     * attempting and goes to the next step. Due to the beacons being very dim, the robot needs to
     * be lined up very precisely near the color beacon for this to be effective.
     */

    private void detectColor(){

        red = colorSensor.red();
        blue = colorSensor.blue();
        green = colorSensor.green();
        //Declare variables to hold the red, blue, and green values found by the color sensor

        telemetry.addData("Blue value", blue);
        telemetry.addData("Red value", red);
        telemetry.addData("Green value", green);
        telemetry.addData("Current step", currentStep);
        //Send data to the robot controller

        if(blue >= 2){
            telemetry.addData("Blue Achieved", true);
            colorFound = Color.BLUE;
            currentStep++;
            resetRequested = true;


        } else if(red >= 2){
            telemetry.addData("Red Achieved", true);
            colorFound = Color.RED;
            telemetry.addData("runTime", runTime.time());
            currentStep++;
            resetRequested = true;

        } else{
            colorFound = Color.OTHER;
            currentStep++;
            resetRequested = true;
        }


    }

    /**
     * Fires the cannon for seven seconds and moves onto the next step when finished.
     */
    private void shoot(){
        runTime.reset();
        while((runTime.time() < 5500) && opModeIsActive()) {
            particleAccelerator.setPower(1.0);
            //particleLift.setPower(1.0);
            cannonSeal.setPosition(opticalDistanceSensor.getLightDetected() <= 0.0043 ? 0.0:0.5);
        }
            particleAccelerator.setPower(0.0);
            particleLift.setPower(0.0);
            reset();


    }

    /**
     * Calculates the rotational error of the robot relative to the target angle. Was used in early
     * versions of of proportional control, but the method is accurate and can be re-implemented
     *
     * @param targetAngle The target angle that is trying to be reached
     * @return The error of the robot's rotation relative to the target
     */
    private double getError(int targetAngle) {

        double robotError = 0.0;
        //Declare a variable to hold the calculated error

        int currentHeading =  gyro.getHeading();
        currentHeading--;
        //Get the heading from the gyro (See notes on "heading" variable)

        robotError = targetAngle - gyro.getHeading();
        return robotError;
        //Calculate and return the rotational error of the robot

    }

    /**
     * Checks if two values are approximately equal to each other.
     *
     * @param value1 The 1st value to test
     * @param value2 The 2nd value
     * @param threshold How far off the two values can be to be considered approximately equal
     * @return A boolean statement describing if the values are equal within the threshold
     */
    private boolean estimateEquals(int value1, int value2, int threshold){
        if((value1 >= value2 - threshold) && (value1 <= value2 + threshold)){
            return true;
        } else{
            return false;
        }
    }

}