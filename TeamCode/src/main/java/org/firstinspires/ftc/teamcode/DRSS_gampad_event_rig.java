package org.firstinspires.ftc.teamcode;

import android.view.KeyEvent;
import android.view.MotionEvent;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
/**
 * Created by AlbrightIT on 4/11/2017.
 */

/**
 * Created by AlbrightIT on 4/20/2017.
 */

@TeleOp(name = "Event_rig", group= "DRSS")
public class DRSS_gampad_event_rig extends OpMode {

    //Hardware must be declared outside of any methods (init, start, loop)
    DcMotor leftMotor;
    DcMotor rightMotor;
    CRServo pushRod;
    Servo cannonSeal;
    boolean xLastPressed = false;
    boolean pushOut = false;
    boolean yLastPressed = false;
    boolean guardDown = false;

    public void init(){

        //Hardware must be assigned within a method (init, start, loop)
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        pushRod = hardwareMap.crservo.get("pushRod");
        cannonSeal = hardwareMap.servo.get("cannonSeal");


        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Controller_1 gamePadone = (new Controller_1());


    }

    public void loop(){
    }

    public void pushRodControl(double value){
        pushRod.setPower(value);
        return;
    }


    class Controller_1 extends Gamepad{

        Controller_1(){
            super();
            try {
                this.copy(gamepad1);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }
        }
        Controller_1(GamepadCallback callback){
            super(callback);
        }

        public boolean a_release = false;


        public void update(android.view.MotionEvent event) {

            id = event.getDeviceId();
            timestamp = event.getEventTime();

            left_stick_x = cleanMotionValues(event.getAxisValue(MotionEvent.AXIS_X));
            left_stick_y = cleanMotionValues(event.getAxisValue(MotionEvent.AXIS_Y));
            right_stick_x = cleanMotionValues(event.getAxisValue(MotionEvent.AXIS_Z));
            right_stick_y = cleanMotionValues(event.getAxisValue(MotionEvent.AXIS_RZ));
            left_trigger = event.getAxisValue(MotionEvent.AXIS_LTRIGGER);
            right_trigger = event.getAxisValue(MotionEvent.AXIS_RTRIGGER);
            dpad_down = event.getAxisValue(MotionEvent.AXIS_HAT_Y) > dpadThreshold;
            dpad_up = event.getAxisValue(MotionEvent.AXIS_HAT_Y) < -dpadThreshold;
            dpad_right = event.getAxisValue(MotionEvent.AXIS_HAT_X) > dpadThreshold;
            dpad_left = event.getAxisValue(MotionEvent.AXIS_HAT_X) < -dpadThreshold;

            callCallback();
        }

        /**
         * Update the gamepad based on a KeyEvent
         * @param event key event
         */
        public void update(android.view.KeyEvent event) {

            id = event.getDeviceId();
            timestamp = event.getEventTime();

            int key = event.getKeyCode();

            if (key == KeyEvent.KEYCODE_BUTTON_A && event.getAction()== KeyEvent.ACTION_UP){
                pushRodControl(0.5);
            }


            callCallback();
        }
    }
}


