package org.firstinspires.ftc.teamcode.safehardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by AlbrightIT on 2/18/2017.
 */

public class DummyServoController implements ServoController {
    @Override
    public void pwmEnable() {

    }

    @Override
    public void pwmDisable() {

    }

    @Override
    public PwmStatus getPwmStatus() {
        return null;
    }

    @Override
    public void setServoPosition(int servo, double position) {

    }

    @Override
    public double getServoPosition(int servo) {
        return 0;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
