package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Outtake {

    public Servo hood;

    public DcMotorEx flyWheel;
    //HOOD ANGLE
    double HOOD_FAR = 0.6;
    double HOOD_CLOSE = 0.3;
    double HOOD_MED = 0.4;

    //FLYWHEEL RPM
    double rpm;
    double RPM_HIGH = 1500;
    double RPM_MED = 1200;
    double RPM_LOW = 900;
    double flyWheelI = 0;
    double flyWheelI_Tune = 0.00005;
    double flyWheelP = 0.0015;

    // RPM CONFIG
    double rpmTarget = 0;
    double rpmTolerance = 100;
    boolean flyWheelAtSpeed = false;

    //VISION TIMEOUT LOCK
    private double lastValidTime = 0;
    private double lastValidDistance = 50;
    private final double VISION_TIMEOUT = 2.0; //seconds

    public void updateFromVision(double distance, boolean visionValid, double currentTime) {

        if (visionValid && distance > 0) {
            lastValidTime = currentTime;
            lastValidDistance = distance;
            setConfigFromDistance(distance);
            return;
        }

        boolean visionRecentlyValid =
                (currentTime - lastValidTime) < VISION_TIMEOUT;

        if (visionRecentlyValid) {
            // Hold last good config
            setConfigFromDistance(lastValidDistance);
        } else {
            // Fallback to MED
            setFallbackConfig();
        }
    }




    public void setFallbackConfig() {
        rpmTarget = RPM_MED;
        hood.setPosition(HOOD_MED);
    }


    public void setConfigFromDistance(double distance) {

        if (distance >= 72) {
            rpmTarget = RPM_HIGH;
            hood.setPosition(HOOD_FAR);
        }
        else if (distance <= 36) {
            rpmTarget = RPM_LOW;
            hood.setPosition(HOOD_CLOSE);
        }
        else {
            rpmTarget = RPM_MED;
            hood.setPosition(HOOD_MED);
        }
    }

    public void flyWheelRPM() {
        rpm = flyWheel.getVelocity();

        double error = rpmTarget - rpm;
        double power = error * flyWheelP + flyWheelI; // math for getting the power to the motor
        if (rpm > 900) {
            flyWheelI += error * flyWheelI_Tune; // integrate the error over time
        } else {
            flyWheelI = 0;
        }

          flyWheel.setPower(Range.clip(power, 0, 1));

        flyWheelAtSpeed = Math.abs(error) <= rpmTolerance;


    }

    public void init(HardwareMap hwMap){

        //HARDWARE MAP
        flyWheel = hwMap.get(DcMotorEx.class, "flyWheel");
        hood = hwMap.get(Servo.class, "hood");

        flyWheel.setDirection(DcMotor.Direction.REVERSE);

        flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
