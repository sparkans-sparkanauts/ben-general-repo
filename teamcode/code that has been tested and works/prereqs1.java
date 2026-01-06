package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class prereqs1 {



    private DcMotor frontRight, frontLeft, backRight, backLeft;
    // Class members
    public double flPower, blPower, frPower, brPower;

    SparkFunOTOS myOtos;
    public void init(HardwareMap hwMap){

        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");

        myOtos = hwMap.get(SparkFunOTOS.class, "sensor_otos");


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    public void drive(double forward, double strafe, double rotate) {


        double fl = forward - strafe + rotate;
        double bl = forward + strafe + rotate;
        double fr = forward + strafe - rotate;
        double br = forward - strafe - rotate;



        double maxPower = 1.0; //motor max power
        double maxSpeed = 1.0;// max speed (change for outreach so we like molest our bot)

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(bl),
                                Math.max(Math.abs(fr),
                                        Math.abs(br)))));

        flPower = fl / max;
        blPower = bl / max;
        frPower = fr / max;
        brPower = br / max;

        frontLeft.setPower(flPower);
        backLeft.setPower(blPower);
        frontRight.setPower(frPower);
        backRight.setPower(brPower);
    }

    public void FieldOrientedTranslate(double targetPowerX, double targetPowerY, double rotation, boolean reset)
    {
        //SparkFunOTOS.Pose2D currentPose = myOtos.getPosition();

        double yaw = 0;

        //if (reset) {
        //  myOtos.setPosition(new SparkFunOTOS.Pose2D(currentPose.x, currentPose.y, 0));
        //}

        double stickRotation = 0;
        if (targetPowerY > 0 && targetPowerX < 0) //quad2
        {
            stickRotation = (Math.atan2(Math.abs(targetPowerY),Math.abs(targetPowerX)) + Math.PI/2) * 180/Math.PI;
        }
        else if (targetPowerY < 0 && targetPowerX < 0) //quad3
        {
            stickRotation = (Math.atan2(Math.abs(targetPowerY),Math.abs(targetPowerX)) + Math.PI) * 180/Math.PI;
        }
        else //quad1 and quad4
        {
            stickRotation = Math.atan2(targetPowerY,targetPowerX) * 180/Math.PI;
        }

        // angle of imu yaw supplemented by the stick's rotation, determined by atan
        double theta = (360-yaw) + stickRotation;
        double power = Math.hypot(targetPowerX,targetPowerY); //get hypotenuse of x and y tgt, getting the power

        // if at max power diag, limit to magnitude of 1
        // with the normalizing code, the diag movement had a bug where max power (being magnitude sqrt(2))--
        // --would cause wheels to flip polarity
        // to counteract this the power is limited to a proper magnitude
        if (power > 1)
        {
            power = 1;
        }
        else if (power < -1)
        {
            power = -1;
        }

        //get the sin and cos of theta
        //math.pi/4 represents 45 degrees, accounting for angular offset of mechanum
        double sin = Math.sin((theta * (Math.PI/180)) - (Math.PI/4));
        double cos = Math.cos((theta * (Math.PI/180)) - (Math.PI/4));
        //max of sin and cos, used to normalize the values for maximum efficiency
        double maxSinCos = Math.max(Math.abs(sin),Math.abs(cos));

        //same sign flip is to account for the inability of atan2, it typically only works for quadrants 1 and 4
        //by flipping the polarity when x < 0, we can use atan for every quadrant
        double flPower = 0;
        double frPower = 0;
        double blPower = 0;
        double brPower = 0;

//        rotation *= -1;

        flPower = power * cos/maxSinCos+rotation;
        frPower = power * sin/maxSinCos+rotation;
        blPower = power * sin/maxSinCos-rotation;
        brPower = power * cos/maxSinCos-rotation;

//        double frontMax = Math.max(Math.abs(flPower),Math.abs(frPower));
//        double backMax = Math.max(Math.abs(blPower),Math.abs(brPower));

        //another normalization
        if ((power + Math.abs(rotation)) > 1)
        {
            flPower /= power + Math.abs(rotation);
            frPower /= power - Math.abs(rotation);
            blPower /= power + Math.abs(rotation);
            brPower /= power - Math.abs(rotation);
        }

        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null)
        {
            double moveSpeed = 1;

            frontLeft.setPower(flPower * moveSpeed);
            frontRight.setPower(frPower * moveSpeed);
            backLeft.setPower(blPower * moveSpeed);
            backRight.setPower(brPower * moveSpeed);
        }

    }
    public void FieldOrientedTranslate(double targetPowerX, double targetPowerY, double rotation) {
        FieldOrientedTranslate(targetPowerX, targetPowerY, rotation, false);
    }






    public void configureOtos() {
        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.METER);
        // myOtos.setAngularUnit(AngleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

    }

}



