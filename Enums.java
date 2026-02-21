package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp
public class Enums extends OpMode {

    private interPLUH rpm = new interPLUH("rpm");
    private interPLUH hoodAngle = new interPLUH("hood");

    LLResult llResult;
    prereqs1 drive = new prereqs1();
    LimelightMath limelightMath = new LimelightMath();
    Outtake flywheelMath = new Outtake();


    // idek what this is

    private DcMotor turret, intake, index;
    private Servo flicker, blocker;
    private Limelight3A limelight;
    private IMU imu;

    // variables
    double forward, strafe, rotate;
    double turretPower, lastError, lastTime;



    double turretP = 0.03;


    // hood/turret config
    String zone = "close";

    public void getLatestResult() {
        llResult = limelight.getLatestResult();
    }



    // states cah cah what the fuck is a kilometer
    private enum FlyWheelState {
        IDLE,
        SPIN_UP,
        READY,
        FIRING,
        REVERSE

    }

    ElapsedTime indexTimer;

    final double fireTime = 300;
    boolean lastX = false;
    boolean lastA = false;
    boolean intakeOn = false;
    boolean isShooting = false;
    boolean readyToTrack = false;
    private FlyWheelState flyWheelState = Enums.FlyWheelState.IDLE;

    @Override
    public void init() {
        // HARDWARE MAP
        drive.init(hardwareMap);
        flywheelMath.init(hardwareMap);
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        index = hardwareMap.get(DcMotor.class, "index");
        flicker = hardwareMap.get(Servo.class,"flicker");
        blocker = hardwareMap.get(Servo.class,"blocker");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        // ODO RECALIBRATE
        drive.configureOtos();

        hoodAngle.add(36,.8);
        hoodAngle.add(0, 0.2);

        rpm.add(36, 1000);
        rpm.add(0, 0);


        // MOTOR DIRECTIONS
        turret.setDirection(DcMotor.Direction.FORWARD); // negative clockwise, positive counterclockwise
        intake.setDirection(DcMotor.Direction.FORWARD);
        index.setDirection(DcMotor.Direction.REVERSE);


        // --- Run without encoders ---
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // zero power action
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //misc
        limelight.pipelineSwitch(0);
        indexTimer = new ElapsedTime();
        lastTime = getRuntime();


    }

    public void start() {

        limelight.start();
    }


    @Override
    public void loop() {



        //DRIVE CONTROL
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        drive.FieldOrientedTranslate(forward, strafe, rotate);


        //TURRET ROTATION
        getLatestResult();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));


        //tracking, PIDF and allat i don't wanna explain it
        if (llResult != null && llResult.isValid() && (gamepad1.left_bumper || readyToTrack)) {

            double error = llResult.getTx();   // degrees
            double currentTime = getRuntime();


            double dt = currentTime - lastTime;

            if (dt <= 0) {

                dt = 0.00001;
            }


            double derivative = (error - lastError) / dt /* Use last number for tuning, lower is less powerful */;

            turretPower = (turretP * error);// * Math.min((derivative/turretD),1);

            turretPower = Math.max(-0.8, Math.min(0.8, turretPower));

            lastError = error;
            lastTime = currentTime;

        } else {

            turretPower = 0;
        }

        turret.setPower(turretPower);

        //INTAKE
        boolean aPressed = gamepad1.a && !lastA;

        //toggle
        if (aPressed) {
            intakeOn = !intakeOn;
            gamepad1.rumble(50); // rumble for when the intake toggles
        }

        //motor control
        if (!intakeOn) {
            intake.setPower(0);
        } else {

            if (gamepad1.b) {
                intake.setPower(-1);// reverse override
                gamepad1.rumble(50);// constant vibration bc its always being called
            } else {
                intake.setPower(1);  //else normal
            }
        }

        lastA = gamepad1.a;


        //FLYWHEEL

        double distance = limelightMath.distanceFromTag(llResult);
        boolean visionValid = (llResult != null && llResult.isValid());

        if (flyWheelState != FlyWheelState.IDLE && distance > 0) {
            flywheelMath.updateFromVision(distance, visionValid, getRuntime());
        }

            flywheelMath.flyWheel.setVelocity(rpm.get(limelightMath.distanceFromTag(llResult)));
            flywheelMath.hood.setPosition(hoodAngle.get(limelightMath.distanceFromTag(llResult)));

        switch (flyWheelState) {

            case IDLE:

                flywheelMath.flyWheelI = 0;
                flywheelMath.rpmTarget = 0;
                index.setPower(0);
                isShooting = false;
                blocker.setPosition(.8);
                flicker.setPosition(.55);
                readyToTrack = false;


                if (gamepad1.left_trigger > 0.1) {
                    flyWheelState = Enums.FlyWheelState.SPIN_UP;
                }

                if (gamepad1.right_trigger > 0.1) {
                    flyWheelState = Enums.FlyWheelState.REVERSE;
                }

                break;

            case SPIN_UP:
                flywheelMath.flyWheelRPM();
                index.setPower(0);
                blocker.setPosition(.8);
                readyToTrack = true;
                flywheelMath.setConfigFromDistance(distance);

                if (flywheelMath.flyWheelAtSpeed) {
                    gamepad1.rumble(50); //quick rumble to let me know its good
                }

                if (flywheelMath.flyWheelAtSpeed && gamepad1.x && !lastX) {
                    indexTimer.reset();
                    isShooting = true;
                    flyWheelState = Enums.FlyWheelState.FIRING;
                }
                if (gamepad1.left_trigger < 0.1) {
                    flyWheelState = Enums.FlyWheelState.IDLE;
                }
                break;

            case FIRING:
                blocker.setPosition(.3);
                flywheelMath.flyWheelRPM();
                index.setPower(1.0);
                readyToTrack = true;

                if (indexTimer.milliseconds() >= 250 && indexTimer.milliseconds() < fireTime) {
                    flicker.setPosition(0.55);
                }

                if (indexTimer.milliseconds() >= fireTime) {
                    index.setPower(0);
                    flicker.setPosition(.65);
                    isShooting = false;
                    flyWheelState = Enums.FlyWheelState.READY;
                }
                break;

            case READY:
                blocker.setPosition(.8);
                flywheelMath.setConfigFromDistance(distance);
                flywheelMath.flyWheelRPM();
                index.setPower(0);
                readyToTrack = false;


                if (gamepad1.left_trigger < 0.1) {
                    flyWheelState = Enums.FlyWheelState.IDLE;
                }

                break;

            case REVERSE: // flywheel intake if needed bc intake breaks

                blocker.setPosition(0.3);
                index.setPower(-.5);
                flywheelMath.flyWheel.setPower(-.4);
                readyToTrack = false;
                if (gamepad1.right_trigger < 0.1) {
                    flyWheelState = Enums.FlyWheelState.IDLE;
                }
        }
        lastX = gamepad1.x;

        telemetry.addData("rpm", flywheelMath.rpm);
        telemetry.addData("target rpm", flywheelMath.rpmTarget);
        telemetry.addData("flywheel state", flyWheelState);

        telemetry.addData("distance", distance);
        telemetry.addData("turret power", turretPower);
        telemetry.addData("hood pose", flywheelMath.hood.getPosition());
        telemetry.addData("Zone", zone);
        telemetry.addData("Valid", llResult != null && llResult.isValid());


        telemetry.addData("blocker pos", blocker.getPosition());
        telemetry.addData("flicker", flicker.getPosition());


        telemetry.update();
    }
}