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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp
public class Enums extends OpMode {

    // idek what this is

    private DcMotor turret, intake, index;
    private DcMotorEx flyWheel;
    private Servo flicker, hood, blocker;
    private Limelight3A limelight;
    private IMU imu;

    // variables
    double forward, strafe, rotate;
    double turretPower, rpm, lastError, lastTime;
    double HOOD_FAR = 0.6;
    double HOOD_CLOSE = 0.3;
    double HOOD_MED = 0.4;

    double turretP = 0.03;
    double turretD = 0;
    double turretF = 0;

    double flyWheelP = 0.0015;

    // hood/turret config
    double hoodPosition = 0.3;

    double rpmTarget = 0;
    double rpmTolerance = 100;
    boolean flywheelAtSpeed = false;

    double RPM_HIGH = 1500;
    double RPM_MED = 1200;
    double RPM_LOW = 900;

    double flyWheelI = 0;
    double flyWheelI_Tune = 0.00005;

    LLResult llResult;

    double tagDistanceInchs;
    String zone = "close";


    public void setHoodAngle() {
        if (llResult != null && llResult.isValid()) {
            if (tagDistanceInchs >= 72) {
                rpmTarget = RPM_HIGH;
                hood.setPosition(HOOD_FAR);
                zone = "far";

            } else if (tagDistanceInchs >= 36) {
                rpmTarget = RPM_MED;
                hood.setPosition(HOOD_MED);
                zone = "med";

            } else {
                rpmTarget = RPM_LOW;
                hood.setPosition(HOOD_CLOSE);
                zone = "close";
            }

        } else { // fallback
            rpmTarget = RPM_MED;
            hood.setPosition(HOOD_MED);
        }
    }

    public void getLatestResult() {
        llResult = limelight.getLatestResult();
    }


    private void flywheelRPM() {

        double error = rpmTarget - rpm;
        double power = error * flyWheelP + flyWheelI; // math for getting the power to the motor
        if (rpm > 900) {
            flyWheelI += error * flyWheelI_Tune; // integrate the error over time
            telemetry.addData("current Intigral", flyWheelI);
        } else {
            flyWheelI = 0;
        }

        flyWheel.setPower(Range.clip(power, 0, 1));

        flywheelAtSpeed = Math.abs(error) <= rpmTolerance;


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

    prereqs1 drive = new prereqs1();
    LimelightMath limelightMath = new LimelightMath();


    @Override
    public void init() {
        // HARDWARE MAP
        drive.init(hardwareMap);
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        index = hardwareMap.get(DcMotor.class, "index");
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        flicker = hardwareMap.get(Servo.class, "servo");
        hood = hardwareMap.get(Servo.class, "hood");
        blocker = hardwareMap.get(Servo.class, "blocker");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        // ODO RECALIBRATE
        drive.configureOtos();


        // MOTOR DIRECTIONS
        turret.setDirection(DcMotor.Direction.FORWARD); // negative clockwise, positive counterclockwise
        intake.setDirection(DcMotor.Direction.FORWARD);
        flyWheel.setDirection(DcMotor.Direction.REVERSE);
        index.setDirection(DcMotor.Direction.REVERSE);


        // --- Run without encoders ---
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // zero power action
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //misc
        hood.setPosition(hoodPosition);
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


        //tracking, PIDF and allat i dont wanna explain it
        if (llResult != null && llResult.isValid() && gamepad1.left_bumper || readyToTrack) {

            double error = llResult.getTx();   // degrees
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;


            double derivative = (error - lastError) / dt /* Use last number for tuning, lower is less powerfull */;

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
        tagDistanceInchs = limelightMath.distanceFromTag();
        rpm = flyWheel.getVelocity();
        switch (flyWheelState) {

            case IDLE:
                rpmTarget = 0;
                flyWheel.setPower(0);
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
                flywheelRPM();
                index.setPower(0);
                blocker.setPosition(.8);
                readyToTrack = true;
                setHoodAngle();

                if (flywheelAtSpeed) {
                    gamepad1.rumble(50); //quick rumble to let me know its good
                }

                if (flywheelAtSpeed && gamepad1.x && !lastX) {
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
                flywheelRPM();
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
                flywheelRPM();
                index.setPower(0);
                readyToTrack = false;
                setHoodAngle();

                if (gamepad1.left_trigger < 0.1) {
                    flyWheelState = Enums.FlyWheelState.IDLE;
                }

                break;

            case REVERSE: // flywheel intake if needed bc intake breaks

                blocker.setPosition(0.3);
                index.setPower(-.5);
                flyWheel.setPower(-.4);
                readyToTrack = false;
                if (gamepad1.right_trigger < 0.1) {
                    flyWheelState = Enums.FlyWheelState.IDLE;
                }
        }
        lastX = gamepad1.x;

        telemetry.addData("distance", limelightMath.distanceFromTag());
        telemetry.addData("rpm", rpm);
        telemetry.addData("turret power", turretPower);
        telemetry.addData("target rpm", rpmTarget);
        telemetry.addData("flywheel state", flyWheelState);
        telemetry.addData("hood pose", hoodPosition);
        telemetry.addData("Zone", zone);
        telemetry.addData("Valid", llResult != null && llResult.isValid());
        telemetry.addData("blocker pos", blocker.getPosition());
        telemetry.addData("flicker", flicker.getPosition());
        telemetry.update();
    }
}