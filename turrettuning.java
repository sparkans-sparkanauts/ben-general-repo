package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class turrettuning extends OpMode {

    // idek what to call this

    private DcMotor turret, Intake, index;
    private DcMotorEx flyWheel;
    private Servo flicker, hood;

    // variables
    double forward, strafe, rotate;
    double hoodPosition = 0.3;
    double rev;

    /*double rpmTarget = 0;
    static final double RPM_TOLERANCE = 100;
    boolean flywheelAtSpeed = false;

    boolean lastA = false;
    ;

    // flywheel config
    double TICKS_PER_REV = 28;
    double MAX_RPM = 8000;
    double GEAR_RATIO = 2;

    // RPM targets
    double RPM_HIGH = 5600;
    double RPM_LOW  = 3600;

    // hood
    double HOOD_CLOSE = 0.25;
   double HOOD_FAR   = 0.55;


    double fireTime = 0.5;

    ElapsedTime indexTimer = new ElapsedTime();
    boolean shooting = false;



    // finally state machine

    private enum flyWheelState{
        IDLE,
        SPIN_UP,
        READY,
        FIRING

    }

    private enum intakeState {
        IDLE,

        INTAKE,


        OUTTAKE
    }

    private intakeState IntakeState = intakeState.IDLE;
    private flyWheelState FlyWheelState = flyWheelState.IDLE;
*/

    boolean lastUp = false;
    boolean lastDown = false;
    boolean lastRight = false;
    boolean lastLeft = false;

    /*
        private double getFlywheelRPM() {
            return (flyWheel.getVelocity() * 60.0) / (TICKS_PER_REV * GEAR_RATIO);
        }

        private void flywheelRPM() {
            double rpm = getFlywheelRPM();

            if (rpm < rpmTarget - RPM_TOLERANCE) {
                flyWheel.setPower(1.0);
                flywheelAtSpeed = false;
            }
            else if (rpm > rpmTarget + RPM_TOLERANCE) {
                flyWheel.setPower(0.0);
                flywheelAtSpeed = false;
            }
            else {
                flywheelAtSpeed = true;
            }
        }
    */
    prereqs1 drive = new prereqs1();
    double sped = 0;

    @Override
    public void init() {
        // HARDWARE MAP
        drive.init(hardwareMap);
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        index = hardwareMap.get(DcMotor.class, "index");
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        flicker = hardwareMap.get(Servo.class, "servo");
        hood = hardwareMap.get(Servo.class, "hood");
        // ODO RECALIBRATE
        drive.configureOtos();


        // MOTOR DIRECTIONS
        turret.setDirection(DcMotor.Direction.FORWARD); // negative clockwise, positive counterclockwise
        Intake.setDirection(DcMotor.Direction.FORWARD);
        flyWheel.setDirection(DcMotor.Direction.REVERSE);
        index.setDirection(DcMotor.Direction.REVERSE);


        // --- Run without encoders ---
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // zero power action
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        hood.setPosition(hoodPosition);

    }


    @Override
    public void loop() {


        //DRIVE CONTROL
        forward = -gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;


        drive.FieldOrientedTranslate(forward, strafe, rotate);


        //REV

        if (gamepad1.y) {
            rev = -1;
        } else {
            rev = 1;
        }

        //MANUAL TURRET CONTROL
        if (gamepad1.right_bumper) {  // clockwise
            turret.setPower(-1);
        } else if (gamepad1.left_bumper) { // counterclockwise
            turret.setPower(1);
        } else {
            turret.setPower(0);
        }

        //INTAKE
        if (gamepad1.b) {
            Intake.setPower(1.0 * rev);
        }

        if (gamepad1.left_trigger > 0.1) {
            flyWheel.setPower(1 * rev);
        } else if (gamepad1.right_trigger > 0.1) {
            flyWheel.setPower(0.5 * rev);
        } else {
            flyWheel.setPower(0);
        }

        if (gamepad1.a) {
            index.setPower(1 * rev);
        } else {
            index.setPower(0.15);
        }

        /*switch (FlyWheelState) {

            case IDLE:
                rpmTarget = 0;
                flyWheel.setPower(0);
                index.setPower(0);
                shooting = false;

                if (gamepad1.left_trigger > 0.1) {
                    rpmTarget = RPM_HIGH;
                    hood.setPosition(HOOD_FAR);
                    FlyWheelState = flyWheelState.SPIN_UP;
                }
                else if (gamepad1.right_trigger > 0.1) {
                    rpmTarget = RPM_LOW;
                    hood.setPosition(HOOD_CLOSE);
                    FlyWheelState = flyWheelState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                flywheelRPM();
                index.setPower(0);

                if (flywheelAtSpeed && gamepad1.a && !lastA) {
                    indexTimer.reset();
                    shooting = true;
                    FlyWheelState = flyWheelState.FIRING;
                }

                if (rpmTarget == 0) {
                    FlyWheelState = flyWheelState.IDLE;
                }
                break;

            case FIRING:
                flywheelRPM();
                index.setPower(1.0);

                if (indexTimer.seconds() >= fireTime) {
                    index.setPower(0);
                    shooting = false;
                    FlyWheelState = flyWheelState.READY;
                }
                break;

            case READY:
                flywheelRPM();
                index.setPower(0);

                if (gamepad1.a && !lastA) {
                    indexTimer.reset();
                    FlyWheelState = flyWheelState.FIRING;
                }

                if (!gamepad1.a) {
                    FlyWheelState = flyWheelState.SPIN_UP;
                }
                break;
        }
*/


        // ===== SERVO FLICK =====
        if (gamepad1.y) {
            flicker.setPosition(0.3);
        } else {
            flicker.setPosition(.55);

        }

        if (gamepad1.dpad_up && !lastUp) {
//           hoodPosition = Math.min(0.8, hoodPosition + 0.1);
            hoodPosition = (hoodPosition + 0.1);
        }

        if (gamepad1.dpad_down && !lastDown) {
//            hoodPosition = Math.max(0.3, hoodPosition - 0.1);
            hoodPosition = (hoodPosition - 0.1);
        }


        if (gamepad1.dpad_right && !lastRight) {
//           hoodPosition = Math.min(0.8, hoodPosition + 0.1);
            sped = (sped + 100);
        }

        if (gamepad1.dpad_left && !lastLeft) {
//            hoodPosition = Math.max(0.3, hoodPosition - 0.1);
            sped = (sped - 100);
        }

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        flyWheel.setVelocity(sped);

        lastRight = gamepad1.dpad_right;
        lastLeft = gamepad1.dpad_left;

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        hood.setPosition(hoodPosition);

        telemetry.addData("hood", hoodPosition);
        telemetry.addData("pos", hood.getPosition());
        telemetry.addData("rpm target", sped);
        telemetry.addData("rpm", flyWheel.getVelocity());
        /*
        telemetry.addData("servo", servo.getPosition());
        telemetry.addData("test", 0);
        */
        //telemetry.addData("Flywheel RPM", getFlywheelRPM());
        //telemetry.addData("Target RPM", rpmTarget);
        //telemetry.addData("Flywheel State", FlyWheelState);


        telemetry.update();

        //lastA = gamepad1.a;

    }
}