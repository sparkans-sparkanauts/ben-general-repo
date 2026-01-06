package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class bentest extends OpMode {

    double forward, strafe, rotate;
    prereqs1 drive = new prereqs1();

    private DcMotor turret, Intake, index, flyWheel;
    private Servo servo;


    @Override
    public void init() {
        // HARDWARE MAP
        drive.init(hardwareMap);
        turret = hardwareMap.get(DcMotor.class, "turret");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        index = hardwareMap.get(DcMotor.class, "index");
        flyWheel = hardwareMap.get(DcMotor.class, "flyWheel");
        servo = hardwareMap.get(Servo.class, "servo");

        // ODO RECALIBRATE
        drive.configureOtos();


        // MOTOR DIRECTIONS
        turret.setDirection(DcMotor.Direction.FORWARD); // negative clockwise, positive counterclockwise
        Intake.setDirection(DcMotor.Direction.REVERSE);
        flyWheel.setDirection(DcMotor.Direction.REVERSE);
        index.setDirection(DcMotor.Direction.REVERSE);


        // --- Run without encoders ---
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //**************** change later to use PIDF   ****************************

    }


    @Override
    public void loop() {

        //DRIVE CONTROL
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;


        drive.FieldOrientedTranslate(forward, strafe, rotate);

        //MANUAL TURRET CONTROL
        if (gamepad1.right_bumper) {  // clockwise
            turret.setPower(-0.35);
        } else if (gamepad1.left_bumper) { // counterclockwise
            turret.setPower(0.35);
        } else {
            turret.setPower(0);
        }

        //INTAKE
        if (gamepad1.b) {
            Intake.setPower(-1.0);
        } else {
            Intake.setPower(1.0);
        }

        //FLY WHEEL
        if (gamepad1.left_trigger > 0.1) {
            flyWheel.setPower(1);
        } else if (gamepad1.right_trigger > 0.1) {
            flyWheel.setPower(0.50);
        } else {
            flyWheel.setPower(0);
        }


        if (gamepad1.a) {
            index.setPower(0.5);
        } else {
            index.setPower(0);


        }


        // ===== SERVO FLICK =====
        if (gamepad1.right_bumper) {
            servo.setPosition(0);
            servo.setPosition(0.5);


        }
    }
}