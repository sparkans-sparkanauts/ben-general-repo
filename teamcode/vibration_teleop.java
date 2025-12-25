package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="vibration teleop", group="Linear Opmode")
public class vibration_teleop extends LinearOpMode {

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private DcMotor Intake, flyWheelLeft, flyWheelRight;
    private Servo flyWheelServo;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // --- Hardware Map ---
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        flyWheelLeft = hardwareMap.get(DcMotor.class, "flyWheelLeft");
        flyWheelRight = hardwareMap.get(DcMotor.class, "flyWheelRight");
        flyWheelServo = hardwareMap.get(Servo.class, "flyWheelServo");

        // --- Motor Directions ---
        frontLeft.setDirection(DcMotor.Direction.REVERSE);   // chain driven
        frontRight.setDirection(DcMotor.Direction.REVERSE);  // chain driven
        backLeft.setDirection(DcMotor.Direction.REVERSE);    // direct drive
        backRight.setDirection(DcMotor.Direction.FORWARD);   // direct drive

        Intake.setDirection(DcMotor.Direction.FORWARD);
        flyWheelRight.setDirection(DcMotor.Direction.FORWARD);
        flyWheelLeft.setDirection(DcMotor.Direction.REVERSE);

        // --- Run without encoders ---
        DcMotor[] motors = {frontLeft, backLeft, frontRight, backRight, Intake, flyWheelLeft, flyWheelRight};
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        flyWheelServo.setPosition(0.49);
        waitForStart();
        double kP = 0.03;   // strength of auto-rotation toward target


        while (opModeIsActive()) {

            // ===== DRIVE CONTROL =====
            double y = -gamepad1.left_stick_y;   // forward/back
            double x = gamepad1.left_stick_x;   // turning
            double strafe = gamepad1.right_stick_x; // strafing
            double pm=1.00;
            if (gamepad1.left_trigger>0.2){
                pm = 0.5;
            }
            // === FORWARD / BACKWARD ===
            if (y > 0.1) {  // forward
                frontLeft.setPower(y*pm);
                frontRight.setPower(-y*pm);
                backLeft.setPower(y*pm);
                backRight.setPower(y*pm);
            } else if (y < -0.1) { // backward
                frontLeft.setPower(y*pm);
                frontRight.setPower(-y*pm);
                backLeft.setPower(y*pm);
                backRight.setPower(y*pm);
            }
            // === TURNING ===
            else if (x > 0.1) {  // turn right
                frontLeft.setPower((x/2)*pm);
                backLeft.setPower((-x/2)*pm);
                frontRight.setPower((x/2)*pm);
                backRight.setPower((x/2)*pm);
            } else if (x < -0.1) { // turn left
                frontLeft.setPower((x/2)*pm);
                backLeft.setPower((-x/2)*pm);
                frontRight.setPower((x/2)*pm);
                backRight.setPower((x/2)*pm);
            }
            // === STRAFING ===
            else if (strafe > 0.1) {  // strafe right
                frontLeft.setPower(strafe*pm);
                backLeft.setPower(strafe*pm);
                frontRight.setPower(strafe*pm);
                backRight.setPower(-strafe*pm);
            } else if (strafe < -0.1) {  // strafe left
                frontLeft.setPower(strafe*pm);
                backLeft.setPower(strafe*pm);
                frontRight.setPower(strafe*pm);
                backRight.setPower(-strafe*pm);
            }
            // === NEUTRAL ===
            else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }

            // ===== INTAKE ALWAYS ON =====
            double intakePower = gamepad1.b ? 1.0 : 0.0;
            if(intakePower==0.0){
                Intake.setPower(1.0);
            }
            if(intakePower==1.0){
                Intake.setPower(-1.0);
            }
            // ===== FLYWHEEL CONTROL =====
            if (gamepad1.left_bumper) {
                flyWheelLeft.setPower(0.50);
                flyWheelRight.setPower(0.50);
            } else if (gamepad1.left_trigger > 0.1) {
                flyWheelLeft.setPower(0.75);
                flyWheelRight.setPower(0.75);
            } else {
                flyWheelLeft.setPower(0);
                flyWheelRight.setPower(0);
            }

            // ===== SERVO FLICK =====
            if (gamepad1.right_bumper) {
                flyWheelServo.setPosition(-0.7);
                sleep(250);
                flyWheelServo.setPosition(0.49);
                sleep(500);
            }

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}


