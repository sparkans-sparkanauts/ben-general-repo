
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;


@TeleOp(name="new DrivetrainTest red allince ", group="Linear Opmode")
public class RedTeleop extends LinearOpMode {

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private DcMotor Intake, flyWheelLeft, flyWheelRight, turret;
    private Limelight3A limelight;
    private Servo flyWheelServo;
    private double getBatteryVoltage(){
        double result = Double.POSITIVE_INFINITY;

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    private double getBatteryPercentage() {
        double voltage = getBatteryVoltage();
        double percent = (voltage / 13.0) * 100.0;

        if (percent > 100) percent = 100;
        if (percent < 0) percent = 0;

        return percent;
    }
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
        turret = hardwareMap.get(DcMotor.class, "turret");
        flyWheelServo = hardwareMap.get(Servo.class, "flyWheelServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight3a");

        // Set to your desired pipeline
        limelight.pipelineSwitch(6);
        // --- Motor Directions ---
        frontLeft.setDirection(DcMotor.Direction.REVERSE);   // chain driven
        frontRight.setDirection(DcMotor.Direction.REVERSE);  // chain driven
        backLeft.setDirection(DcMotor.Direction.REVERSE);    // direct drive
        backRight.setDirection(DcMotor.Direction.FORWARD);   // direct drive

        Intake.setDirection(DcMotor.Direction.FORWARD);
        flyWheelRight.setDirection(DcMotor.Direction.FORWARD);
        flyWheelLeft.setDirection(DcMotor.Direction.REVERSE);
        turret.setDirection(DcMotor.Direction.FORWARD);
        // Run with encoders for some cause why not
        DcMotor[] motors = {frontLeft, backLeft, frontRight, backRight, turret};
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        // --- Run without encoders ---
        DcMotor[] motorss = {Intake, flyWheelLeft, flyWheelRight};
        for (DcMotor m : motorss) { // Extra s is important leave it there !!
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        flyWheelServo.setPosition(0.49);
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            // ===== DRIVE CONTROL =====
            double y = -gamepad1.left_stick_y;   // forward/back
            double x = gamepad1.left_stick_x;   // turning
            double s = gamepad1.right_stick_x; // strafing
            double pm=1.00;
            if (gamepad1.left_trigger>0.2){
                pm = 0.5;
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            frontLeft.setPower((y)+(x)+(s));
            frontRight.setPower((-y)+(x)+(s));
            backLeft.setPower((y)+(-x)+(s));
            backRight.setPower((y)+(x)+(-s));

            // ===== INTAKE ALWAYS ON =====
            double intakePower = gamepad1.b ? 1.0 : 0.0;
            if(intakePower==0.0){
                Intake.setPower(1.0);
            }
            if(intakePower==1.0){
                Intake.setPower(-1.0);
            }
            double batt = getBatteryPercentage();
            // ===== FLYWHEEL CONTROL =====
            if (gamepad1.left_bumper) {
                flyWheelLeft.setPower(0.650);
                flyWheelRight.setPower(0.650);
            } else if (gamepad1.left_trigger > 0.1) {
                flyWheelLeft.setPower(0.75);
                flyWheelRight.setPower(0.75);
            } else if (gamepad1.right_trigger > 0.1) {
                flyWheelLeft.setPower(1);
                flyWheelRight.setPower(1);
            }else {
                flyWheelLeft.setPower(0);
                flyWheelRight.setPower(0);
            }

            // ===== SERVO FLICK =====
            if (gamepad1.right_bumper) {
                flyWheelServo.setPosition(.29); // Up Postiton
                sleep(250); // time to get there
                flyWheelServo.setPosition(0.49); // Down Neterual
                sleep(500); // Time to get down then wait for ball to role on
            }
            if (result != null && result.isValid()) {

                double tx = result.getTx();  // horizontal offset

                if (tx > 2) {
                    turret.setPower(-0.3);     // too far right → rotate left
                }
                else if (tx < -2) {
                    turret.setPower(0.3);      // too far left → rotate right
                }
                else {
                    turret.setPower(0);        // within ±2 degrees → aligned
                    telemetry.addLine("AIMED: |tx| < 2");
                }
            }
            else {
                // No target → stop turret to avoid it spinning randomly
                turret.setPower(0);
                telemetry.addLine("Auto-Aim: No target");
            }
            if (result != null && result.isValid()) {

                double tx = result.getTx(); // ← horizontal angle to target
                double ty = result.getTy(); // vertical angle (if you need it)

                telemetry.addData("Angle to Target (tx)", tx);
                telemetry.update();

            } else {
                telemetry.addLine("No target detected");
                telemetry.update();
            }
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
