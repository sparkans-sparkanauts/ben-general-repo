package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="LLFullAuto_Barlow_Rewrite")
public class eastonscott extends OpMode {

    private Limelight3A limelight;

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private DcMotor intake, flywheelLeft, flywheelRight;
    private Servo flywheelServo;

    // PID tuning
    double xP = 0.015;          // horizontal correction
    double yP = 0.05;           // vertical/forward correction

    int timer = 0;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);  // AprilTag pipeline

        // Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        intake     = hardwareMap.get(DcMotor.class, "intake");
        flywheelLeft  = hardwareMap.get(DcMotor.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotor.class, "flywheelRight");

        flywheelServo = hardwareMap.get(Servo.class, "flywheelServo");

        // Directions
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheelServo.setPosition(0.27);

        intake.setPower(1.0);  // pre-intake during INIT

        // Set all to no-encoder mode
        DcMotor[] motors = {frontLeft, backLeft, frontRight, backRight, flywheelLeft, flywheelRight};
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        intake.setPower(0.0);  // fix: correct capitalization
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        // ===================================================
        // 1. Alignment Phase (timer < 300)
        // ===================================================
        if (timer < 300) {

            if (result != null && result.isValid()) {

                double tx = result.getTx();      // horizontal offset (deg)
                double ty = result.getTy();      // vertical offset (deg)

                double powerX = tx * xP;
                double powerY = ty * yP;

                // Basic mecanum correction: forward + turn
                double fl = (-powerX + powerY);
                double bl = ( powerX + powerY);
                double fr = (-powerX + powerY);
                double br = ( powerX + powerY);

                frontLeft.setPower(fl);
                backLeft.setPower(bl);
                frontRight.setPower(fr);
                backRight.setPower(br);

                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("powerX", powerX);
                telemetry.addData("powerY", powerY);

            } else {
                stopMotors();
                telemetry.addLine("Tag Lost — waiting...");
            }
        }

        // ===================================================
        // 2. Shooting Phase (300–550)
        // ===================================================
        else if (timer < 550) {

            stopMotors();
            telemetry.addLine("Shooting...");

            flywheelLeft.setPower(0.7);
            flywheelRight.setPower(0.7);

            // Start shooting at timer > 400
            if (timer > 400) {

                if (timer % 50 < 20) {
                    flywheelServo.setPosition(0.72); // shoot
                    intake.setPower(0.0);
                } else {
                    flywheelServo.setPosition(0.27); // reset

                    if (timer < 450) {
                        intake.setPower(1.0); // feeds next ring
                    }
                }
            }
        }

        // ===================================================
        // 3. END (timer >= 550)
        // ===================================================
        else {
            stopMotors();
            flywheelLeft.setPower(0.0);
            flywheelRight.setPower(0.0);
            intake.setPower(0.0);

            telemetry.addLine("Done");
        }

        telemetry.addData("timer", timer);
        telemetry.update();

        // Timer ALWAYS increments now
        timer++;
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}
