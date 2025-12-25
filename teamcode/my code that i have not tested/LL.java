
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="LL Auto Align + Drive + Shoot", group="Linear")
public class LL extends OpMode {

    // === LIMELIGHT ===
    private Limelight3A limelight;

    // === DRIVETRAIN ===
    private DcMotor frontLeft, backLeft, frontRight, backRight;

    // === FLYWHEEL ===
    private DcMotor flyWheel;
    private Servo flyWheelServo;

    // === TURNING PID CONSTANTS ===
    double kP = 0.04;
    double kI = 0.0;
    double kD = 0.0015;

    double integral = 0;
    double previousError = 0;

    // === STEPS ===
    enum State { ROTATE_TO_TAG, DRIVE_TO_RANGE, SPIN_FLYWHEEL, DONE }
    State state = State.ROTATE_TO_TAG;

    // === RANGE TARGET ===
    double targetArea = 2.5;  // increase to get closer

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        flyWheel = hardwareMap.get(DcMotor.class, "flyWheel");
        flyWheelServo = hardwareMap.get(Servo.class, "flyWheelServo");

        // Motor Directions (YOUR SETUP)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        limelight.start();
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            stopDrive();
            telemetry.addLine("NO TAG");
            telemetry.update();
            return;
        }

        double tx = result.getTx();
        double ta = result.getTa();

        switch (state) {

            // ============================================================
            // 1️⃣  ROTATE UNTIL TAG CENTERED
            // ============================================================
            case ROTATE_TO_TAG: {

                double error = tx;
                double derivative = error - previousError;
                integral += error;

                double turnPower = (kP * error) + (kI * integral) + (kD * derivative);

                previousError = error;

                // Clamp
                turnPower = Math.max(-0.35, Math.min(0.35, turnPower));

                // Apply your motor pattern for turning:
                frontLeft.setPower(-turnPower);
                backLeft.setPower(-turnPower);
                frontRight.setPower(turnPower);
                backRight.setPower(turnPower);

                telemetry.addData("TURNING tx", tx);
                telemetry.addData("Power", turnPower);

                // If aligned within ±1 degree, move to forward-driving mode
                if (Math.abs(tx) < 1.0) {
                    stopDrive();
                    state = State.DRIVE_TO_RANGE;
                }

                break;
            }

            // ============================================================
            // 2️⃣  DRIVE FORWARD UNTIL CLOSE ENOUGH (Based on Ta)
            // ============================================================
            case DRIVE_TO_RANGE: {

                if (ta >= targetArea) {
                    stopDrive();
                    state = State.SPIN_FLYWHEEL;
                    break;
                }

                double forwardPower = 0.25;

                // Your forward pattern:
                frontLeft.setPower(-forwardPower);
                frontRight.setPower(-forwardPower);
                backLeft.setPower(forwardPower);
                backRight.setPower(forwardPower);

                telemetry.addData("Driving Forward Ta=", ta);

                break;
            }

            // ============================================================
            // 3️⃣  SPIN FLYWHEEL
            // ============================================================
            case SPIN_FLYWHEEL: {

                flyWheel.setPower(1.0);

                telemetry.addLine("Flywheel Spinning!");
                state = State.DONE;
                break;
            }

            // ============================================================
            case DONE: {
                stopDrive();
                telemetry.addLine("DONE.");
                break;
            }
        }

        telemetry.update();
    }

    // ================================================================
    // Stop drivetrain helper
    // ================================================================
    void stopDrive() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}
