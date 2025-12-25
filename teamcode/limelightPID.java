package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="new main ", group="Linear Opmode")
public class limelightPID extends LinearOpMode {

    // ===== HARDWARE DECLARATION =====
    private DcMotor frontLeft, backLeft, frontRight, backRight, intake, flyWheel, turret;

    // ===== TURNT PID VARIABLES =====
    private double Kp = 0.02;  // proportional gain
    private double Ki = 0.0;   // integral gain (optional)
    private double Kd = 0.001; // derivative gain (optional)
    private double integral = 0;
    private double lastError = 0;
    private double maxTurretPower = 0.6; // safety cap

    @Override
    public void runOpMode() {

        // ===== INITIALIZE HARDWARE =====
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        intake     = hardwareMap.get(DcMotor.class, "intake");
        flyWheel   = hardwareMap.get(DcMotor.class, "flyWheel");
        turret     = hardwareMap.get(DcMotor.class, "turret");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // ===== MOTOR DIRECTIONS =====
        frontLeft.setDirection(DcMotor.Direction.REVERSE);  // chain driven
        frontRight.setDirection(DcMotor.Direction.REVERSE); // chain driven
        backLeft.setDirection(DcMotor.Direction.REVERSE);   // direct drive
        backRight.setDirection(DcMotor.Direction.FORWARD);  // direct drive
        intake.setDirection(DcMotor.Direction.FORWARD);
        flyWheel.setDirection(DcMotor.Direction.FORWARD);

        // ===== RUN WITHOUT ENCODERS =====
        DcMotor[] motors = {frontLeft, backLeft, frontRight, backRight, intake, flyWheel};
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // ===== LIMELIGHT SETTINGS =====
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ===== LIMELIGHT TELEMETRY =====
            LLResult result = limelight.getLatestResult();
            double tx = 0;

            if (result != null && result.isValid()) {
                tx = result.getTx();
                Pose3D botpose = result.getBotpose();

                telemetry.addData("tx", tx);
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
            }

            // ===== TURNTUR PID COMPUTATION =====
            double error = -tx; // want tx -> 0
            integral += error * 0.02; // loop assumed ~20ms
            double derivative = (error - lastError) / 0.02;
            double turretPower = Kp * error + Ki * integral + Kd * derivative;
            turretPower = Range.clip(turretPower, -maxTurretPower, maxTurretPower);
            turret.setPower(turretPower);
            lastError = error;

            telemetry.addData("turretPower", turretPower);

            // ===== DRIVE CONTROL (MECANUM) =====
            double y = -gamepad1.left_stick_y;   // forward/back
            double x = gamepad1.right_stick_x;   // strafe
            double rx = gamepad1.left_stick_x;   // rotate
            double pm = 1.0;                      // power multiplier

            // calculate motor powers
            double frontLeftPower  = y + x + rx;
            double backLeftPower   = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower  = y + x - rx;

            // normalize so no value exceeds 1.0
            double max = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                    Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
            );
            if (max > 1.0) {
                frontLeftPower  /= max;
                backLeftPower   /= max;
                frontRightPower /= max;
                backRightPower  /= max;
            }

            // set motor powers
            frontLeft.setPower(frontLeftPower * pm);
            backLeft.setPower(backLeftPower * pm);
            frontRight.setPower(frontRightPower * pm);
            backRight.setPower(backRightPower * pm);

            // ===== INTAKE CONTROL =====
            double intakePower = gamepad1.b ? 1.0 : 0.0;
            intake.setPower(intakePower == 0.0 ? 1.0 : -1.0);

            // ===== FLYWHEEL CONTROL =====
            if (gamepad1.left_bumper) {
                flyWheel.setPower(0.50);
            } else if (gamepad1.left_trigger > 0.1) {
                flyWheel.setPower(0.75);
            } else {
                flyWheel.setPower(0);
            }

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    // ===== HELPER: GET BEARING =====
    private double getBearing(double tx) {
        return tx;
    }
}
