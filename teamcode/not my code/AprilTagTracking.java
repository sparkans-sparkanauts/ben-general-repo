//Tracks the april tag and moves the robot to a desired distance

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

@TeleOp(name = "AprilTagTracking", group = "Concept")
public class AprilTagTracking extends LinearOpMode {

    private DcMotor frontLeft, backLeft, frontRight, backRight;

    private double DESIRED_DISTANCE = 56.0; // inches
    private double SPEED_GAIN = 0.02;
    private double STRAFE_GAIN = 0.015;
    private double TURN_GAIN = 0.01;

    private double RANGE_TOLERANCE = 2.0; // inches
    private double BEARING_TOLERANCE = 2.0; // degrees

    @Override
    public void runOpMode() {
        initDriveMotors();

        telemetry.addData("->", "SWATISNIPER3000 TRACKING MODE READY");
        telemetry.addData("->", "Desired Distance: %.1f inches", DESIRED_DISTANCE);
        telemetry.addData("->", "Press Play to begin tracking");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean targetFound = false;
            double drive = 0, strafe = 0, turn = 0;

            // Get REAL Limelight data
            double tv = getLimelightValue("tv"); // Target valid (0 or 1)
            double tx = getLimelightValue("tx"); // Horizontal offset
            double ty = getLimelightValue("ty"); // Vertical offset
            double ta = getLimelightValue("ta"); // Target area
            double tid = getLimelightValue("tid"); // AprilTag ID

            if (tv == 1.0) {
                targetFound = true;

                // Estimate distance from target area
                double estimatedDistance = estimateDistanceFromArea(ta);
                double rangeError = estimatedDistance - DESIRED_DISTANCE;
                double bearingError = -tx; // Horizontal error
                double yawError = -ty; // Vertical error

                drive = -rangeError * SPEED_GAIN;
                strafe = -bearingError * STRAFE_GAIN;
                turn = -yawError * TURN_GAIN;

                // Limit powers for safety
                drive = Math.max(-0.5, Math.min(0.5, drive));
                strafe = Math.max(-0.5, Math.min(0.5, strafe));
                turn = Math.max(-0.5, Math.min(0.5, turn));

                moveRobot(drive, strafe, turn);

                telemetry.addLine("\n=== SWATISNIPER3000 TRACKING ACTIVE ===");
                telemetry.addData("TARGET", "AprilTag #%.0f", tid);
                telemetry.addData("RANGE", "Current: %.1f\" | Target: %.1f\" | Error: %.1f\"",
                        estimatedDistance, DESIRED_DISTANCE, rangeError);
                telemetry.addData("BEARING", "%.1f deg | Correction: %.3f", bearingError, strafe);
                telemetry.addData("YAW", "%.1f deg | Correction: %.3f", yawError, turn);
                telemetry.addData("TARGET AREA", "%.1f%%", ta * 100);

                boolean rangeLocked = Math.abs(rangeError) < RANGE_TOLERANCE;
                boolean bearingLocked = Math.abs(bearingError) < BEARING_TOLERANCE;
                boolean yawLocked = Math.abs(yawError) < BEARING_TOLERANCE;

                if (rangeLocked && bearingLocked && yawLocked) {
                    telemetry.addLine("*** TARGET LOCKED ***");
                    telemetry.addData("STATUS", "Perfect alignment at 56 inches!");
                } else {
                    telemetry.addLine(">>> TRACKING... <<<");
                    if (rangeError > 0) {
                        telemetry.addData("MOVEMENT", "Move FORWARD %.1f inches", Math.abs(rangeError));
                    } else {
                        telemetry.addData("MOVEMENT", "Move BACKWARD %.1f inches", Math.abs(rangeError));
                    }
                }
            } else {
                moveRobot(0, 0, 0);
                telemetry.addLine("SWATISNIPER3000: No targets in sight");
                telemetry.addData("DEBUG", "tv=%.0f (0=no target, 1=target found)", tv);
                telemetry.addData("DEBUG", "Make sure Limelight is on pipeline 8");
            }

            telemetry.addLine("\n=== LIMELIGHT REAL DATA ===");
            telemetry.addData("Target Found", targetFound);
            telemetry.addData("tv (Valid Target)", "%.0f", tv);
            telemetry.addData("tx (Horizontal)", "%.2f°", tx);
            telemetry.addData("ty (Vertical)", "%.2f°", ty);
            telemetry.addData("ta (Area)", "%.3f", ta);
            telemetry.addData("tid (Tag ID)", "%.0f", tid);

            telemetry.update();
            sleep(20);
        }

        moveRobot(0, 0, 0);
    }

    private void initDriveMotors() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void moveRobot(double drive, double strafe, double turn) {
        double frontLeftPower = drive + strafe + turn;
        double backLeftPower = drive - strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backRightPower = drive + strafe - turn;

        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    private double estimateDistanceFromArea(double ta) {
        if (ta <= 0) return 100.0;
        // Calibration formula: distance = k / sqrt(area)
        // You'll need to calibrate this constant for your specific setup
        // Start with 6.0 and adjust based on real measurements
        double estimatedDistance = 6.0 / Math.sqrt(ta);
        return Math.max(24.0, Math.min(120.0, estimatedDistance));
    }

    // Method to get REAL values from Limelight via HTTP
    private double getLimelightValue(String key) {
        try {
            // Create URL connection to Limelight
            URL url = new URL("http://limelight.local:5807/" + key);
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            connection.setConnectTimeout(1000); // 1 second timeout
            connection.setReadTimeout(1000);

            // Read the response
            BufferedReader in = new BufferedReader(new InputStreamReader(connection.getInputStream()));
            String inputLine;
            StringBuilder content = new StringBuilder();
            while ((inputLine = in.readLine()) != null) {
                content.append(inputLine);
            }
            in.close();
            connection.disconnect();

            // Parse the response as double
            return Double.parseDouble(content.toString().trim());

        } catch (Exception e) {
            // If there's an error (Limelight not connected, etc.), return 0
            telemetry.addData("Network Error", "Could not connect to Limelight");
            return 0.0;
        }
    }

    // Optional: Method to set Limelight pipeline (for switching between different pipelines)
    private void setLimelightPipeline(int pipeline) {
        try {
            URL url = new URL("http://limelight.local:5807/pipeline");
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("POST");
            connection.setDoOutput(true);
            connection.getOutputStream().write(String.valueOf(pipeline).getBytes());
            connection.getOutputStream().close();
            connection.disconnect();
        } catch (Exception e) {
            telemetry.addData("Pipeline Set Error", e.getMessage());
        }
    }
}