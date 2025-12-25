package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="apriltag turret", group="Linear Opmode")
public class gptquickstart1 extends LinearOpMode {

    private DcMotor turret;
    private Limelight3A limelight;

    // Tuning constants
    private static final double TURRET_POWER = 0.35;
    private static final double BEARING_DEADBAND = 2.0;
    private static final double MAX_POWER = 0.6;

    @Override
    public void runOpMode() {

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor Intake = hardwareMap.get(DcMotor.class, "Intake");
        DcMotor flyWheelLeft = hardwareMap.get(DcMotor.class, "flyWheelLeft");
        DcMotor flyWheelRight = hardwareMap.get(DcMotor.class, "flyWheelRight");
        Servo flyWheelServo = hardwareMap.get(Servo.class, "flyWheelServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                double tx = result.getTx();  // x offset in degrees
                Pose3D botpose = result.getBotpose();

                telemetry.addData("tx", tx);
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());

                double bearing = getBearing(tx);

                double power;
                if (bearing > BEARING_DEADBAND) {
                    power = Range.clip(TURRET_POWER, -MAX_POWER, MAX_POWER);
                } else if (bearing < -BEARING_DEADBAND) {
                    power = Range.clip(-TURRET_POWER, -MAX_POWER, MAX_POWER);
                } else {
                    power = 0.0;
                }

                turret.setPower(power);

                telemetry.addData("bearing", "%.2f", bearing);
                telemetry.addData("turret power", "%.2f", power);
            }

            telemetry.update();
        }
    }

    // ----------------------
    // Bearing function
    // ----------------------
    private double getBearing(double tx) {
        // Here bearing = tx from Limelight
        // Positive = target is right, Negative = left
        return tx;
    }
}
