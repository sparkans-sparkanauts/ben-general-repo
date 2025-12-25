package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="new main ", group="Linear Opmode")
public class apriltaglimelight extends LinearOpMode {

    private DcMotor frontLeft, backLeft, frontRight, backRight, intake, flyWheel, turret;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // --- Hardware Map ---
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flyWheel = hardwareMap.get(DcMotor.class, "flyWheel");
        turret = hardwareMap.get(DcMotor.class, "turret");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // --- Motor Directions ---
        frontLeft.setDirection(DcMotor.Direction.REVERSE);   // chain driven
        frontRight.setDirection(DcMotor.Direction.REVERSE);  // chain driven
        backLeft.setDirection(DcMotor.Direction.REVERSE);    // direct drive
        backRight.setDirection(DcMotor.Direction.FORWARD);   // direct drive

        intake.setDirection(DcMotor.Direction.FORWARD);
        flyWheel.setDirection(DcMotor.Direction.FORWARD);
        // --- Run without encoders ---
        DcMotor[] motors = {frontLeft, backLeft, frontRight, backRight, intake, flyWheel};
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


            // ======LIMELIGHT CONSTANTS
        double turretPower = 0.35; //
        double bearingDeadband = 2.0;
        double maxPower = 0.6;
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            // ======LIMELIGHT TELEMETRY & GET BEARING

            LLResult result = limelight.getLatestResult();

            double tx = 0;
            double power = 0;
            if (result != null && result.isValid()) { //check to maker sure tag is visble and valid: aka make sure tag is either red goal or blue goal

                tx = result.getTx();
                Pose3D botpose = result.getBotpose(); //get 3d localization

                // add offsets and bot pose to telemetry
                telemetry.addData("tx", tx);
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());


                double bearing = getBearing(tx); // uses tx to define "bearing" variable

                if (bearing > bearingDeadband) { // if bearing is more than the deadband, turn right towards the tag
                    power = Range.clip(turretPower, -maxPower, maxPower); // cap power to prevent erroneous setting breaking something
                } else if (bearing < -bearingDeadband) {// if bearing is more than the deadband, turn left towards the tag
                    power = Range.clip(-turretPower, -maxPower, maxPower);// cap power to prevent erroneous setting breaking something
                } else {
                    power = 0.0; // if either tag in inside deadband, or no tag is detected, set power to 0
                }
            }

            // =======Turret Rotation========
            turret.setPower(power);


            // ===== DRIVE CONTROL =====
            double y = -gamepad1.left_stick_y;   // forward/back
            double x = gamepad1.left_stick_x;   // turning
            double strafe = gamepad1.right_stick_x; // strafing
            double pm = 1.00;
            //if (gamepad1.left_trigger>0.2){
            //    pm = 0.5;
            //}
            //else {
            //    pm=1.00;
            //}
            // === FORWARD / BACKWARD ===
            if (y > 0.1) {  // forward
                frontLeft.setPower(y * pm);
                frontRight.setPower(-y * pm);
                backLeft.setPower(y * pm);
                backRight.setPower(y * pm);
            } else if (y < -0.1) { // backward
                frontLeft.setPower(y * pm);
                frontRight.setPower(-y * pm);
                backLeft.setPower(y * pm);
                backRight.setPower(y * pm);
            }
            // === TURNING ===
            else if (x > 0.1) {  // turn right
                frontLeft.setPower((x / 2) * pm);
                backLeft.setPower((-x / 2) * pm);
                frontRight.setPower((x / 2) * pm);
                backRight.setPower((x / 2) * pm);
            } else if (x < -0.1) { // turn left
                frontLeft.setPower((x / 2) * pm);
                backLeft.setPower((-x / 2) * pm);
                frontRight.setPower((x / 2) * pm);
                backRight.setPower((x / 2) * pm);
            }
            // === STRAFING ===
            else if (strafe > 0.1) {  // strafe right
                frontLeft.setPower(strafe * pm);
                backLeft.setPower(strafe * pm);
                frontRight.setPower(strafe * pm);
                backRight.setPower(-strafe * pm);
            } else if (strafe < -0.1) {  // strafe left
                frontLeft.setPower(strafe * pm);
                backLeft.setPower(strafe * pm);
                frontRight.setPower(strafe * pm);
                backRight.setPower(-strafe * pm);
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
            if (intakePower == 0.0) {
                intake.setPower(1.0);
            }
            if (intakePower == 1.0) {
                intake.setPower(-1.0);
            }
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

    private double getBearing(double tx) {// define bearing as tx
        return (tx);
    }
}

