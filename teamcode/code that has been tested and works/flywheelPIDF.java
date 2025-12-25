package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class flywheelPIDF extends OpMode {

    public DcMotorEx flyWheel;

    public double highVelocity = 2500;

    public double lowVelocity = 900;

    public double curTargetVelocity = highVelocity;

    double P = 0;
    double F = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;



    @Override
    public void init(){

        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine( "Init Complete");

    }


    @Override
    public void loop() {

        //get gamepad commands
        //set target velocity
        //update telemetry

    if (gamepad1.yWasPressed()){
        if (curTargetVelocity == highVelocity) {
            curTargetVelocity = highVelocity;

             } else {curTargetVelocity = lowVelocity; }
        }

        if (gamepad1.bWasPressed()) {

            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];

        }

        if (gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }


        //set new pid coeff
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        //set velocity

        flyWheel.setVelocity(curTargetVelocity);

        double curVelocity = flyWheel.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f",  curVelocity);
        telemetry.addData("Error", "%.2f",  error);
        telemetry.addData("Tuning P", "%.4f (D-Pad u/d)", P );
        telemetry.addData("Tuning F", "%.4f (D-Pad l/r)", F );
        telemetry.addData("Step Size", "%.4f (B button)", stepSizes[stepIndex]);
    }
}
