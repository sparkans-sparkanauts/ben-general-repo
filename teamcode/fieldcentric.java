package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

    public class fieldcentric extends OpMode {

        double forward, strafe, rotate;
        fieldcentricprereqs drive = new fieldcentricprereqs();


        @Override
        public void init(){

            drive.init(hardwareMap);

        }


        @Override
        public void loop() {
            forward = gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;


            drive.driveFieldRelitive(forward,strafe,rotate);


        }






    }
