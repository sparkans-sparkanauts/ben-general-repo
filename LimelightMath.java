package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;

public class LimelightMath {

    public double distanceFromTag(LLResult llresult) {

        if (llresult != null && llresult.isValid()) {


            double LLAngle = 20;

            double theta = llresult.getTy();
            theta += LLAngle;

            theta = Math.toRadians(theta);

            //mm
            double tagHeight = 762;
            //mm
            double LLHeight = 275.4;
            double LIMELIGHTCONSTANT = 1;



            double LIMELIGHTDISTCONST_M = tagHeight - LLHeight;
            double DIST_M = LIMELIGHTDISTCONST_M / Math.tan(theta);

            return (DIST_M / 25.4) - LIMELIGHTCONSTANT;
        }
        return -1;
    }

}
