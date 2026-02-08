package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;

public class LimelightMath {

    LLResult llResult;
    public double distanceFromTag() {
        if (llResult != null && llResult.isValid()) {

            double theta = llResult.getTy();
            //degrees
            double LLAngle = 20;
            theta += LLAngle;

            theta = Math.toRadians(theta);

            //mm
            double tagHeight = 762;
            //mm
            double LLHeight = 279.4;
            double LIMELIGHTDISTCONST_M = tagHeight - LLHeight;
            double DIST_M = LIMELIGHTDISTCONST_M / Math.tan(theta);
            double DIST_I = DIST_M / 25.4;

            return DIST_I;
        }
        return -1;
    }

}
