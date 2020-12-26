package org.firstinspires.ftc.teamcode.DoobiLibraries;

public class Shooter {

        static final double g = 9.81;



        public static double calcThetaPivot(double d, double h, double v) {


            double theta_minus = Math.atan((v * v + Math.sqrt(Math.pow(v, 4) - g * (g * d * d + 2 * v * v * h))) / (g * d));


            theta_minus = Math.toDegrees(theta_minus);

            return theta_minus;
        }

        public static double calcThetaX(double x, double y, double GoalX, double GoalY)
        {
            double changeX = GoalX - x;
            double changeY = GoalY - y;

            double phi = Math.atan(changeY/changeX);
            return 90 - phi;


        }
}
