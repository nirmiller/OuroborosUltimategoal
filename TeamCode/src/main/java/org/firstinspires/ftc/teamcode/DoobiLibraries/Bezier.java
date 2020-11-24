package org.firstinspires.ftc.teamcode.DoobiLibraries;

import java.util.ArrayList;


public class Bezier {


    public static ArrayList<Point> interpolateSpline(double[][] variables) {
        ArrayList<Point> bezierPoints = new ArrayList<Point>();

        double x = 0;
        double y = 0;
        for (double t = 0; t < 1; t = t + .1) {

            x = functionX(variables[0][0], variables[0][1], variables[0][2], variables[0][3], t);
            y = functionY(variables[1][0], variables[1][1], variables[1][2], variables[1][3], t);
            if(y == 0 && x == 0)
            {
                x = functionX(variables[0][0], variables[0][1], variables[0][2], variables[0][3], t+.01);
                y = functionY(variables[1][0], variables[1][1], variables[1][2], variables[1][3], t+.01);
            }
            bezierPoints.add(new Point(x, y, Math.toDegrees(Math.atan(x/y))));
        }

        return bezierPoints;
    }

    private static double functionY(double y0, double ay1, double ay2, double y2, double t) {
        return Math.pow(1 - t, 3) * y0 + 3 * ay1 * t * Math.pow((1 - t), 2) + 3 * ay2 * t * t * (1 - t) + y2 * t * t * t;
    }

    private static double functionX(double x0, double ax1, double ax2, double x2, double t) {
        return Math.pow(1 - t, 3) * x0 + 3 * ax1 * t * Math.pow((1 - t), 2) + 3 * ax2 * t * t * (1 - t) + x2 * t * t * t;
    }

    public static double[][] getVariables(double x0, double y0, double x1, double y1, double x2, double y2) {
        double[][] variables = new double[2][4];


        double Ax1 = x1;
        double Ay1 = y1;

        double Ax2 = 8 * x1 / 3 - x0 / 3 - x2 / 3 - Ax1;
        double Ay2 = 8 * y1 / 3 - y0 / 3 - y2 / 3 - Ay1;


        double a = x0;
        double e = y0;

        double b = Ax1;
        double f = Ay1;

        double c = Ax2;
        double g = Ay2;

        double d = x2;
        double h = y2;

        variables[0] = new double[]{a, b, c, d};
        variables[1] = new double[]{e, f, g, h};


        return variables;
    }


}


