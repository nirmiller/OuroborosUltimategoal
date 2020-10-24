package org.firstinspires.ftc.teamcode.DoobiLibraries;

public class Point {
    double X;
    double Y;
    double face;

    public Point(){
        X = 0;
        Y = 0;
        face = 0;
    }

    public Point(double x, double y, double face) {
        X = x;
        Y = y;
        this.face = face;
    }

    public double getX() {
        return X;
    }

    public double getY() {
        return Y;
    }

    public double getFace() {
        return face;
    }
}
