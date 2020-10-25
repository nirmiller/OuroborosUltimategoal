package org.firstinspires.ftc.teamcode.DoobiLibraries;

public class Holonomic {




    /*
        motorPower[0] = front left motor        0 : (fl)
        motorPower[1] = front right motor       1 : (fr)
        motorPower[2] = back left motor         2 : (bl)
        motorPower[3] = back right motor        3 : (br)
     */
    private static double cap(double a) {

        return Math.round(a*10000)/10000.0;

    }



    public static void normalize(double[] val)
    {
        //check if data needs normalization
        boolean f = false;
        for(double g : val)
        {
            if(g > 1 ||  g < -1)
            {
                f = true;
            }
        }

        if(f){
            //get max and min
            double max = 0;
            for(double a : val)
            {
                if(Math.abs(a) > max)
                {
                    max = a;
                }
            }

            //normalize
            for(int i = 0; i < val.length; i++)
            {
                val[i] = cap(val[i] / Math.abs(max));
            }
        }
        f = false;

    }
    public static double floor(double a)
    {
        if(a > 1)
        {
            a = 1.0;
        }
        else if(a < -1)
        {
            a = -1;
        }
        return a;
    }

    public static double[] calcPowerAuto(double heading, double face) {
        double difAng = Math.toRadians(face - heading);

        double[] motorPower = new double[4];

        motorPower[0] = Math.cos(difAng) - Math.sin(difAng);
        motorPower[1] = Math.cos(difAng) + Math.sin(difAng);
        motorPower[2] = Math.cos(difAng) + Math.sin(difAng);
        motorPower[3] = Math.cos(difAng) - Math.sin(difAng);

        normalize(motorPower);

        return motorPower;

    }

    public static double[] calcPowerTele(double face, double rotationPower, double X, double Y) {
        double[] motorPower = new double[4];
        face = Math.toRadians(face);

        double x_comp = Y*Math.cos(face) + X*Math.sin(face);
        double y_comp = Y*Math.sin(face) - X*Math.cos(face);
        double rot_comp = rotationPower;
        motorPower[0] = x_comp - y_comp - rot_comp;
        motorPower[1] = x_comp + y_comp + rot_comp;
        motorPower[2] = x_comp + y_comp - rot_comp;
        motorPower[3] = x_comp - y_comp + rot_comp;

        return motorPower;
    }

}
