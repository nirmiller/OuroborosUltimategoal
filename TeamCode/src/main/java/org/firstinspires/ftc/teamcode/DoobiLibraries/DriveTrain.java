package org.firstinspires.ftc.teamcode.DoobiLibraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveTrain {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;


    double odomRight;
    double odomLeft;
    double odomPerp;

    double theta;

    final double COUNTS_PER_INCH = 308.876;

    LinearOpMode linearOpMode;

    public DriveTrain(LinearOpMode linearOpMode)
    {
        this.linearOpMode = linearOpMode;

        fl = linearOpMode.hardwareMap.dcMotor.get("fl");
        fr = linearOpMode.hardwareMap.dcMotor.get("fr");
        bl = linearOpMode.hardwareMap.dcMotor.get("bl");
        br = linearOpMode.hardwareMap.dcMotor.get("br");

        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        theta = 0;
        resetEncoders();

    }
    public void resetEncoders() {

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private double cap(double a) {

        return Math.round(a*100)/100.0;

    }



    private void normalize(double[] val)
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

    public void driveTest(double angle, double face, double power, double timeout)
    {
        ElapsedTime time = new ElapsedTime();


        time.reset();

        while(time.seconds() < timeout && linearOpMode.opModeIsActive())
        {
            double[] motor = new double[4];
            double b = Math.toRadians(theta - angle);

            motor[0] = power*(Math.cos(b) - Math.sin(b));
            motor[1] = power*(Math.cos(b) + Math.sin(b));
            motor[2] = power*(Math.cos(b) + Math.sin(b));
            motor[3] = power*(Math.cos(b) - Math.sin(b));
            normalize(motor);

            odomLeft = -bl.getCurrentPosition() / COUNTS_PER_INCH;
            odomRight = -br.getCurrentPosition() / COUNTS_PER_INCH;

            theta = ((odomLeft - odomRight)/8.25) % (2 * Math.PI);
            linearOpMode.telemetry.addData("Angle", Math.toDegrees(theta));
            linearOpMode.telemetry.update();


            if(time.seconds() >= timeout/2)
            {
                fl.setPower(motor[0] * .5);
                fr.setPower(motor[1] * .5);
                bl.setPower(motor[2] * .5);
                br.setPower(motor[3] * .5);
            }
            else
            {
                fl.setPower(motor[0]);
                fr.setPower(motor[1]);
                bl.setPower(motor[2]);
                br.setPower(motor[3]);
            }

        }

        choop();


    }


    public void choop()
    {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }



}
