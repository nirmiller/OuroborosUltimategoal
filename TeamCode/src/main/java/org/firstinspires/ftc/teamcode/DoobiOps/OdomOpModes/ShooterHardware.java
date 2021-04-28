package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.Sensors;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Shooter;

import static android.os.SystemClock.sleep;

public class ShooterHardware {

    public static double GOAL_X_BLUE = 0.0;
    public static double GOAL_Y_BLUE = 0.0;
    public static double GOAL_Z_BLUE = 0.0;
    public static double GOAL_X_RED = 0.0;
    public static double GOAL_Y_RED = 0.0;
    public static double GOAL_Z_RED = 0.0;

    public static double INITIAL_Z = 0.0;

    public boolean liftReady;
    static double PIVOT_THETA_TO_ENCODER = 0.0;
    static double LIFT_INDEX_TO_ENCODER = 0.0;
    static double POWER_TO_VELOCITY = 0.0;

    LinearOpMode opMode;

    DcMotor shooter;
    DcMotor pivot;
    DcMotor lift;
    Servo mag;
    Servo pivotStop;
    Sensors sensors;

    private double currentAngle;

    private double currentLiftHeightIndex;

    public ShooterHardware(LinearOpMode opMode)
    {
        this.opMode = opMode;
        sensors = new Sensors(opMode);

        shooter = opMode.hardwareMap.dcMotor.get("shooter");
        pivot = opMode.hardwareMap.dcMotor.get("pivot");
        lift = opMode.hardwareMap.dcMotor.get("lift");
        mag = opMode.hardwareMap.servo.get("mag");
        pivotStop = opMode.hardwareMap.servo.get("ps");


        pivot.setDirection(DcMotor.Direction.REVERSE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        /*setLift();
        sleep(1000);
        setPivotAngle();
        while (!sensors.button.isPressed() && opMode.opModeIsActive())
        {
            pivot.setPower(-.2);
        }*/
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // pivot.setPower(0);

        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftReady = false;
        double encoder = 15;

        while (lift.getCurrentPosition() < encoder)
        {
            lift.setPower(.7);
            opMode.telemetry.addData("lift encoder pos: ", lift.getCurrentPosition());
            opMode.telemetry.update();

        }
        liftReady = true;


        lift.setPower(.4);
        opMode.sleep(500);
        while (!liftReady) {
        }
        encoder = 1000;
        while (pivot.getCurrentPosition() < encoder)
        {
            pivot.setPower(.7);
        }
        pivot.setPower(0);
        opMode.sleep(700);
        pivotStop.setPosition(.535);
        opMode.sleep(700);
        pivot.setPower(-0.05);
        opMode.sleep(1000);
        pivot.setPower(0);
        opMode.sleep(1000);
        //pivot.setPower(.05);
        //lift.setPower(0);
        opMode.telemetry.addLine("shooterRedy");

    }


    public void setPivotAngle()
    {
        double encoder = 800;
        while (pivot.getCurrentPosition() < encoder && opMode.opModeIsActive())
        {
            pivot.setPower(1);
        }
        pivot.setPower(0);
        //opMode.telemetry.addData("pivot encoder pos: ", pivot.getCurrentPosition());
        opMode.telemetry.update();

    }public void setPivotAngle2()
    {
        double encoder = pivot.getCurrentPosition() - 30;
        while (pivot.getCurrentPosition() > encoder && opMode.opModeIsActive())
        {
            pivot.setPower(-.1);
        }
        pivot.setPower(0);
        //opMode.telemetry.addData("pivot encoder pos: ", pivot.getCurrentPosition());
        opMode.telemetry.update();

    }
    public void setPivotAngle3()
    {
        double encoder = pivot.getCurrentPosition() - 20;
        while (pivot.getCurrentPosition() > encoder && opMode.opModeIsActive())
        {
            pivot.setPower(-.1);
        }
        pivot.setPower(0);
        //opMode.telemetry.addData("pivot encoder pos: ", pivot.getCurrentPosition());
        opMode.telemetry.update();

    }



    public void pivotDown()
    {
        double encoder = 0;
        while (pivot.getCurrentPosition() > encoder && opMode.opModeIsActive())
        {
            pivot.setPower(-.3);
        }
        pivot.setPower(0);
        //opMode.telemetry.addData("pivot encoder pos: ", pivot.getCurrentPosition());
        opMode.telemetry.update();

    }

    public void pivotPID (double pos, boolean moveUp, double kP, double kI, double kD, double timeout)
    {
        ElapsedTime time = new ElapsedTime();
        ElapsedTime timeoutTimer = new ElapsedTime();

        double error;
        double power;

        double proportional;
        double integral = 0;
        double derivative;

        double prevRunTime;

        double initPos = pivot.getCurrentPosition();

        double lastError = pos - initPos;

        time.reset();
        timeoutTimer.reset();
        while (Math.abs(pos - pivot.getCurrentPosition()) > 10 && timeoutTimer.seconds() < timeout && opMode.opModeIsActive()) {
            prevRunTime = time.seconds();

            error = pos - pivot.getCurrentPosition();

            proportional = error * kP;

            integral += (error * (time.seconds() - prevRunTime)) * kI;


            derivative = ((error - lastError) / (time.seconds() - prevRunTime)) * kD;


            power = proportional + integral + derivative;

            if (moveUp)
            {
                pivot.setPower(power);
            }
            else
            {
                pivot.setPower(-power);
            }

            opMode.telemetry.addData("error ", error);
            opMode.telemetry.addData("P", proportional);
            opMode.telemetry.addData("I", integral);
            opMode.telemetry.addData("D", derivative);
            opMode.telemetry.addData("power", power);
            opMode.telemetry.update();

            lastError = error;

            opMode.idle();
            if (Math.abs(pos - pivot.getCurrentPosition()) < 10)
            {
                break;
            }

        }
        opMode.telemetry.addLine("exited");
        opMode.telemetry.update();

        pivot.setPower(0);
    }

    public void setLift()
    {
        //TODO: encoder pos now 300 so make sure to start the auto at the highest pos the lift can be without falling
        double encoder = 300;
        while (lift.getCurrentPosition() < encoder && opMode.opModeIsActive())
        {
            if (lift.getCurrentPosition() < (encoder)) {
                lift.setPower(.7);
            }
            else
            {
                lift.setPower(.5);
            }
            //opMode.telemetry.addData("lift encoder pos: ", lift.getCurrentPosition());
            opMode.telemetry.update();

        }
        liftReady = true;


        if (!opMode.opModeIsActive()){
            lift.setPower(0);
        }else{
            lift.setPower(.19);
        }


    }

    public void ignite()
    {
        shooter.setPower(1);
    }

    public void withdraw()
    {
        shooter.setPower(0);
    }
    public void resetShooter()
    {
        double angleChange = 0 - currentAngle;
        double indexChange = 0 - currentLiftHeightIndex;

        //Move pivotdown
        //Move Lift Down



        currentAngle = 0;
        currentLiftHeightIndex = 0;
    }

    public void shoot(double current_x, double current_y, double goal_x, double goal_y, double goal_z)
    {
        //EVERYTHING IS IN METERS!!!
        double distance = Math.sqrt(Math.pow((goal_x - current_x), 2) + Math.pow((goal_y - current_y), 2));
        double z = goal_z - INITIAL_Z;

        double theta = Shooter.calcThetaPivot(distance, z, 1 * POWER_TO_VELOCITY);

        ignite();
        //loadShooter();



    }

    public void hitRing() {

        mag.setPosition(0);
        opMode.sleep(200);
        mag.setPosition(1);
    }

}
