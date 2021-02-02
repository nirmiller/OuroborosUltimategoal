package org.firstinspires.ftc.teamcode.DoobiLibraries;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdometryGlobalCoordinatePosition;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;


public abstract class TeleLib extends OpMode {


    final double COUNTS_PER_INCH = 308.876;

    double right_stick_x;
    double left_stick_x;
    double left_stick_y;

    double theta;

    boolean arcade = false;


    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private Servo intakemain;
    private Servo intakeclaw;
    private DcMotor shooter;
    private DcMotor pivot;
    private DcMotor lift;
    private DcMotor intake;
    private Servo whook;
    private Servo wobble;
    private Servo mag;
    static double OPEN = 0.0;
    static double CLOSED = 1;

    DcMotor verticalLeft, verticalRight, horizontal;
    String verticalLeftEncoderName = "fr", verticalRightEncoderName = "fl", horizontalEncoderName = "bl";

    double wobblePos = .5;
    double hookPos = .5;
    double magPos = 1;

    double[] motorPowers;

    public OdometryGlobalCoordinatePosition ogcp;
    public Thread global;
    private boolean magout = false;

    public Thread gamer_1;
    public Thread gamer_2;

    @Override
    public void init() {
        //Drive base

        //Init complete

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        pivot = hardwareMap.dcMotor.get("pivot");
        shooter = hardwareMap.dcMotor.get("shooter");
        lift = hardwareMap.dcMotor.get("lift");
        intake = hardwareMap.dcMotor.get("intake");
        //intakemain = hardwareMap.servo.get("intakemain");
        //intakeclaw = hardwareMap.servo.get("intakeclaw");

        whook = hardwareMap.servo.get("whook");
        wobble = hardwareMap.servo.get("wobble");
        mag = hardwareMap.servo.get("mag");

        wobble.setDirection(Servo.Direction.FORWARD);

        //intake.setDirection(DcMotor.Direction.FORWARD);
        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pivot.setDirection(DcMotor.Direction.REVERSE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        verticalLeft = fr;
        verticalRight = fl;
        horizontal = bl;

        theta = 0;

        motorPowers = new double[4];

        resetEncoders();

        ogcp = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 35);
        global = new Thread(ogcp);
        global.start();
        arcade = false;



    }


    public void resetEncoders() {

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    public void drive() {
        if (arcade && gamepad1.b) {
            arcade = false;
        } else if (!arcade && gamepad1.b) {
            arcade = true;
        }
        if (arcade) {
            arcadedrive();
        } else {
            holonomicdrive();
        }

        telemetry.addData("Drive ", arcade ? "Arcade" : "Holonomic");
    }


    public void arcadedrive() {
        theta = ogcp.returnOrientation();
        left_stick_y = gamepad1.left_stick_y;
        left_stick_x = gamepad1.left_stick_x;
        right_stick_x = gamepad1.right_stick_x;

        if (Math.abs(left_stick_x) > 0.05 ||
                Math.abs(left_stick_y) > 0.05 ||
                Math.abs(right_stick_x) > 0.05) {

            fl.setPower(left_stick_y - left_stick_x - right_stick_x);
            fr.setPower(left_stick_y - left_stick_x + right_stick_x);
            bl.setPower(left_stick_y + left_stick_x - right_stick_x);
            br.setPower(left_stick_y + left_stick_x + right_stick_x);

        } else {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }

        telemetry.addData("Angle : ", ogcp.returnOrientation());
        telemetry.addData("X Position : ", ogcp.returnXCoordinate());
        telemetry.addData("Y Position : ", ogcp.returnYCoordinate());

    }

    public void holonomicdrive() {
        left_stick_y = -gamepad1.left_stick_y;
        left_stick_x = gamepad1.left_stick_x;
        right_stick_x = gamepad1.right_stick_x;
        theta = -ogcp.returnOrientation();


        double[] motors = new double[4];
        motors = Holonomic.calcPowerTele(theta, right_stick_x, left_stick_x, left_stick_y);

        if (Math.abs(left_stick_x) > 0.05 ||
                Math.abs(left_stick_y) > 0.05 ||
                Math.abs(right_stick_x) > 0.05) {

            motors = Holonomic.calcPowerTele(theta, right_stick_x, left_stick_x, left_stick_y);

            fl.setPower(motors[0]);
            fr.setPower(motors[1]);
            bl.setPower(motors[2]);
            br.setPower(motors[3]);

        } else {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }

        telemetry.addData("Angle : ", theta);
        telemetry.addData("X Position : ", ogcp.returnXCoordinate());
        telemetry.addData("Y Position : ", ogcp.returnYCoordinate());


    }


    public void intake() {
        ElapsedTime runtime = new ElapsedTime();
        if (gamepad2.right_trigger > .5)
        {
            intake.setPower(1);
        }
        /*if(gamepad2.a && intakemain.getPosition()==.35)
        {
            runtime.reset();
            while(runtime.milliseconds() < 300){}
            intakemain.setPosition(1);
        }
        else if (gamepad2.a)
        {
            runtime.reset();
            while(runtime.milliseconds() < 300)
            {

            }
            intakemain.setPosition(.35);
        }

        if(gamepad2.b && intakeclaw.getPosition()==OPEN)
        {

            runtime.reset();
            while(runtime.milliseconds() < 300)
            {

            }
            intakeclaw.setPosition(CLOSED);
        }
        else if (gamepad2.b && intakeclaw.getPosition()==CLOSED)
        {

            runtime.reset();
            while(runtime.milliseconds() < 300)
            {

            }
            intakeclaw.setPosition(OPEN);
        }
        if (gamepad1.a)
        {
            intakemain.setPosition(0);
        }
*/
    }

    public void wobbleGoal() {
        if (gamepad2.y && whook.getPosition() == 0) {

            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    ElapsedTime time = new ElapsedTime();
                    while (gamepad2.y && time.milliseconds() < 300) { }
                    whook.setPosition(1);
                }
            });

            thread.start();

        } else if (gamepad2.y && whook.getPosition() != 0) {

            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    ElapsedTime time = new ElapsedTime();
                    while (gamepad2.y && time.milliseconds() < 300) {
                    }
                    whook.setPosition(0);
                }
            });

            thread.start();

        } else if (gamepad2.x && wobble.getPosition() == 0) {

            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    ElapsedTime time = new ElapsedTime();
                    while (gamepad2.x && time.milliseconds() < 300) {
                    }
                    wobble.setPosition(1);
                }
            });

            thread.start();

        } else if (gamepad2.x && wobble.getPosition() != 0) {

            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    ElapsedTime time = new ElapsedTime();
                    while (gamepad2.x && time.milliseconds() < 300) {
                    }
                    wobble.setPosition(0);
                }
            });
            thread.start();
        }
        telemetry.addData("hook position", whook.getPosition());
        telemetry.addData("wobble position", wobble.getPosition());
    }


    public void shooter() {
        if (gamepad2.right_bumper) {
            shooter.setPower(1);
        } else {
            shooter.setPower(0);
        }

        if (gamepad2.dpad_up) {
            pivot.setPower(.5);

        } else if (gamepad2.dpad_down) {
            pivot.setPower(-.5);
        }else
        {
            pivot.setPower(0);
        }

        telemetry.addData("pivot encoder pos: ", pivot.getCurrentPosition());

    }

    public void magazine() {

        double right_stick_y = gamepad2.right_stick_y;
        if (Math.abs(right_stick_y) > .05) {
            lift.setPower(right_stick_y / 2);
        } else {
            lift.setPower(0);
        }

        telemetry.addData("Lift Pos : ", lift.getCurrentPosition());

        if (gamepad2.left_bumper && !magout) {
            mag.setPosition(0);
            sleep(200);
            mag.setPosition(1);

        }
        telemetry.addData("Mag pos :", mag.getPosition());
    }



}
