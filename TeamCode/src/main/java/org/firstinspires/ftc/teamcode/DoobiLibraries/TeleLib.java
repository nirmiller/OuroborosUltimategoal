package org.firstinspires.ftc.teamcode.DoobiLibraries;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.ThreadHandler;

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


    private DcMotor br;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor fl;
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
    double liftPower;
    double lift_pos;


    DcMotor verticalLeft, verticalRight, horizontal;
    String verticalLeftEncoderName = "fr", verticalRightEncoderName = "fl", horizontalEncoderName = "bl";

    double wobblePos = .5;
    double hookPos = .5;
    double magPos = 1;
    double position = 1;

    double[] motorPowers;

    public Thread global;
    private boolean magout = false;

    public Thread gamer_1;
    public Thread gamer_2;

    ThreadHandler th_wobble;
    ThreadHandler th_shooter;

    double kill_count;
    boolean lift_bottom;
    boolean lift_top;

    @Override
    public void init() {
        //Drive base

        //Init complete

        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");

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

        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        verticalLeft = fr;
        verticalRight = fl;
        horizontal = bl;

        theta = 0;
        position = 1;

        motorPowers = new double[4];

        resetEncoders();

        arcade = false;

        th_wobble = new ThreadHandler();
        th_shooter = new ThreadHandler();
        kill_count = 0;
        lift_bottom = true;
        lift_top = false;

        lift_pos = 0;
        liftPower = 0;
    }


    public void resetEncoders() {

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        left_stick_y = gamepad1.left_stick_y;
        left_stick_x = gamepad1.left_stick_x;
        right_stick_x = gamepad1.right_stick_x;

        if (Math.abs(left_stick_x) > 0.05 ||
                Math.abs(left_stick_y) > 0.05 ||
                Math.abs(right_stick_x) > 0.05) {

            fl.setPower(left_stick_y + left_stick_x + right_stick_x);
            fr.setPower(left_stick_y + left_stick_x - right_stick_x);
            bl.setPower(left_stick_y - left_stick_x + right_stick_x);
            br.setPower(left_stick_y - left_stick_x - right_stick_x);

        } else {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }
        telemetry.addData("fl encoder", fl.getCurrentPosition());
        telemetry.addData("fr encoder", fr.getCurrentPosition());
        telemetry.addData("bl encoder", bl.getCurrentPosition());
        telemetry.addData("br encoder", br.getCurrentPosition());


    }

    public void holonomicdrive() {
        left_stick_y = -gamepad1.left_stick_y;
        left_stick_x = gamepad1.left_stick_x;
        right_stick_x = gamepad1.right_stick_x;
        //theta = -ogcp.returnOrientation();


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

        //   telemetry.addData("Angle : ", theta);
        //  telemetry.addData("X Position : ", ogcp.returnXCoordinate());
        //   telemetry.addData("Y Position : ", ogcp.returnYCoordinate());


        if (gamepad2.left_bumper) {

            intakemain.setPosition(position - .1);
            position = intakemain.getPosition();


        }


    }


    public void killAll(){

        if(gamepad2.right_stick_button){
            kill_count++;
        }

        if(kill_count >= 2){
            th_shooter.th_kill();
            th_wobble.th_kill();
            pivot.setPower(0);
            lift.setPower(0);
            intake.setPower(0);
            shooter.setPower(0);
            kill_count = 0;
        }
    }


    public void intake() {
        ElapsedTime runtime = new ElapsedTime();
        double right_trigger = gamepad2.right_trigger;
        double left_trigger = gamepad2.left_trigger;
        if (right_trigger > .5) {
            intake.setPower(-right_trigger);
        } else if (left_trigger > .5) {
            intake.setPower(left_trigger);
        } else {
            intake.setPower(0);
        }
    }

    Thread whook_up = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            while (gamepad2.y && time.milliseconds() < 300) {
            }
            whook.setPosition(1);
        }
    });

    Thread whook_down = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            while (gamepad2.y && time.milliseconds() < 300) {
            }
            whook.setPosition(0);
        }
    });

    Thread wobble_down = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            while (gamepad2.x && time.milliseconds() < 300) {
            }
            wobble.setPosition(0);
        }
    });

    Thread wobble_up = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            while (gamepad2.x && time.milliseconds() < 300) {
            }
            wobble.setPosition(1);
        }
    });

    public void wobbleGoal() {
        if (gamepad2.y && whook.getPosition() == 0) {

            th_wobble.queue(whook_up);


        } else if (gamepad2.y && whook.getPosition() != 0) {

            th_wobble.queue(whook_down);

        } else if (gamepad2.x && wobble.getPosition() == 0) {

            th_wobble.queue(wobble_up);

        } else if (gamepad2.x && wobble.getPosition() != 0) {

            th_wobble.queue(wobble_down);
        }
        telemetry.addData("hook position", whook.getPosition());
        telemetry.addData("wobble position", wobble.getPosition());
        telemetry.addData("Wobble Thread", th_wobble.live());
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
        } else {
            pivot.setPower(0);
        }

        telemetry.addData("pivot encoder pos: ", pivot.getCurrentPosition());
        telemetry.addData("lift encoder pos: ", lift.getCurrentPosition());


    }



    Thread liftUp_thread = new Thread(new Runnable() {
        @Override
        public void run() {
            lift.setPower(.5);
            while(lift.getCurrentPosition() < 400){

            }
            lift.setPower(.2);
        }
    });

    Thread liftDown_thread = new Thread(new Runnable() {
        @Override
        public void run() {
            lift.setPower(-.3);
            sleep(1000);
            lift.setPower(0);
        }
    });



    public void magazine() {
        double right_stick_y = -gamepad2.right_stick_y;
        if (right_stick_y > .05 && !lift_top) {

            th_shooter.queue(liftUp_thread);
            lift_top = true;
            lift_bottom = false;


        } else if (right_stick_y < -.05 && !lift_bottom) {


            th_shooter.queue(liftDown_thread);
            lift_bottom = true;
            lift_top = false;
        }


        if (gamepad2.left_bumper && !magout) {
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    mag.setPosition(0);
                    sleep(200);
                    mag.setPosition(1);
                }
            });

            thread.start();
        }

    }


}
