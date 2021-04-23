package org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.Holonomic;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Point;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Sensors;

import java.util.ArrayList;

public class OdomDriveTrain {
    //Drive motors
    DcMotor fr, br, fl, bl;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;
    Servo hook;
    Servo wobble;
    Servo mag;


    public OdometryGlobalCoordinatePosition globalPositionUpdate;
    public Thread global;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "fr", rbName = "br", lfName = "fl", lbName = "bl";
    String verticalLeftEncoderName = "fr", verticalRightEncoderName = "fl", horizontalEncoderName = "bl";
    double theta;

    final double COUNTS_PER_INCH = 35;

    LinearOpMode opMode;
    Sensors sensors;

    public OdomDriveTrain(LinearOpMode opMode)
    {
        this.opMode = opMode;
        sensors = new Sensors(opMode);

        fl = opMode.hardwareMap.dcMotor.get("fl");
        fr = opMode.hardwareMap.dcMotor.get("fr");
        bl = opMode.hardwareMap.dcMotor.get("bl");
        br= opMode.hardwareMap.dcMotor.get("br");


        //intake = hardwareMap.dcMotor.get("intake");

        hook = opMode.hardwareMap.servo.get("whook");
        wobble = opMode.hardwareMap.servo.get("wobble");
        mag = opMode.hardwareMap.servo.get("mag");


        //intake = hardwareMap.dcMotor.get("intake");


        mag = opMode.hardwareMap.servo.get("mag");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();

        verticalLeft = fr;
        verticalRight = fl;
        horizontal = bl;

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 12);

        global = new Thread(globalPositionUpdate);
        global.start();

        sensors = new Sensors(opMode);

        opMode.telemetry.addData("Status", "Hardware Map Init Complete");
        opMode.telemetry.update();
    }

    public void end()
    {
        choop();
        global.interrupt();
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

    public void splineMove(ArrayList<Point> spline, double power, double timeout, double allowed_error){
        for(Point p : spline)
        {
            goToPoint(p.getX(), p.getY(), 0, power, allowed_error, timeout);
        }
        choop();
    }



    public void timestrafeMove(double timeout, double power, double direction){

        ElapsedTime time = new ElapsedTime();
        setStrafePower(power);
        while(opMode.opModeIsActive() && time.milliseconds() < timeout  ){

        }
        choop();
    }

    public void timeMoveForward(double timeout, double power){


        ElapsedTime time = new ElapsedTime();
        setMotorsPower(power);
        while(opMode.opModeIsActive() && time.milliseconds() < timeout){

        }
        choop();


    }
    public void goToPoint(double targetX, double targetY, double face, double power,  double allowedDistanceError, double timeout) {

        ElapsedTime time = new ElapsedTime();
        time.reset();

        //distance to x and y for trig calculations
        double distanceToX = targetX - globalPositionUpdate.returnXCoordinate();
        double distanceToY = targetY - globalPositionUpdate.returnYCoordinate();

        //gets total distance needed to travel
        double distance = Math.hypot(distanceToX, distanceToY);
        double[] motor = new double[4];
        double k = 0;
        //distanceToX = targetX - globalPositionUpdate.returnXCoordinate();
       // distanceToY = targetY - globalPositionUpdate.returnYCoordinate();
        distance = Math.hypot(distanceToX, distanceToY);
        //uses right triange trig to figure out what ange the robot needs to move at
        //maybe could integrate Nir's holonomic odom math into?
        double moveAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));
        double angleCorrection = 0;


        while (opMode.opModeIsActive() && distance > allowedDistanceError && time.seconds() < timeout) {


            distanceToX = targetX - globalPositionUpdate.returnXCoordinate();
            distanceToY = targetY - globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToX, distanceToY);

            angleCorrection = face - globalPositionUpdate.returnOrientation();
            motor = Holonomic.calcPowerAuto(moveAngle, sensors.getGyroYaw(), 0);


            fl.setPower(motor[0] * power);
            fr.setPower(motor[1] * power);
            bl.setPower(motor[2] * power);
            br.setPower(motor[3] * power);



            //figures out what power to set the motors to so we can move at this angle
            opMode.telemetry.addData("Angle : ", sensors.getGyroYaw());
            opMode.telemetry.addData("X Position : ", globalPositionUpdate.returnXCoordinate());
            opMode.telemetry.addData("Y Position : ", globalPositionUpdate.returnYCoordinate());
            opMode.telemetry.update();

        }

       //choop();
    }

    public void turnPID(double angleChange, boolean turnRight, double kP, double kI, double kD, double timeout) {

        ElapsedTime time = new ElapsedTime();
        ElapsedTime timeoutTimer = new ElapsedTime();

        double error;
        double power;

        double proportional;
        double integral = 0;
        double derivative;

        double prevRunTime;

        double initAngle = sensors.getGyroYaw2();
        if (angleChange == 180)
        {
            initAngle = sensors.getGyroYaw();
        }
        double lastError = angleChange - Math.abs(sensors.getGyroYaw() - initAngle);

        time.reset();
        timeoutTimer.reset();

        while (Math.abs(sensors.getGyroYaw() - (angleChange + initAngle)) > .1 && timeoutTimer.seconds() < timeout && opMode.opModeIsActive()) {
            prevRunTime = time.seconds();


            error = angleChange - Math.abs(sensors.getGyroYaw() - initAngle);


            proportional = error * kP;

            integral += (error * (time.seconds() - prevRunTime)) * kI;


            derivative = ((error - lastError) / (time.seconds() - prevRunTime)) * kD;


            power = proportional + integral + derivative;

            if (power < .3 && kI == 0 && kD == 0)
            {
                power = .19;
            }

            turn(power, turnRight);

            opMode.telemetry.addData("error ", error);
            opMode.telemetry.addData("P", proportional);
            opMode.telemetry.addData("I", integral);
            opMode.telemetry.addData("D", derivative);
            opMode.telemetry.addData("power", power);
            opMode.telemetry.update();

            lastError = error;
           /* if (sensors.getGyroYaw2() < 360 && sensors.getGyroYaw() > 180)
            {
                initAngle = 360;
            }*/

            opMode.idle();


        }
        opMode.telemetry.addLine("exited");
        opMode.telemetry.update();

        choop();

    }
    public void turn(double power, boolean isRight)
    {
        if(isRight)
        {
            fr.setPower(-power);
            br.setPower(-power);
            fl.setPower(power);
            bl.setPower(power);
        }
        else
        {
            fr.setPower(power);
            br.setPower(power);
            fl.setPower(-power);
            bl.setPower(-power);
        }
    }

    public double getEncoderAverage() {
        double count = 4.0;
        if(fr.getCurrentPosition() == 0)
        {
            count--;
        }
        if(fl.getCurrentPosition() == 0)
        {
            count--;
        }
        if(br.getCurrentPosition() == 0)
        {
            count--;
        }
        if(bl.getCurrentPosition() == 0)
        {
            count--;
        }
        if(count == 0)
        {
            return 0;
        }
        return (fl.getCurrentPosition() + fr.getCurrentPosition()
                + br.getCurrentPosition() + bl.getCurrentPosition()) / count;
    }
    private double getStrafeEncoderAverage(double direction) {

        double count = 4.0;
        double average = 0;

        if(fr.getCurrentPosition() == 0)
        {
            count--;
        }
        if(fl.getCurrentPosition() == 0)
        {
            count--;
        }
        if(br.getCurrentPosition() == 0)
        {
            count--;
        }
        if(bl.getCurrentPosition() == 0)
        {
            count--;
        }
        if (count == 0)
        {
            return 0;
        }
        if(direction < 0)
        {
            average = (((-fl.getCurrentPosition() + -1*fr.getCurrentPosition()
                    + -br.getCurrentPosition() + bl.getCurrentPosition())) ) / count;
        }
        else if(direction > 0)
        {
            average = (((fl.getCurrentPosition() + -fr.getCurrentPosition()
                    + br.getCurrentPosition() + -1*-bl.getCurrentPosition())))  / count;
        }
        return average;
    }

    public void setMotorsPower(double power)
    {

        fl.setPower(power);
        fr.setPower(power);
        br.setPower(power);
        bl.setPower(power);
    }

    public void setStrafePower(double power)
    {
        fr.setPower(-power);
        br.setPower(-power);
        fl.setPower(-power);
        bl.setPower(power);
    }

        public double getTargetPercentile(double reading) {
        return Math.abs(getEncoderAverage() / reading);
    }
    public void encoderMove(double power, double distance, double runtimeS)
    {
        resetEncoders();
        ElapsedTime time = new ElapsedTime();

        double initEncoder = 0;

        time.reset();

        double finalPower = power;
        power = .2;
        double d = 0;
        while (d < distance && time.seconds() < runtimeS && opMode.opModeIsActive()) {
            d = Math.abs(getEncoderAverage() - initEncoder);
            if (d <= distance/1.25)
            {
                power+= .05;
            }else if(d > distance/1.25){
                power += -.025;
            }
            setMotorsPower(power);
            if (!opMode.opModeIsActive())
            {
                choop();
            }

            opMode.telemetry.addData("Encoder distance left", (distance - getEncoderAverage()));
            opMode.telemetry.update();

        }
        choop();
    }


    public void gyroStrafe(double power, double distance, boolean left, double timeoutMS)
    {
        distance = distance * COUNTS_PER_INCH;
        ElapsedTime time = new ElapsedTime();
        resetEncoders();
        double pos = -1;
        if(left)
        {
            pos = 1;
        }


        double pfr = -power * pos;
        double pfl = -power * pos;
        double pbl = power * pos;
        double pbr = power * pos;



        double angle = sensors.getGyroYaw();
        double average = 0;
        resetEncoders();
        opMode.telemetry.addData("Math.abs(average)", Math.abs(getStrafeEncoderAverage(pos)));
        opMode.telemetry.update();
        while(opMode.opModeIsActive() && ((distance) - Math.abs(getStrafeEncoderAverage(pos))) > 0 && time.milliseconds() < timeoutMS)
        {
            opMode.telemetry.addLine("entered");

            average = getStrafeEncoderAverage(pos);

            if(angle > 2)
            {
                if(!left)
                {
                    pfr = power;
                    pfl = power * .7;
                    pbr = -power;
                    pbl = -power * .7;
                }
                else if(left)
                {
                    pfr = -power *.7;
                    pfl = -power ;
                    pbr = power *.7;
                    pbl = power;
                }
            }
            else if(angle < -2)
            {
                if(!left)
                {
                    pfr = power * .7;
                    pfl = power;
                    pbr = -power * .7;
                    pbl = -power;
                }
                else if(left)
                {
                    pfr = -power;
                    pfl = -power*.7;
                    pbr = power ;
                    pbl = power *.7;
                }

            }
            else {
                pfr = -power * pos;
                pfl = -power * pos;
                pbl = power * pos;
                pbr = power * pos;
            }


            fr.setPower(pfr);
            fl.setPower(pfl);
            bl.setPower(pbl);
            br.setPower(pbr);
            angle = sensors.getGyroYaw();
            opMode.telemetry.addData("Angle", angle);
            opMode.telemetry.addData("Encoder distance left", ((distance * 4) - Math.abs(getStrafeEncoderAverage(pos))));
            opMode.telemetry.addData("Math.abs(average)", Math.abs(getStrafeEncoderAverage(pos)));
            opMode.telemetry.addData("Math.abs(distance * 5)", Math.abs(distance * 4));

            //opMode.sleep(1000);

            opMode.telemetry.update();
        }
        choop();
    }
    public void gyroTurnNinety(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 90;

        do {

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();
            if (sensors.getGyroYaw() < goal) {
                turn(.21, false);
            } else {
                turn(.21, true);
            }


        } while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > 2 && runtime.milliseconds() < timeOutMS);

        choop();
    }


    public void gyroTurnNinetyFast(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 90;

        do {

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();
            if (sensors.getGyroYaw() < goal && sensors.getGyroYaw() > 0) {
                turn(.7, false);
            } else {
                turn(.7, true);
            }


        } while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > 2 && runtime.milliseconds() < timeOutMS);

        choop();
    }

    public void gyroTurn270(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 270;

        do {

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();
            if (sensors.getGyroYaw() < goal) {
                turn(.27, false);
            } else {
                turn(.27, true);
            }


        } while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > .5 && runtime.milliseconds() < timeOutMS);

        choop();
    }

    public void gyroTurn270Fast(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 270;

        do {

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();

            turn(.7, true);


        } while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > 2 && runtime.milliseconds() < timeOutMS);

        choop();
    }

    public void gyroTurn180(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 180;


        while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) != 0  && runtime.milliseconds() < timeOutMS){

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();
            if(sensors.getGyroYaw() < goal) {
                turn(.3, false);
            } else {
                turn(.3, true);
            }


        }

        choop();
    }
    public void gyroTurn180Fast(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 180;
        if (sensors.getGyroYaw() > 0 && sensors.getGyroYaw() < 180) {
            goal = 180;
        }
        else {
            goal = 180;
        }


        do  {

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();
            if (sensors.getGyroYaw() < goal && sensors.getGyroYaw() > 0) {
                turn(.7, false);
            }
            else {
                turn(.7, true);
            }


        } while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > 2 && runtime.milliseconds() < timeOutMS);

        choop();
    }

    public void gyroTurnStraight(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 0;

        do {

            if (sensors.getGyroYaw() > 0 && sensors.getGyroYaw() < 180) {
                goal = 0;
            } else {
                goal = 360;
            }

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();
            if (sensors.getGyroYaw() < goal) {
                turn(.3, false);
            } else {
                turn(.3, true);
            }


        } while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) != 0 && runtime.milliseconds() < timeOutMS);

        choop();
    }
    public void gyroTurnStraightfast(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 0;

        while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > .1 && runtime.milliseconds() < timeOutMS) {

            if (sensors.getGyroYaw() > 0 && sensors.getGyroYaw() < 180) {
                goal = 0;
            } else {
                goal = 360;
            }

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();
            if (sensors.getGyroYaw() < goal) {
                turn(.7, false);
            } else {
                turn(.7, true);
            }


        } while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > 2 && runtime.milliseconds() < timeOutMS);

        choop();
    } 
    public void holoForward(double power, double distance, double timeoutMS){
        distance = distance * COUNTS_PER_INCH;
        ElapsedTime time = new ElapsedTime();
        resetEncoders();

        double[] motor_power = new double[4];


        double initial_angle = sensors.getGyroYawwwwwwwwwwwwwwwwwww();
        double average = 0;
        double angle_heading = 0;
        double angle_face = 0;
        resetEncoders();
        double d = 0;
        power = .25;
        while(opMode.opModeIsActive() &&  (distance) - Math.abs(getEncoderAverage()) > 0 && time.milliseconds() < timeoutMS)
        {
            average = getEncoderAverage();
            angle_face = initial_angle - sensors.getGyroYawwwwwwwwwwwwwwwwwww();


            d = Math.abs(getEncoderAverage());
            if (d <= distance/1.25)
            {
                power+= .25;
            }else if(d > distance/1.25){
                power += -.025;
            }

            motor_power = Holonomic.calcPowerAuto(angle_heading, angle_face, 0);
            fl.setPower(motor_power[0] * power);
            fr.setPower(motor_power[3] * power);
            bl.setPower(motor_power[2] * power);
            br.setPower(motor_power[1] * power);
            opMode.telemetry.addData("fl :", motor_power[0]);
            opMode.telemetry.addData("fr :", motor_power[3]);
            opMode.telemetry.addData("bl :", motor_power[2]);
            opMode.telemetry.addData("br :", motor_power[1]);

            opMode.telemetry.addData("Angle :", sensors.getGyroYawwwwwwwwwwwwwwwwwww());

            opMode.telemetry.update();
        }
        choop();
    }

    public void holoStrafe(double power, double distance, boolean left, double timeoutMS) {

        distance = distance * COUNTS_PER_INCH;
        ElapsedTime time = new ElapsedTime();
        resetEncoders();
        double pos = -1;
        if(left)
        {
            pos = 1;
        }



        double[] motor_power = new double[4];


        double initial_angle = sensors.getGyroYawwww();
        double average = 0;
        double angle_heading = 0;
        double angle_face = 0;
        resetEncoders();

        if(left){
            angle_heading = 90;
        }else{
            angle_heading = 270;
        }
        while(opMode.opModeIsActive() && ((distance) - Math.abs(getStrafeEncoderAverage(pos))) > 0 && time.milliseconds() < timeoutMS)
        {



            average = getStrafeEncoderAverage(pos);
            angle_face = initial_angle - sensors.getGyroYawwww();


            motor_power = Holonomic.calcPowerAuto(angle_heading, angle_face, 0);
            fl.setPower(motor_power[0] * power);
            fr.setPower(motor_power[3] * power);
            bl.setPower(motor_power[2] * power);
            br.setPower(motor_power[1] * power);
            opMode.telemetry.addData("fl :", motor_power[0]);
            opMode.telemetry.addData("fr :", motor_power[1]);
            opMode.telemetry.addData("bl :", motor_power[2]);
            opMode.telemetry.addData("br :", motor_power[3]);

            opMode.telemetry.addData("Angle :", sensors.getGyroYawwwwwwwwwwwwwwwwwww());

            opMode.telemetry.update();
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

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
    
    


}
