package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdomDriveTrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdometryGlobalCoordinatePosition;


/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "Odom w/ Hardware classes")
public class OdomWithHardwareClasses extends LinearOpMode {

    final double COUNTS_PER_INCH = 307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    OdomDriveTrain drive;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        drive = new OdomDriveTrain(this);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();


        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());


            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    public void goToPosition(double targetX, double targetY, double power, double orientation, double allowedDistanceError) {

        //distance to x and y for trig calculations
        double distanceToX = (targetX * COUNTS_PER_INCH)- globalPositionUpdate.returnXCoordinate();
        double distanceToY = (targetY * COUNTS_PER_INCH) - globalPositionUpdate.returnYCoordinate();

        //gets total distance needed to travel
        double distance = Math.hypot(distanceToX, distanceToY);

        while (opModeIsActive() && distance > allowedDistanceError) {
            distanceToX = targetX - globalPositionUpdate.returnXCoordinate();
            distanceToY = targetY - globalPositionUpdate.returnYCoordinate();

            //uses right triange trig to figure out what ange the robot needs to move at
            //maybe could integrate Nir's holonomic odom math into?
            double moveAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));

            //figures out what power to set the motors to so we can move at this angle
            double movementX = calculateX(moveAngle, power);
            double movementY = calculateY(moveAngle, power);

            double angleCorrection = orientation - globalPositionUpdate.returnOrientation();

        }
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
