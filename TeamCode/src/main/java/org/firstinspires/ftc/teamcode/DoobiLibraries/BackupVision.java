package org.firstinspires.ftc.teamcode.DoobiLibraries;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.ArrayList;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

public class BackupVision {
    LinearOpMode opMode;
    private VuforiaLocalizer vuforia;

    public static final String vuKey =
            "AdzMYbL/////AAABmflzIV+frU0RltL/ML+2uAZXgJiI" +
                    "Werfe92N/AeH7QsWCOQqyKa2G+tUDcgvg8uE8QjHeBZPcpf5hAwlC5qCfvg76eBoaa2b" +
                    "MMZ73hmTiHmr9fj3XmF4LWWZtDC6pWTFrzRAUguhlvgnck6Y4jjM16Px5TqgWYuWnpcxNM" +
                    "HMyOXdnHLlyysyE64PVzoN7hgMXgbi2K8+pmTXvpV2OeLCag8fAj1Tgdm/kKGr0TX86aQsC2" +
                    "RVjToZXr9QyAeyODi4l1KEFmGwxEoteNU8yqNbBGkPXGh/+IIm6/s/KxCJegg8qhxZDgO8110F" +
                    "RzwA5a6EltfxAMmtO0G8BB9SSkApxkcSzpyI0k2LxWof2YZG6x4H";

    public void initVision(LinearOpMode opMode) {
        this.opMode = opMode;

        // variable allows image to show up on robot controller phone
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        opMode.hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // original vuforia license key
        parameters.vuforiaLicenseKey = vuKey;
        // hardware mapping of webcam device
        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        // start vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // set RGB format to 565
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        // allowing the frame to only be 3 images at a time
        vuforia.setFrameQueueCapacity(3);
        opMode.telemetry.addLine("Vision init");
    }

    public BackupVision(LinearOpMode opMode) {
        this.opMode = opMode;

        // variable allows image to show up on robot controller phone
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        opMode.hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // original vuforia license key
        parameters.vuforiaLicenseKey = vuKey;
        // hardware mapping of webcam device
        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        // start vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // set RGB format to 565
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        // allowing the frame to only be 3 images at a time
        vuforia.setFrameQueueCapacity(3);
        opMode.telemetry.addLine("Vision init");
        opMode.telemetry.update();
    }
    public Bitmap getBitmap() throws InterruptedException{
        // method to actually capture frame
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        Image rgb = frame.getImage(1);

        long numImages = frame.getNumImages();

        // go through all images taken in frame and find ones that match correct format
        for (int i = 0; i < numImages; i++) {
            int fmt = frame.getImage(i).getFormat();

            if (fmt == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;

            }
            else {
                opMode.telemetry.addLine("Didn't find correct rgb format");
                opMode.telemetry.update();

            }

        }

        // create bitmap
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        frame.close();

        opMode.telemetry.addLine("Got Bitmap");
        opMode.telemetry.addData("width", rgb.getWidth());
        opMode.telemetry.addData("height", rgb.getHeight());
        opMode.telemetry.update();

        opMode.sleep(500);

        return bm;
    }
//only using blue for now
    public int senseBlue(LinearOpMode opMode) throws InterruptedException {

        Bitmap bitmap = getBitmap();
        ArrayList<Integer> StoneX = new ArrayList<Integer>();

        int pos = 0;
        int stonexAvg = 0;

        // top left = (0,0)

        while(opMode.opModeIsActive()) {

            // can change this but prob not neccesary
            //TODO: may need to change if sensing other yellow stuff
            for (int colNum = 0; colNum < bitmap.getWidth(); colNum++) {

                //Shouldnt need to change this, but the colors in the backround maybe might mess it up\
                //TODO: may need to change if sensing other yellow stuff
                for (int rowNum = 0; rowNum < bitmap.getHeight(); rowNum++) {
                    int pixel = bitmap.getPixel(colNum, rowNum);

                    // receive R, G, and B values for each pixel
                    int redPixel = red(pixel);
                    int greenPixel = green(pixel);
                    int bluePixel = blue(pixel);

                    /*opMode.telemetry.addData("Red", redPixel);
                    opMode.telemetry.addData("Green", greenPixel);
                    opMode.telemetry.addData("Blue", bluePixel);
                    opMode.telemetry.update();*/

                    // only add y-coordinates of yellow pixels to list
                    //TODO: Test whether these are the right RGB values for yellow
                    if (redPixel > 140 && greenPixel > 100 && bluePixel < 60) {
                        StoneX.add(rowNum);
                    }

                }
            }


            // counts how many yellow pixels there
            for (int x : StoneX) {
                stonexAvg++;
            }
/*
            if(StoneX.size() > 0) stonexAvg /= StoneX.size();
            else{
                stonexAvg = 550;
                opMode.telemetry.addData("Failed", "Divided by Zero");
            }
*/
            // get average x-coordinate value of all yellow pixels
            //TODO: note the num pixels for each position
            opMode.telemetry.addData("AVG X = ", stonexAvg);
            opMode.telemetry.update();
            opMode.sleep(2000);
//calculates based on pixel size
            //TODO: Change these numbers based on testing how many pixels there are
            if (stonexAvg < 2000 && stonexAvg > 1800) {
                pos = 1;
            } else if (stonexAvg > 2000) {
                pos = 2;
            } else {
                pos = 0;
            }

            //TODO: may need to comment this out when testing for the num of pixels
            opMode.telemetry.addData("Position", pos);
            opMode.telemetry.update();
            break;
        }
        return pos;
    }
//no need
    public String senseRed(LinearOpMode opMode) throws InterruptedException {

        Bitmap bitmap = getBitmap();
        ArrayList<Integer> StoneX = new ArrayList<Integer>();

        String pos = "";
        int stonexAvg = 0;

        // top left = (0,0)
        while (opMode.opModeIsActive()) {

            opMode.telemetry.addData("width", bitmap.getWidth());
            opMode.telemetry.addData("height", bitmap.getHeight());


            // scan 3 columns
            for (int colNum = bitmap.getWidth(); colNum < bitmap.getWidth(); colNum++) {

                for (int rowNum = (bitmap.getHeight() / 2) + 50; rowNum < (bitmap.getHeight() / 2) + 200; rowNum++){
                    int pixel = bitmap.getPixel(colNum, rowNum);

                    // receive R, G, and B values for each pixel
                    int redPixel = red(pixel);
                    int greenPixel = green(pixel);
                    int bluePixel = blue(pixel);

                  /*  opMode.telemetry.addData("Red", redPixel);
                    opMode.telemetry.addData("Green", greenPixel);
                    opMode.telemetry.addData("Blue", bluePixel);
                    opMode.telemetry.update();*/
                    // only add x-coordinates of black pixels to list

                    if (redPixel < 30 && greenPixel < 30 && bluePixel < 30) {
                        StoneX.add(colNum);
                    }
                }
            }


            // get sum of all black pixels' x coordinates
            for (int x : StoneX) {
                stonexAvg += x;
            }

            if(StoneX.size() > 0) stonexAvg /= StoneX.size();
            else{
                stonexAvg = 600;
                opMode.telemetry.addData("Failed", "Divided by Zero");
            }



            // get average x-coordinate value of all black pixels
            opMode.telemetry.addData("Num Pixels = ", stonexAvg);


            if (stonexAvg < 540) {
                pos = "left";
            } else if (stonexAvg > 700) {
                pos = "right";
            } else {
                pos = "center";
            }

            opMode.telemetry.addData("AVG X = ", stonexAvg);
            //opMode.telemetry.addData("Position", pos);
            opMode.telemetry.update();
            break;
        }

        return pos;
    }


    public Bitmap vufConvertToBitmap(Frame frame) { return vuforia.convertFrameToBitmap(frame); }

}
