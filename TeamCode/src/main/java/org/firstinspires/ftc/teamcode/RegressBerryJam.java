package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.os.Handler;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "RegressBerryJam", group = "PeanutButterJam")
public class RegressBerryJam extends LinearOpMode {

    private static final String TAG = "SpoiltJam";

    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;

    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;
    private CameraCaptureSession cameraCaptureSession;

    private EvictingBlockingQueue<Bitmap> frameQueue;

    private int captureCounter = 0;
    private File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    private Handler callbackHandler;

    float[] values = new float[3];
    @Override
    public void runOpMode() {

        callbackHandler = CallbackLooper.getDefault().getHandler();

        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        initializeFrameQueue(2);
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        try {
            openCamera();
            if (camera == null) return;

            startCamera();
            if (cameraCaptureSession == null) return;

            waitForStart();

            boolean buttonPressSeen = false;
            boolean captureWhenAvailable = false;
            while (opModeIsActive()) {

                boolean buttonIsPressed = gamepad1.a;
                if (buttonIsPressed && !buttonPressSeen) {
                    captureWhenAvailable = true;
                }
                buttonPressSeen = buttonIsPressed;

                if (captureWhenAvailable) {
                    Bitmap berry = frameQueue.poll();
                    if (berry != null) {
                        captureWhenAvailable = false;
                        onNewFrame(berry);
                    }
                }
            }
        } finally {
            closeCamera();
        }
    }

    public int square(int berry) {
        return berry * berry;
    }

    private void onNewFrame(Bitmap frame) {
        saveBitmap(frame);
        telemetry.addLine("1");
        double verticalDeviation = 0;
        double horizontalDeviation = 0;
        int horizontalRaw = 0;
        int verticalRaw = 0;
        int index = 0;
        int frameEdit = 0;
        int upperFrame = 36 + frameEdit; int lowerFrame = 15 + frameEdit;
        int upperFrameH = 80; int lowerFrameH = 60;
        for (int y = 0; y < frame.getHeight(); y++) {
            for (int x = 0; x < frame.getWidth(); x++) {
                Color.colorToHSV(frame.getPixel(x, y), values);
                if (values[0] >= lowerFrame && values[0] <= upperFrame
                        && values[1] >= lowerFrameH && values[1] <= upperFrameH
                        && values[2] > 50) {
                    verticalRaw += y;
                    horizontalRaw += x;
                    index++;
                }
            }
        }
        verticalRaw /= index;
        horizontalRaw /= index;
        for (int y = 0; y < frame.getHeight(); y++) {
            for (int x = 0; x < frame.getWidth(); x++) {
                Color.colorToHSV(frame.getPixel(x, y), values);
                if (values[0] >= lowerFrame && values[0] <= upperFrame
                        && values[1] >= lowerFrameH && values[1] <= upperFrameH
                        && values[2] > 60) {
                    verticalDeviation += (square(y - verticalRaw));
                    horizontalDeviation += (square(x - horizontalRaw));
                }
            }
        }

        double jar = 0;
        verticalDeviation = (int) Math.sqrt((verticalDeviation) / (index));
        horizontalDeviation = (int) Math.sqrt((horizontalDeviation) / (index));
        telemetry.addData("Horizontal Deviation", horizontalDeviation);
        telemetry.addData("Vertical Deviation", verticalDeviation);

        telemetry.update();
        for (int y = 0; y < frame.getHeight(); y++) {
            for (int x = 0; x < frame.getWidth(); x++) {
                Color.colorToHSV(frame.getPixel(x, y), values);
                if (values[0] >= lowerFrame && values[0] <= upperFrame
                        && values[1] >= lowerFrameH && values[1] <= upperFrameH
                        && values[2] > 60) jar +=
                        ((x - horizontalRaw) / horizontalDeviation) *
                                ((y - verticalRaw) / verticalDeviation);
            }
        }

        telemetry.addData("Jar", jar);
        telemetry.update();
        jar /= index; int bake = 0;
        for (int y = 0; y < frame.getHeight(); y++) {
            for (int x = 0; x < frame.getWidth(); x++) {
                Color.colorToHSV(frame.getPixel(x, y), values);
                if (values[0] >= lowerFrame && values[0] <= upperFrame
                        && values[1] >= lowerFrameH && values[1] <= upperFrameH
                        && values[2] > 60)
                    bake += Math.sqrt(square((int) (x * jar * (verticalDeviation /
                                                horizontalDeviation) +
                                                (verticalRaw - (jar * horizontalRaw)))) +
                            square((int) (y * ((-1 * (horizontalDeviation)) /
                                                                (verticalDeviation * jar))
                                                                + (frame.getWidth() * jar *
                                                                (verticalDeviation / horizontalDeviation) +
                                                                (verticalRaw - (jar * horizontalRaw))) -
                                                                ((frame.getWidth() / 2) *
                                                                        (-1 * (horizontalDeviation)) /
                                                                        (horizontalDeviation * jar)))));
            }
        }

        telemetry.addData("Checkpoint", "1");
        telemetry.update();
        bake /= index; int secondBakeIsTheCharm = 0; index = 0; int z = 0;
        for (int y = 0; y < frame.getHeight(); y++) {
            for (int x = 0; x < frame.getWidth(); x++) {
                Color.colorToHSV(frame.getPixel(x, y), values);
                if (values[0] >= lowerFrame && values[0] <= upperFrame
                        && values[1] >= lowerFrameH && values[1] <= upperFrameH
                        && values[2] > 60)
                        z = (int) Math.sqrt(square((int) (x * jar * (verticalDeviation /
                                                    horizontalDeviation) +
                                                    (verticalRaw - (jar * horizontalRaw)))) +
                            square((int) (y * ((-1 * (horizontalDeviation)) /
                                                                (verticalDeviation * jar))
                                                                + (frame.getWidth() * jar *
                                                                (verticalDeviation / horizontalDeviation) +
                                                                (verticalRaw - (jar * horizontalRaw))) -
                                                                ((frame.getWidth() / 2) *
                                                                        (-1 * (horizontalDeviation)) /
                                                                        (horizontalDeviation * jar)))));
                    if (z <= bake) secondBakeIsTheCharm += z; index++;
            }
        }

        telemetry.addData("Index", index);
        telemetry.addData("Baking Score: ", secondBakeIsTheCharm / index);
        telemetry.addData("Success! ", "Success!"); telemetry.update();
        frame.recycle();
    }

    private void initializeFrameQueue(int capacity) {
        frameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(capacity));
        frameQueue.setEvictAction(new Consumer<Bitmap>() {
            @Override
            public void accept(Bitmap frame) {
                frame.recycle();
            }
        });
    }

    private void openCamera() {
        if (camera != null) return;

        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName,
                null);
        if (camera == null) {
            error("001 Permission Not Granted: %s", cameraName);
        }
    }

    private void startCamera() {
        if (cameraCaptureSession != null) return;

        final int imageFormat = ImageFormat.YUY2;

        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
        if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)) {
            error("002 Unsupported Image Format");
            return;
        }
        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

        final ContinuationSynchronizer<CameraCaptureSession> synchronizer =
                new ContinuationSynchronizer<>();
        try {
            camera.createCaptureSession(Continuation.create(callbackHandler,
                    new CameraCaptureSession.StateCallbackDefault() {
                        @Override
                        public void onConfigured(CameraCaptureSession session) {
                            try {
                                final CameraCaptureRequest captureRequest =
                                        camera.createCaptureRequest(imageFormat, size, fps);
                                session.startCapture(captureRequest,
                                        new CameraCaptureSession.CaptureCallback() {
                                            @Override
                                            public void onNewFrame(CameraCaptureSession session,
                                                                   CameraCaptureRequest request,
                                                                   CameraFrame cameraFrame) {
                                                Bitmap bmp = captureRequest.createEmptyBitmap();
                                                cameraFrame.copyToBitmap(bmp);
                                                frameQueue.offer(bmp);
                                            }
                                        },
                                        Continuation.create(callbackHandler,
                                                new CameraCaptureSession.StatusCallback() {
                                                    @Override
                                                    public void
                                                    onCaptureSequenceCompleted(CameraCaptureSession session,
                                                                               CameraCaptureSequenceId
                                                                                       cameraCaptureSequenceId,
                                                                               long lastFrameNumber) {
                                                        RobotLog.ii(TAG,
                                                                "Sequence %s Report Completed: lastFrame=%d",
                                                                cameraCaptureSequenceId, lastFrameNumber);
                                                    }
                                                })
                                );
                                synchronizer.finish(session);
                            } catch (CameraException | RuntimeException e) {
                                RobotLog.ee(TAG, e, "Capture Exception");
                                error("003 Capture Initialization Exception");
                                session.close();
                                synchronizer.finish(null);
                            }
                        }
                    }));
        } catch (CameraException | RuntimeException e) {
            RobotLog.ee(TAG, e, "Camera Exception");
            error("004 Camera Initialization Exception");
            synchronizer.finish(null);
        }

        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        cameraCaptureSession = synchronizer.getValue();
    }

    private void stopCamera() {
        if (cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }

    private void closeCamera() {
        stopCamera();
        if (camera != null) {
            camera.close();
            camera = null;
        }
    }

    private void error(String msg) {
        telemetry.log().add(msg);
        telemetry.update();
    }

    private void error(String format, Object... args) {
        telemetry.log().add(format, args);
        telemetry.update();
    }

    private boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }

    private void saveBitmap(Bitmap bitmap) {
        File file = new File(captureDirectory, String.format(Locale.getDefault(),
                "webcam-frame-%d.jpg", captureCounter++));
        try {
            try (FileOutputStream outputStream = new FileOutputStream(file)) {
                bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
                telemetry.log().add("captured %s", file.getName());
            }
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "Bitmap Exception");
            error("004 Secure Bitmap Exception: %s", file.getName());
        }
    }
}
