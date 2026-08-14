package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.DMatch;
import org.opencv.core.CvType;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
@Configurable
@TeleOp(name="VSLAM")
public class opmode extends OpMode {
    Servo wheel1,wheel2,pan,tilt;
    OpenCvCamera webcam;
    TelemetryManager.TelemetryWrapper panelsTelemetry;
    CameraStreamSource source;
    public static double wheel1Pos=0.5,wheel2Pos=0.5,panPos=0.5,tiltPos=0.3;
    public static double brightnessTolerance = 0.2;
    public static int consecutiveBrightnessTol = 11;
    public static double matchRatio = 0.7;
    public static int maxDescriptorDistance = 2500;
    public static int ransacInt = 30;
    public static double ransacThresh = 5;

    public static ArrayList<Feature> features = new ArrayList<>();
    public static ArrayList<Match> matches = new ArrayList<>();
    @Override
    public void init() {
        wheel1 = hardwareMap.get(Servo.class,"wheel1");
        wheel2 = hardwareMap.get(Servo.class,"wheel2");
        pan = hardwareMap.get(Servo.class,"pan");
        tilt = hardwareMap.get(Servo.class,"tilt");
        panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class,"c270");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMonitorViewId);
        CameraPipeline pipeline = new CameraPipeline();

        webcam.setPipeline(pipeline);
        source = pipeline;

        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webcam.startStreaming(
                                640,480, OpenCvCameraRotation.UPRIGHT
                        );

                        PanelsCameraStream.INSTANCE.startStream(source,60);
                        panelsTelemetry.addData("Camera:", "Opened");
                        panelsTelemetry.update();

                    }

                    @Override
                    public void onError(int errorCode) {
                        panelsTelemetry.addData("Error: ",errorCode);
                        panelsTelemetry.update();
                    }
                }
        );
    }


    @Override
    public void init_loop(){
        wheel1.setPosition(wheel1Pos);
        wheel2.setPosition(wheel2Pos);
        tilt.setPosition(tiltPos);
        pan.setPosition(panPos);

        panelsTelemetry.addData("Status: ","Init");
        panelsTelemetry.update();
    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        wheel1.setPosition(wheel1Pos);
        wheel2.setPosition(wheel2Pos);
        tilt.setPosition(tiltPos);
        pan.setPosition(panPos);

        panelsTelemetry.addData("feature count: ", features.size());
        panelsTelemetry.addData("matches: ", matches.size());
        panelsTelemetry.addData("fps",webcam.getFps());
        panelsTelemetry.addData("camera: ", "running");
        panelsTelemetry.update();

    }

    @Override
    public void stop(){
        PanelsCameraStream.INSTANCE.stopStream();
        if (webcam!=null){
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }



    public static class CameraPipeline extends OpenCvPipeline implements CameraStreamSource {

        private byte[] pixels;
        private final int[][] rad3circle = {
                {0, -3}, //13
                {1, -3}, //14
                {2, -2}, //15
                {3, -1}, //16
                {3, 0}, //1
                {3, 1}, //2
                {2, 2}, //3
                {1, 3}, //4
                {0, 3}, //5
                {-1, 3}, //6
                {-2, 2}, //7
                {-3, 1}, //8
                {-3, 0}, //9
                {-3, -1}, //10
                {-2, -2}, //11
                {-1, -3} //12
        };
        private ArrayList<Feature> previousFeatures = new ArrayList<>();
        private ArrayList<Feature> currentFeatures = new ArrayList<>();
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(
                        Bitmap.createBitmap(
                                1, 1, Bitmap.Config.RGB_565
                        )
                );

        @Override
        public Mat processFrame(Mat input) {
            /**
             * Processing
             */
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY); //greyscale
            //read through all pixels and obtain brightness of each

            int width = input.cols();
            int height = input.rows();
            if (pixels == null || pixels.length != width * height) {
                pixels = new byte[width * height];
            }
            input.get(0, 0, pixels);
            //feature extraction
            currentFeatures.clear();
            for (int y = 3; y < input.rows() - 3; y += 4) {
                for (int x = 3; x < input.cols() - 3; x += 4) {
                    if (isFeaturePoint(pixels, width, x, y, consecutiveBrightnessTol)) {
                        Point point = new Point(x, y);
                        byte[] descriptor = createDescriptor(pixels, width, x, y);
                        currentFeatures.add(new Feature(point, descriptor));
//                        input.put(y,x,255);
                    }
                }
            }
            //feature matching
            if (!previousFeatures.isEmpty()) {
                matches = ransacFilter(matchFeatures(previousFeatures, currentFeatures));
            } else {
                matches.clear();
            }

            features.clear();
            features.addAll(currentFeatures);
            Imgproc.cvtColor(input, input, Imgproc.COLOR_GRAY2RGB);

            for (Match match : matches) {
                Point previous = match.previous.point;
                Point current = match.current.point;
                Imgproc.circle(input, previous, 4, new Scalar(255, 0, 0), 1);
                Imgproc.line(input, previous, current, new Scalar(0, 255, 255), 1);

            }

            Bitmap bitmap = Bitmap.createBitmap(
                    input.width(),
                    input.height(),
                    Bitmap.Config.RGB_565
            );
            Utils.matToBitmap(input, bitmap);
            lastFrame.set(bitmap);
            previousFeatures = new ArrayList<>(currentFeatures);

            return input;
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(
                    bitmapConsumer ->
                            bitmapConsumer.accept(lastFrame.get())
            );
        }

        public boolean isFeaturePoint(byte[] pixels, int width, int x, int y, int N) {

            int centerIdx = y * width + x;
            int centerBrightness = pixels[centerIdx] & 0xFF;

            int threshold = Math.max(20, (int) (centerBrightness * brightnessTolerance));
            int high = centerBrightness + threshold;
            int low = centerBrightness - threshold;
            //local contrast test
            int min = 255;
            int max = 0;

            for (int i = 0; i < 16; i++) {
                int px = x + rad3circle[i][0];
                int py = y + rad3circle[i][1];
                int value = pixels[py * width + px] & 0xFF;
                if (value > max) {
                    max = value;
                }
                if (value < min) {
                    min = value;
                }
            }
            if (max - min < 40) {
                return false;
            }

            //early rejection test
            int bright = 0, dark = 0;
            int value = pixels[y * width + (x + 3)] & 0xFF;
            if (value > high) {
                bright++;
            } else if (value < low) {
                dark++;
            }
            value = pixels[y * width + (x - 3)] & 0xFF;
            if (value > high) {
                bright++;
            } else if (value < low) {
                dark++;
            }
            value = pixels[(y - 3) * width + x] & 0xFF;
            if (value > high) {
                bright++;
            } else if (value < low) {
                dark++;
            }
            value = pixels[(y + 3) * width + x] & 0xFF;
            if (value > high) {
                bright++;
            } else if (value < low) {
                dark++;
            }

            if (bright < 3 && dark < 3) {
                return false;
            }

            //fast circle test
            int consecB = 0, consecD = 0;
            for (int i = 0; i < 16 + N - 1; i++) {

                int idx = i % 16;
                int px = x + rad3circle[idx][0];
                int py = y + rad3circle[idx][1];
                int pixelIdx = py * width + px;

                double pixelBrightness = pixels[pixelIdx] & 0xFF;

                if (pixelBrightness > centerBrightness + threshold) {
                    consecB++;
                    consecD = 0;
                } else if (pixelBrightness < centerBrightness - threshold) {
                    consecD++;
                    consecB = 0;
                } else {
                    consecB = 0;
                    consecD = 0;
                }

                if (consecD >= N || consecB >= N) {
                    return true;
                }
            }
            return false;
        }

        public byte[] createDescriptor(byte[] pixels, int width, int x, int y) {
            byte[] descriptor = new byte[49];
            int index = 0;
            int sum = 0;
            for (int dy = -3; dy <= 3; dy++) {
                for (int dx = -3; dx <= 3; dx++) {
                    int px = x + dx;
                    int py = y + dy;
                    int val = pixels[py * width + px] & 0xFF;
                    descriptor[index++] = (byte) val;
                    sum += val;
                }
            }
            double average = (double) sum / 49;
            for (int i = 0; i < descriptor.length; i++) {
                int val = descriptor[i] & 0xFF;
                descriptor[i] = (byte) (val - average);
            }
            return descriptor;
        }

        public ArrayList<Match> matchFeatures(ArrayList<Feature> previousFeatures, ArrayList<Feature> currentFeatures) {
            ArrayList<Match> result = new ArrayList<>();
            boolean[] previousUsed = new boolean[previousFeatures.size()];
            for (Feature current : currentFeatures) {
                int bestIndex = -1;
                Feature bestFeature = null;
                int bestDistance = Integer.MAX_VALUE;
                int secondBestDistance = Integer.MAX_VALUE;
                for (Feature previous : previousFeatures) {
                    int distance = descriptorDistance(current.descriptor, previous.descriptor);
                    if (distance < bestDistance) {
                        secondBestDistance = bestDistance;
                        bestDistance = distance;
                        bestFeature = previous;
                    } else if (distance < secondBestDistance) {
                        secondBestDistance = distance;
                    }
                }
                if (bestFeature == null) {
                    continue;
                }
                if (bestDistance > maxDescriptorDistance) {
                    continue;
                }
                if (secondBestDistance == Integer.MAX_VALUE) {
                    continue;
                }
                double ratio = (double) bestDistance / (double) secondBestDistance;
                if (ratio < matchRatio) {
                    result.add(new Match(bestFeature, current, bestDistance));
                }
            }
            return result;
        }

        public ArrayList<Match> ransacFilter(ArrayList<Match> input){
            ArrayList<Match> result = new ArrayList<>();
            if (input.isEmpty()){
                return result;
            }
            java.util.Random random = new java.util.Random();
            int best = 0;
            for (int i = 0; i<ransacInt;i++){
                Match sample = input.get(random.nextInt(input.size()));
                double dx = sample.current.point.x-sample.previous.point.x;
                double dy = sample.current.point.y-sample.previous.point.y;
                ArrayList<Match> inliers = new ArrayList<>();
                for (Match match:input){
                    double mDx = match.current.point.x-match.previous.point.x;
                    double mDy = match.current.point.y-match.previous.point.y;
                    double error = Math.hypot(mDx-dx,mDy-dy);
                    if(error<ransacThresh){
                        inliers.add(match);
                    }
                }
                if(inliers.size()>best){
                    best = inliers.size();
                    result = inliers;
                }
            }
            return result;
        }
        public int descriptorDistance(byte[] a, byte[] b) {
            int distance = 0;
            for (int i = 0; i < a.length; i++) {
                int valueA = a[i];
                int valueB = b[i];
                distance += Math.abs(valueA - valueB);
            }
            return distance;
        }
    }

    public static class Feature{
        public Point point;
        public byte[] descriptor;
        public Feature(Point point, byte[] descriptor){
            this.point = point;
            this.descriptor=descriptor;
        }
    }

    public static class Match{
        public Feature previous;
        public Feature current;
        public int distance;
        public Match(Feature previous, Feature current, int distance){
            this.previous = previous;
            this.current=current;
            this.distance=distance;

        }
    }

}