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

import java.util.concurrent.atomic.AtomicReference;
@Configurable
@TeleOp(name="VSLAM")
public class opmode extends OpMode {
    Servo wheel1,wheel2,pan,tilt;
    OpenCvCamera webcam;
    TelemetryManager.TelemetryWrapper panelsTelemetry;
    CameraStreamSource source;
    public static double wheel1Pos=0.5,wheel2Pos=0.5,panPos=0.5,tiltPos=0.25;
    public static double brightnessTolerance = 0.2;
    public static int consecutiveBrightnessTol = 9;

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

    }

    @Override
    public void stop(){
        PanelsCameraStream.INSTANCE.stopStream();
        if (webcam!=null){
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }



    public static class CameraPipeline extends OpenCvPipeline implements CameraStreamSource{
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(
                        Bitmap.createBitmap(
                                1,1,Bitmap.Config.RGB_565
                        )
                );

        @Override
        public Mat processFrame(Mat input){
            /**
             * Processing
             */

            Imgproc.cvtColor(input,input, Imgproc.COLOR_RGB2GRAY); //greyscale
            //read through all pixels and obtain brightness of each
            for (int y = 3; y<input.rows()-3;y++){
                for (int x = 3; x<input.cols()-3;x++){
                    if (isFeaturePoint(input,x,y,consecutiveBrightnessTol)){
                        input.put(y,x,255);
                    }
                }
            }
//            Imgproc.cvtColor(input,input,Imgproc.COLOR_GRAY2RGB);
            Bitmap bitmap = Bitmap.createBitmap(
                    input.width(),
                    input.height(),
                    Bitmap.Config.RGB_565
            );
            Utils.matToBitmap(input,bitmap);
            lastFrame.set(bitmap);
            return input;
        }
        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation){
            continuation.dispatch(
                    bitmapConsumer ->
                            bitmapConsumer.accept(lastFrame.get())
            );
        }

        public boolean isFeaturePoint(Mat brightness, int x, int y, int N){
            int[][] rad3circle = {
                    {0,-3}, //13
                    {1,-3}, //14
                    {2,-2}, //15
                    {3,-1}, //16
                    {3,0}, //1
                    {3,1}, //2
                    {2,2}, //3
                    {1,3}, //4
                    {0,3}, //5
                    {-1,3}, //6
                    {-2,2}, //7
                    {-3,1}, //8
                    {-3,0}, //9
                    {-3,-1}, //10
                    {-2,-2}, //11
                    {-1,-3} //12
            };

            double centerBrightness = brightness.get(y,x)[0];
            double threshold = centerBrightness*brightnessTolerance;
            int consecB = 0, consecD = 0;
            for (int i = 0; i<16+N-1;i++){
                int idx = i%16;
                int px = x+rad3circle[idx][0];
                int py = y+rad3circle[idx][1];

                double pixelBrightness = brightness.get(py,px)[0];
                if(pixelBrightness>centerBrightness+threshold){
                    consecB++;
                    consecD=0;
                }else if (pixelBrightness<centerBrightness-threshold){
                    consecD++;
                    consecB=0;
                }else{
                    consecB=0;
                    consecD=0;
                }

                if (consecD>=N || consecB>=N){
                    return true;
                }
            }
            return false;
        }
    }
}

