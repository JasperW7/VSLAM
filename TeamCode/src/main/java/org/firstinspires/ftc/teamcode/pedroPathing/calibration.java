package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Bitmap;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

@Configurable
@TeleOp
public class calibration extends LinearOpMode {
    public static int height = 480;
    public static int width = 640;

    public static int checkerboardHight = 6;
    public static int checkerboardWidth = 9;
    public static double squareSize = 25;
    public static int framesRequired = 20;
    private OpenCvCamera webcam;
    private calibrationPipeline pipeline;
    private TelemetryManager.TelemetryWrapper panelsTelemetry;
    public static double k1=0, k2=0, p1=0, p2=0, k3=0;
    public static double fx=0,fy=0,cx=0,cy=0;

    @Override
    public void runOpMode(){
        panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        WebcamName name = hardwareMap.get(WebcamName.class,"c270");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(name,cameraMonitorViewId);
        pipeline = new calibrationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(width,height, OpenCvCameraRotation.UPRIGHT);
                PanelsCameraStream.INSTANCE.startStream(pipeline,30);
            }


            @Override
            public void onError(int errorCode) {
                panelsTelemetry.addData("error", errorCode);
                panelsTelemetry.update();
            }
        });
        waitForStart();

        while (opModeIsActive()){
            panelsTelemetry.addData("Checkerboard: ", pipeline.checkerboardFound ? "found": "not found");
            panelsTelemetry.addData("captured frameds: ",pipeline.getCapturedFrameCount() + "/" + framesRequired);
            panelsTelemetry.addData("status", pipeline.getStatus());
            panelsTelemetry.addData("fx",fx);
            panelsTelemetry.addData("fy",fy);
            panelsTelemetry.addData("cx",cx);
            panelsTelemetry.addData("cy",cy);
            panelsTelemetry.addData("k1",k1);
            panelsTelemetry.addData("k2",k2);
            panelsTelemetry.addData("p1",p1);
            panelsTelemetry.addData("p2",p2);
            panelsTelemetry.addData("k3",k3);

            panelsTelemetry.addData("fps: ", webcam.getFps());
            panelsTelemetry.update();

            if (!pipeline.calibrationFinished && pipeline.getCapturedFrameCount()>=framesRequired){
                pipeline.calibrate();
            }
            sleep(100);

        }
        PanelsCameraStream.INSTANCE.stopStream();
        if(webcam!=null){
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    public static class calibrationPipeline extends OpenCvPipeline implements CameraStreamSource {
        public boolean checkerboardFound = false;
        public boolean calibrationFinished = false;
        private MatOfPoint2f currentCorners = new MatOfPoint2f();
        private final ArrayList<Mat> imagePoints = new ArrayList<>();
        private Point lastCapture = null;
        public static int minimumMovement = 40;
        public static long minimumInterval = 1000;
        private long lastTime = 0;
        private String status = "waiting";
        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
        @Override
        public Mat processFrame(Mat input){
            Mat gray = new Mat();
            Imgproc.cvtColor(input,gray,Imgproc.COLOR_RGB2GRAY);
            Size size = new Size(checkerboardWidth,checkerboardHight);
            MatOfPoint2f corners = new MatOfPoint2f();
            checkerboardFound = Calib3d.findChessboardCorners(gray,size,corners,Calib3d.CALIB_CB_ADAPTIVE_THRESH|Calib3d.CALIB_CB_NORMALIZE_IMAGE | Calib3d.CALIB_CB_FAST_CHECK);
            if (checkerboardFound){
                Imgproc.cornerSubPix(gray,corners,new Size(11,11),new Size(-1,-1),new TermCriteria(TermCriteria.EPS|TermCriteria.MAX_ITER,30,0.001));
                currentCorners=corners;
                Calib3d.drawChessboardCorners(input,size,corners,true);
                Point[] points = corners.toArray();
                double centX = 0;
                double centY = 0;
                for (Point point : points){
                    centX+=point.x;
                    centY+=point.y;

                }
                centX/=points.length;
                centY/=points.length;
                Point currCent = new Point(centX,centY);
                long now = System.currentTimeMillis();
                boolean movedEnough = lastCapture==null || Math.hypot(centX-lastCapture.x,centY-lastCapture.y)>minimumMovement;
                boolean waitedEnough = now - lastTime>=minimumInterval;
                if (movedEnough && waitedEnough){
                    captureFrame(currentCorners);
                    lastCapture = currCent;
                    lastTime = now;
                }
            }
            gray.release();
            Bitmap bitmap = Bitmap.createBitmap(input.width(),input.height(),Bitmap.Config.RGB_565);
            Utils.matToBitmap(input,bitmap);
            lastFrame.set(bitmap);
            return input;
        }
        @Override
        public void getFrameBitmap(
                Continuation<? extends Consumer<Bitmap>> continuation
        ) {
            continuation.dispatch(
                    bitmapConsumer ->
                            bitmapConsumer.accept(lastFrame.get())
            );
        }


        private void captureFrame(MatOfPoint2f corners){
            if (imagePoints.size()>=framesRequired){
                return;
            }
            MatOfPoint2f saved = new MatOfPoint2f();
            saved.fromArray(corners.toArray());
            imagePoints.add(saved);
            status = "Captured " + imagePoints.size() + "/" + framesRequired;

        }
        public int getCapturedFrameCount(){
            return imagePoints.size();
        }
        public String getStatus(){
            return status;
        }
        public void calibrate(){
            if (imagePoints.size()<5){
                status = "need at least 5 frames";
                return;
            }
            status = "calibrating";
            ArrayList<Mat> objectPoints = new ArrayList<>();
            for (int frame = 0; frame<imagePoints.size();frame++){
                MatOfPoint3f objectCorners = new MatOfPoint3f();
                Point3[] points = new Point3[checkerboardWidth*checkerboardHight];
                int idx = 0;
                for (int y = 0; y<checkerboardHight;y++){
                    for (int x = 0; x<checkerboardWidth;x++){
                        points[idx++] = new Point3(x*squareSize,y*squareSize,0);
                    }
                }
                objectCorners.fromArray(points);
                objectPoints.add(objectCorners);
            }
            Mat cameraMatrix = Mat.eye(3,3, CvType.CV_64F);
            Mat distortion = Mat.zeros(8,1,CvType.CV_64F);
            ArrayList<Mat> rvecs = new ArrayList<>();
            ArrayList<Mat> tvecs = new ArrayList<>();
            double rms = Calib3d.calibrateCamera(objectPoints,imagePoints,new Size(width,height),cameraMatrix,distortion,rvecs,tvecs);
             fx = cameraMatrix.get(0,0)[0];
             fy = cameraMatrix.get(1,1)[0];
             cx = cameraMatrix.get(0,2)[0];
             cy = cameraMatrix.get(1,2)[0];
            k1 = distortion.get(0,0)[0];
            k2 = distortion.get(1,0)[0];
            p1 = distortion.get(2,0)[0];
            p2 = distortion.get(3,0)[0];
            k3 = distortion.get(4,0)[0];
            System.out.println("rms error: " + rms);
            System.out.println("matrix: ");
            System.out.println("["+fx+",0,"+cx+"]");
            System.out.println("["+fy+",0,"+cy+"]");
            System.out.println("[0,0,1]");
            System.out.println("distortion:");
            System.out.println("k1:"+k1);
            System.out.println("k2:"+k2);
            System.out.println("p1:"+p1);
            System.out.println("p2:"+p2);
            System.out.println("k3:"+k3);
            status = "finished, rms="+rms;
            calibrationFinished=true;

        }

    }
}
