package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Bitmap;
import android.os.SystemClock;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
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
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.features2d.BFMatcher;
import org.opencv.features2d.ORB;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

@Configurable
@TeleOp(name="test vslam")
public class vslam extends LinearOpMode {
    public static int width = 640;
    public static int height = 480;

    public static int orbFeatures = 500;
    public static float orbScale = 1.2f;
    public static int orbLevels = 8;
    public static int orbThreshold = 31;
    public static int orbFirst = 0;
    public static int orbWta = 2;
    public static int orbSize = 31;
    public static int orbFastThreshold = 20;

    public static float ratioTest = 0.75f;
    public static float minGoodMatches = 15; // was 10, too many garbage poses at 10

    public static double ransacThresh = 1.5;
    public static double ransacConfidence = 0.999;
    public static int ransacMaxIters = 1000;

    public static boolean drawFeatures = true;
    public static boolean drawMatches = true;

    public static double translationScale = 1;
    public static double voxelSize = 0.05;
    public static int maxMapPoints = 20000;
    public static int maxNewPointsperFrame = 200;
    public static double maxTraignaultionDepth = 50;

    // c270 calib, ran checkerboard calib script for this
    public static double fx = 1201.301549650407;
    public static double fy = 1183.633514447323;
    public static double cx = 408.86431808728605;
    public static double cy = 161.5314095319902;
    public static double k1 = 0.46312970087929056;
    public static double k2 = -1.9861524150618615;
    public static double p1 = -0.0041021594732788;
    public static double p2 = 0.07755604987483086;
    public static double k3 = 4.769011405961361;

    private OpenCvCamera camera;
    private VSLAMPipeline pipeline;

    private TelemetryManager.TelemetryWrapper panelsTelemetry;
    CameraStreamSource source;

    private int frameCounter = 0; //for throttling map redraw

    @Override
    public void runOpMode(){
        panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        WebcamName name = hardwareMap.get(WebcamName.class,"c270");
        camera = OpenCvCameraFactory.getInstance().createWebcam(name,cameraMonitorViewId);
        pipeline = new VSLAMPipeline();
        camera.setPipeline(pipeline);
        source = pipeline;
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(width,height, OpenCvCameraRotation.UPRIGHT);
                PanelsCameraStream.INSTANCE.startStream(source,60);
                panelsTelemetry.addData("Camera:","Opened");
                panelsTelemetry.update();
            }
            @Override
            public void onError(int errorCode) {
                panelsTelemetry.addData("error", errorCode);
                panelsTelemetry.update();
            }
        });
        while (!isStarted() && !isStopRequested()){
            updateTelemetry(panelsTelemetry);
            sleep(50);
        }
        waitForStart();
        while (opModeIsActive()) {
            updateTelemetry(panelsTelemetry);
            sleep(20);
        }
        camera.stopStreaming();
        camera.closeCameraDevice();
    }

    private void updateTelemetry(TelemetryManager.TelemetryWrapper telemetry){
        if (pipeline== null){
            return;
        }
        telemetry.addData("fps",pipeline.getFps());
        telemetry.addData("orb features", pipeline.featureCount);
        telemetry.addData("raw matches", pipeline.rawMatches);
        telemetry.addData("ratio matches", pipeline.goodMatches);
        telemetry.addData("ransac inliers", pipeline.ransacInliers);
        telemetry.addData("pose inliers", pipeline.poseInliers);
        telemetry.addData("pose", pipeline.poseValid ? "valid" : "invalid");
        telemetry.addData("map points", pipeline.mapPointCount);
        if (pipeline.poseValid){
            telemetry.addData("tx",pipeline.tx);
            telemetry.addData("ty",pipeline.ty);
            telemetry.addData("tz",pipeline.tz);
            telemetry.addData("rx",pipeline.rx);
            telemetry.addData("ry",pipeline.ry);
            telemetry.addData("rz",pipeline.rz);
            Drawing.addPose(pipeline.wx,pipeline.wz); // dropping y, top down view
        }
        telemetry.addData("x",pipeline.wx);
        telemetry.addData("y",pipeline.wy);
        telemetry.addData("z",pipeline.wz);
        telemetry.addData("wrx",pipeline.wrx);
        telemetry.addData("wry",pipeline.wry);
        telemetry.addData("wrz",pipeline.wrz);

        double heading = getHeadingFromRotationVector(pipeline.wrx,pipeline.wry,pipeline.wrz);

        frameCounter++;
        if (frameCounter%5==0){
            Drawing.drawMapPts(pipeline.getMapSnapshot()); // copies the whole map so dont do every tick
        }
        Drawing.drawTrail();
        Drawing.drawCurr(pipeline.wx,pipeline.wz,heading);
        Drawing.update();

        telemetry.update();
    }

    private double getHeadingFromRotationVector(double rx, double ry, double rz){
        Mat rvec = new Mat(3,1,CvType.CV_64F);
        rvec.put(0,0,rx,ry,rz);
        Mat r = new Mat();
        Calib3d.Rodrigues(rvec,r);
        double fx = r.get(0,2)[0];
        double fz = r.get(2,2)[0];
        rvec.release();
        r.release();
        return Math.atan2(fx,fz);
    }
    public static class VSLAMPipeline extends OpenCvPipeline implements CameraStreamSource {
        private final ORB orb;
        private final BFMatcher matcher;
        private final Mat cameraMatrix;
        private final Mat distortion;
        private Mat prevGray = new Mat();
        private Mat prevDescriptors = new Mat();
        private MatOfKeyPoint prevKeypoints = new MatOfKeyPoint();
        private boolean hasPreviousFrame = false;
        private long lastTime = 0;
        private double fps = 0;
        public volatile int featureCount = 0;
        public volatile int rawMatches = 0;
        public volatile int goodMatches = 0;
        public volatile int ransacInliers = 0;
        public volatile int poseInliers = 0;
        public volatile boolean poseValid = false;
        public volatile double tx = 0;
        public volatile double ty = 0;
        public volatile double tz = 0;
        public volatile double rx = 0;
        public volatile double ry = 0;
        public volatile double rz = 0;

        private double[] worldR = new double[]{1,0,0,0,1,0,0,0,1};
        private double [] worldC = new double[]{0,0,0};
        public volatile double wx = 0,wy = 0,wz = 0;
        public volatile double wrx = 0, wry = 0, wrz = 0;
        private final ConcurrentHashMap<Long,float[]> worldMap = new ConcurrentHashMap<>();
        public volatile int mapPointCount = 0;
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(
                        Bitmap.createBitmap(
                                1, 1, Bitmap.Config.RGB_565
                        )
                );
        public VSLAMPipeline(){
            orb = ORB.create(orbFeatures,orbScale,orbLevels,orbThreshold,orbFirst,orbWta,ORB.HARRIS_SCORE,orbSize,orbFastThreshold);
            matcher = BFMatcher.create(Core.NORM_HAMMING,false);
            cameraMatrix = new Mat(3,3, CvType.CV_64F);
            cameraMatrix.put(0,0,fx,0,cx,0,fy,cy,0,0,1);
            distortion = new Mat(1,5,CvType.CV_64F);
            distortion.put(0,0,k1,k2,p1,p2,k3);
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

        @Override
        public Mat processFrame(Mat input){
            long now = SystemClock.elapsedRealtime();
            if (lastTime !=0){
                double dt = (double) (now - lastTime) /1000;
                if (dt>0){
                    double instantFps = 1/dt;
                    fps = 0.9*fps+0.1*instantFps;
                }
            }
            lastTime = now;
            if (input.empty()){
                return input;
            }

            Mat gray = new Mat();
            if (input.channels()==4){
                Imgproc.cvtColor(input,gray,Imgproc.COLOR_RGBA2GRAY);
            }else if (input.channels()==3){
                Imgproc.cvtColor(input,gray,Imgproc.COLOR_RGB2GRAY);
            }else {
                input.copyTo(gray);
            }

            Mat undistorted = new Mat();
            Calib3d.undistort(gray,undistorted,cameraMatrix,distortion);

            MatOfKeyPoint currentKP = new MatOfKeyPoint();
            Mat currentDescriptors = new Mat();
            orb.detectAndCompute(undistorted,new Mat(), currentKP,currentDescriptors);
            featureCount = currentKP.rows();

            if (!hasPreviousFrame) {
                undistorted.copyTo(prevGray);
                currentDescriptors.copyTo(prevDescriptors);
                currentKP.copyTo(prevKeypoints);
                hasPreviousFrame = true;
                gray.release();
                undistorted.release();
                Mat firstOutput = drawFeatures(input,currentKP);
                updateLastFrame(firstOutput);
                return firstOutput;
            }

            List<MatOfDMatch> knnMatches = new ArrayList<>();
            if (!prevDescriptors.empty() && !currentDescriptors.empty() && prevDescriptors.rows()>=2 && currentDescriptors.rows()>=2){
                matcher.knnMatch(prevDescriptors,currentDescriptors,knnMatches,2);
            }
            rawMatches = knnMatches.size();

            List<DMatch> goodMatchList = new ArrayList<>();
            for (MatOfDMatch match : knnMatches){
                DMatch [] matches = match.toArray();
                if (matches.length<2){
                    continue;
                }
                if (matches[0].distance<ratioTest*matches[1].distance){
                    goodMatchList.add(matches[0]);
                }
            }
            goodMatches = goodMatchList.size();

            KeyPoint[] prevKP = prevKeypoints.toArray();
            KeyPoint[] currentKeypoints = currentKP.toArray();
            List<Point> prevPoints = new ArrayList<>();
            List<Point> currentPoints = new ArrayList<>();
            for (DMatch match : goodMatchList){
                if (match.queryIdx>=prevKP.length || match.trainIdx>=currentKeypoints.length){
                    continue;
                }
                prevPoints.add(prevKP[match.queryIdx].pt);
                currentPoints.add(currentKeypoints[match.trainIdx].pt);
            }

            MatOfPoint2f prevMatPoints = new MatOfPoint2f();
            MatOfPoint2f currentMatPoints = new MatOfPoint2f();
            prevMatPoints.fromList(prevPoints);
            currentMatPoints.fromList(currentPoints);
            Mat fundamentalMask = new Mat();
            if (prevPoints.size()>=8){
                Calib3d.findFundamentalMat(prevMatPoints,currentMatPoints,Calib3d.FM_RANSAC,ransacThresh,ransacConfidence,fundamentalMask);
            }
            ransacInliers = fundamentalMask.empty() ? 0:Core.countNonZero(fundamentalMask);
            Mat essentialMask = new Mat();
            Mat essentialMatrix = new Mat();
            if (prevPoints.size()>=5){
                essentialMatrix = Calib3d.findEssentialMat(prevMatPoints,currentMatPoints,cameraMatrix,Calib3d.RANSAC,ransacConfidence,ransacThresh,ransacMaxIters,essentialMask);
            }
            Mat R = new Mat();
            Mat t = new Mat();
            poseValid = false;
            if (!essentialMatrix.empty() && prevPoints.size()>=5){
                try {
                    int poseInlierCount = Calib3d.recoverPose(essentialMatrix, prevMatPoints, currentMatPoints, cameraMatrix, R, t, essentialMask);
                    poseInliers = poseInlierCount;
                    if (poseInlierCount >= minGoodMatches) {
                        poseValid = true;
                        tx = t.get(0, 0)[0];
                        ty = t.get(1, 0)[0];
                        tz = t.get(2, 0)[0];
                        Mat rotationVector = new Mat();
                        Calib3d.Rodrigues(R, rotationVector);
                        rx = rotationVector.get(0, 0)[0];
                        ry = rotationVector.get(1, 0)[0];
                        rz = rotationVector.get(2, 0)[0];
                        rotationVector.release();
                    }
                }catch(Exception ignored){
                    // recoverPose throws sometimes if the essential mat is degenerate, just skip the frame
                    poseValid = false;
                }
            }
            if (poseValid){
                double [] owr = worldR.clone();
                double [] owc = worldC.clone();
                try {
                    Mat P1 = Mat.eye(3,4,CvType.CV_64F);
                    Mat P2 = Mat.zeros(3,4,CvType.CV_64F);
                    for (int r = 0; r < 3; r++) {
                        for (int c = 0; c < 3; c++) {
                            P2.put(r, c, R.get(r, c)[0]);
                        }
                        P2.put(r, 3, t.get(r, 0)[0]*translationScale);
                    }
                    Mat P1proj = new Mat();
                    Mat P2proj = new Mat();
                    Core.gemm(cameraMatrix,P1,1,new Mat(),0,P1proj);
                    Core.gemm(cameraMatrix,P2,1,new Mat(),0,P2proj);
                    Mat points4D = new Mat();
                    Calib3d.triangulatePoints(P1proj,P2proj,prevMatPoints,currentMatPoints,points4D);
                    int n = points4D.cols();
                    int added = 0;
                    for (int i = 0; i<n && added<maxNewPointsperFrame;i++){
                        double w = points4D.get(3,i)[0];
                        if (Math.abs(w)<1e-9) continue; // point at infinity basically, skip
                        double px = points4D.get(0,i)[0]/w;
                        double py = points4D.get(1,i)[0]/w;
                        double pz = points4D.get(2,i)[0]/w;
                        if (pz<=0 || pz>maxTraignaultionDepth) continue;
                        double [] worldPt = matVec3(owr,new double[] {px,py,pz});
                        worldPt[0]+=owc[0];
                        worldPt[1]+=owc[1];
                        worldPt[2]+=owc[2];
                        addPointToMap(worldPt[0],worldPt[1],worldPt[2]);
                        added++;
                    }
                    mapPointCount = worldMap.size();
                    points4D.release();
                    P1.release();
                    P2.release();
                    P1proj.release();
                    P2proj.release();
                }catch (Exception ignored){
                }
                // compose global pose - std vo formula, this took forever to get right
                double[] Rrel = mat3ToArray(R);
                double[] Trel = new double[]{t.get(0,0)[0]*translationScale,t.get(1,0)[0]*translationScale,t.get(2,0)[0]*translationScale};
                double []RrelT = matTranspose3(Rrel);
                double[] negTrel = {-Trel[0],-Trel[1],-Trel[2]};
                double [] cCurrPrev = matVec3(RrelT,negTrel);
                double [] cCurrWrld = matVec3(owr,cCurrPrev);
                cCurrWrld[0]+=owc[0];
                cCurrWrld[1]+=owc[1];
                cCurrWrld[2]+=owc[2];
                worldR = matMul3(owr,RrelT);
                worldC = cCurrWrld;
                wx = worldC[0];
                wy = worldC[1];
                wz = worldC[2];
                Mat wrm = new Mat(3,3,CvType.CV_64F);
                wrm.put(0,0,worldR);
                Mat wrvec = new Mat();
                Calib3d.Rodrigues(wrm,wrvec);
                wrx = wrvec.get(0,0)[0];
                wry = wrvec.get(1,0)[0];
                wrz = wrvec.get(2,0)[0];
                wrm.release();
                wrvec.release();
            }

            Mat output = input.clone();
            if (drawFeatures){
                KeyPoint[] kp = currentKP.toArray();
                for (KeyPoint keyPoint : kp){
                    Imgproc.circle(output,keyPoint.pt,2,new org.opencv.core.Scalar(0,255,0),-1);
                }
            }
            if (drawMatches){
                for (DMatch match:goodMatchList){
                    if (match.queryIdx>=prevKP.length||match.trainIdx>= currentKeypoints.length) continue;
                    Point p = currentKeypoints[match.trainIdx].pt;
                    Imgproc.circle(output,p,3,new Scalar(255,0,0),-1);
                }
            }
            Imgproc.putText(output,"orb: " + featureCount, new Point(10,25), Imgproc.FONT_HERSHEY_PLAIN,0.6,new Scalar(0,255,0),2);
            Imgproc.putText(output,"matches: " + goodMatches, new Point(10,50), Imgproc.FONT_HERSHEY_PLAIN,0.6,new Scalar(0,255,0),2);
            Imgproc.putText(output,"ransac inliers: " + ransacInliers, new Point(10,75), Imgproc.FONT_HERSHEY_PLAIN,0.6,new Scalar(0,255,0),2);
            Imgproc.putText(output,"pose inliers: " + poseInliers, new Point(10,100), Imgproc.FONT_HERSHEY_PLAIN,0.6,new Scalar(0,255,0),2);
            Imgproc.putText(output,"map pts: " + mapPointCount,new Point(10,125),Imgproc.FONT_HERSHEY_PLAIN,0.6,new Scalar(0,255,0),2);
            undistorted.copyTo(prevGray);
            currentDescriptors.copyTo(prevDescriptors);
            currentKP.copyTo(prevKeypoints);

            gray.release();
            undistorted.release();
            fundamentalMask.release();
            essentialMask.release();
            essentialMatrix.release();
            currentKP.release();
            currentDescriptors.release();
            R.release();
            t.release();
            prevMatPoints.release();
            currentMatPoints.release();
            updateLastFrame(output);
            return output;
        }
        private void addPointToMap(double x, double y, double z){
            if (worldMap.size()>=maxMapPoints) return;
            long key = voxelKey(x,y,z);
            worldMap.putIfAbsent(key,new float[]{(float) x, (float) y, (float) z});
        }
        private long voxelKey(double x, double y, double z){
            // pack 3 ints into a long, 21 bits each. good enough range for our field size
            long ix = Math.round(x/voxelSize);
            long iy = Math.round(y/voxelSize);
            long iz = Math.round(z/voxelSize);
            long mask = 0x1FFFFFL;
            return (ix&mask) | ((iy&mask)<<21)| ((iz&mask)<<42);
        }
        public List<float[]> getMapSnapshot(){
            return new ArrayList<>(worldMap.values());
        }
        public void resetMap(){
            worldMap.clear();
            mapPointCount = 0;
            worldR = new double[]{1,0,0,0,1,0,0,0,1};
            worldC = new double[]{0,0,0};
            wx = wy = wz = 0;
            wrx = wry = wrz = 0;
        }
        private Mat drawFeatures(Mat input, MatOfKeyPoint kp){
            Mat output = input.clone();
            if (!drawFeatures){
                return output;
            }
            KeyPoint[] points = kp.toArray();
            for (KeyPoint keypoint:points){
                Imgproc.circle(output,keypoint.pt,2,new Scalar(0,255,0),-1);
            }
            Imgproc.putText(output,"orb: "+points.length,new Point(10,25),Imgproc.FONT_HERSHEY_PLAIN,0.6,new Scalar(0,255,0),2);
            return output;
        }
        public double getFps(){
            return fps;
        }
        private void updateLastFrame(Mat mat) {
            // panels stream reads lastFrame directly, forgot this the first time and stared at a black screen for like an hour
            Bitmap bitmap = Bitmap.createBitmap(mat.width(), mat.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(mat, bitmap);
            lastFrame.set(bitmap);
        }

    }
    private static double [] mat3ToArray(Mat m){
        double [] r = new double[9];
        for (int i = 0; i<3;i++){
            for (int j = 0; j<3;j++){
                r[i*3+j] = m.get(i,j)[0];
            }
        }
        return r;
    }
    private static double [] matTranspose3(double[] a){
        double [] r = new double[9];
        for (int i = 0; i<3;i++){
            for (int j = 0; j<3;j++){
                r[i*3+j] = a[j*3+i];
            }
        }
        return r;
    }
    private static double[] matMul3(double [] a, double [] b){
        double[]r = new double[9];
        for (int i = 0; i<3;i++){
            for (int j = 0; j<3;j++){
                double sum = 0;
                for (int k = 0; k<3;k++){
                    sum+=a[i*3+k]*b[k*3+j];
                }
                r[i*3+j] = sum;
            }
        }
        return r;
    }
    private static double[] matVec3(double[] a, double[] v){
        double[] r = new double[3];
        for (int i = 0; i<3;i++){
            double sum = 0;
            for (int j = 0; j<3;j++){
                sum+=a[i*3+j]*v[j];
            }
            r[i] = sum;
        }
        return r;
    }

    // draws trail + map cloud on panels field view, top down like the orb slam3 viewer
    public static class Drawing{
        public static double plotScale = 20; // vslam scale is arbitrary, just tuned this til it looked right
        public static int maxPoints = 2000;
        public static int maxRendered = 1500;
        private static final FieldManager field = PanelsField.INSTANCE.getField();
        private static final Style trail = new Style("","#4CAF50",0);
        private static final Style pose = new Style("","#3F51B5",0);
        private static final Style point = new Style("","#9C27B0",0);
        private static final ArrayDeque<double[]> trailPts = new ArrayDeque<>();

        public static void addPose(double x, double y){
            trailPts.addLast(new double[]{x*plotScale,y*plotScale});
            while (trailPts.size()>maxPoints) trailPts.removeFirst();
        }
        public static void drawTrail(){
            field.setStyle(trail);
            double[] prev = null;
            for (double[] p : trailPts){
                if (prev!=null){
                    field.moveCursor(prev[0],prev[1]);
                    field.line(p[0],p[1]);
                }
                prev = p;
            }
        }
        public static void drawCurr(double wx, double wz, double h){
            field.setStyle(pose);
            double x = wx*plotScale;
            double z = wz*plotScale;
            field.moveCursor(x,z);
            field.circle(3);
            double hx = x+Math.sin(h)*8;
            double hz = z+Math.cos(h)*8;
            field.moveCursor(x,z);
            field.line(hx,hz);
        }
        public static void drawMapPts (List<float[]> pts){
            field.setStyle(point);
            int n= Math.max(1,pts.size()/maxRendered); // subsample or it chugs once the map fills up
            for (int i = 0; i<pts.size();i+=n) {
                float[] p = pts.get(i);
                field.moveCursor(p[0] * plotScale, p[2] * plotScale);
                field.circle(0.5);
            }
        }
        public static void update(){
            field.update();
        }
    }

}