package com.mbsbahru.na568Teamproject_MohammedAlanUsmanBahru;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.calib3d.Calib3d;

import android.os.Environment;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.content.Intent;
import androidx.appcompat.app.AppCompatActivity;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.UUID;
import com.google.android.material.floatingactionbutton.FloatingActionButton;

public class MainActivity extends AppCompatActivity implements CvCameraViewListener2, SensorEventListener {
    private int toggleButtonSavecsvVsIntImuVsReference = 1;
    private boolean toggleLowpassFilter = false;
    private boolean toggleKalmanFilter = true;
    private SensorManager sensorManager;
    private Sensor magnetometer;
    private Sensor accelerometer;
    private Sensor gyroscope;
    private KalmanFilter camKF;
    private boolean isUpdating = false;

    private float[] ImuAccelData = new float[3];
    private float[] ImuGyroData = new float[3];
    private float[] ImuMagnetData = new float[3];

    private float[] ImuPositionsData = new float[3];
    private float[] ImuVelocitiesData = new float[3];
    private float[] ImuOrientationData = new float[3];
    private float[] ImuRotationMatrix = new float[9];
    private float[] ImuInclinationMatrix = new float[9];
    private float lastUpdateTime = 0.0f;
    private float alpha = 0.7f;
    private float driftRate = 0;

    private float[] velUnfiltered = new float[3];
    private float[] posUncorrected = new float[3];
    private float[] lastRawPosition = new float[3];
    private static final String TAG = "MainActivity";
    private static final int    VIEW_MODE_HSV_SEGMENTATION   = 1;
    private static final int    VIEW_MODE_REF_INPUT   = 2;
    private static final int    VIEW_MODE_POSE_ACT   = 0;
     private int mViewMode;
    NoiseEstimator noiseEstimator;
    private int limT = 50;
    private int limL = 50;
    private double area1;
    private double[] tvecArr = new double[3];
    private Moments moment1;
    private int centroid_x1;
    private int centroid_y1;
    private Point centroidBase;
     private Mat mRgba;
     private Mat mShow;
     private Mat mBinerBase;
     private Mat mBinerHandle;
     private Mat mGray;
     private Mat mEdge;
     private Mat mHSV;
    private MenuItem     mItemHsvSegmentation;
    private MenuItem     mItemHsvRefInput;
    private MenuItem     mItemPoseAction;
    private CameraBridgeViewBase mOpenCvCameraView;



    private MatOfPoint3f objectPoints = new MatOfPoint3f();
    private MatOfPoint2f imagePoints = new MatOfPoint2f();

    double focalLength = 25; // focal length of Google Pixel 7 in mm from internet (not being used anymore as we have calibrated intrinsic)
    double pxSize = 0.1; // assumed pixel width (not being used anymore as we have calibrated intrinsic)
    MatOfDouble distCoeffs = new MatOfDouble();
    Mat cameraMatrix = Mat.eye(3, 3, CvType.CV_64F);
    double f_x = 943.1746; // camera intrinsic after calibration: focal length fx
    double f_y = 951.7460; // camera intrinsic after calibration: focal length fy
    double c_x = 650.4762; // camera intrinsic after calibration: principal point cx
    double c_y = 364.3175; // camera intrinsic after calibration: principal point cy
    double skewCam = 2.5376; // camera intrinsic after calibration: radial skewness s
    double radDist_1 = 0.2585; // camera intrinsic after calibration: radial distortion k_1
    double radDist_2 = -1.6159; // camera intrinsic after calibration: radial distortion k_2
    double radDist_3 = 4.2888; // camera intrinsic after calibration: radial distortion k_3
    double tanDist_1 = -0.0051; // camera intrinsic after calibration: radial distortion p_1
    double tanDist_2 = -0.0008022; // camera intrinsic after calibration: radial distortion p_2
    double euc_dist_R = 0;
    private double BASE_REAL_LENGTH = ReferenceInput.objectLength; // real-world object length in cm // obj2 27.8
    private double BASE_REAL_WIDTH = ReferenceInput.objectWidth; // real-world object width in cm // obj2 20.75
    double REFERENCE_R = ReferenceInput.referenceR;
    double REFERENCE_PHI = ReferenceInput.referencePhi;
    double REFERENCE_THETA = ReferenceInput.referenceTheta;
    double REFERENCE_PSI = ReferenceInput.referencePsi;
    double TOLERANCE_R = 5;
    double TOLERANCE_PHI = 5;
    double TOLERANCE_THETA = 5;
    double TOLERANCE_PSI = 5;
    private double[] CamLinPositionsData = new double[3];
    private double[] CamAngPositionsData = new double[3];
    private double[] filteredState = new double[6];
    double[][] u_control = new double[3][1];
    private Mat rvec = new Mat();
    private Mat tvec = new Mat();
    Mat tvecNeg = new Mat(tvec.rows(), tvec.cols(), tvec.type());
    Mat rMatTranspose = new Mat(3, 3, CvType.CV_64F);
    Mat CamLinPositions = new Mat();
    private Mat CamRotationMatrix = new Mat();
    double[] rMat = new double[9];
    private double[][] R_in_w = new double[3][3];
    private double[] t_in_w = new double[3];
    private boolean isRecording = false;
    private double x_last = 0.0;
    private double y_last = 0.0;
    private double z_last = 0.0;
    private double phi_last = 0.0;
    private double theta_last = 0.0;
    private double psi_last = 0.0;

    private static final UUID MAGIC_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {

        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };
     private Scalar BLUE_COLOR;
     private Scalar RED_COLOR;
     private Scalar GREEN_COLOR;
     private Scalar YELLOW_COLOR;
    private Scalar ORANGE_COLOR;
    private Scalar BLACK_COLOR;

     /**
      * Called when the activity is first created.
      */
     @Override
     public void onCreate(Bundle savedInstanceState) {
         super.onCreate(savedInstanceState);
         getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
         setContentView(R.layout.camera_view);

         double[] initial_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
         double[] process_noise_std = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
         double[] measurement_noise_std = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
         camKF = new KalmanFilter(initial_state, process_noise_std, measurement_noise_std);
         noiseEstimator = new NoiseEstimator(camKF);

         sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
         magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
         accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
         gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

         mOpenCvCameraView = findViewById(R.id.activity_surface_view);
         mOpenCvCameraView.setVisibility(CameraBridgeViewBase.VISIBLE);
         mOpenCvCameraView.setCvCameraViewListener(this);
     }

     @Override
     public boolean onCreateOptionsMenu(Menu menu) {
         mItemHsvSegmentation = menu.add("HSV Object Segmentation");
         mItemHsvRefInput = menu.add("Reference Input");
         mItemPoseAction = menu.add("Pose Perfect Action");
         return true;
     }

     @Override
     public void onPause() {
         super.onPause();
         if (mOpenCvCameraView != null)
             mOpenCvCameraView.disableView();
         sensorManager.unregisterListener(this);
     }

    @Override
    public void onResume() {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_4_0, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        sensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_NORMAL);
        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
        sensorManager.registerListener(this, gyroscope, SensorManager.SENSOR_DELAY_NORMAL);
    }

     public void onDestroy() {
         if (mOpenCvCameraView != null)
             mOpenCvCameraView.disableView();

         super.onDestroy();
     }

     public void onCameraViewStarted(int width, int height) {
         mRgba = new Mat(height, width, CvType.CV_8UC4);
         mShow = new Mat();
         mHSV = new Mat(height, width, CvType.CV_8UC3);
         mBinerBase = new Mat(height, width, CvType.CV_8UC1);
         mBinerHandle = new Mat(height, width, CvType.CV_8UC1);
         mGray = new Mat(height, width, CvType.CV_8UC1);
         mEdge = new Mat(height, width, CvType.CV_8UC1);
         BLUE_COLOR = new Scalar(0, 0, 255, 255);
         RED_COLOR = new Scalar(255, 0, 0, 255);
         GREEN_COLOR = new Scalar(50, 100, 50, 255);
         YELLOW_COLOR = new Scalar(255, 255, 0, 0);
         ORANGE_COLOR = new Scalar(255, 165, 0, 0);
         BLACK_COLOR = new Scalar(0, 0, 0, 0);
     }

     public void onCameraViewStopped() {
         mGray.release();
         mEdge.release();
         mRgba.release();
         mHSV.release();
         mShow.release();
         mBinerBase.release();
         mBinerHandle.release();
     }

     public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
         FloatingActionButton bStart = findViewById(R.id.fabstart);
         FloatingActionButton bReset = findViewById(R.id.fabreset);

         if (toggleButtonSavecsvVsIntImuVsReference == 0) {
             bStart.setOnClickListener(new View.OnClickListener() {
                 @Override
                 public void onClick(View view) {
                     isRecording = true; // Start recording the data into csv file
                 }
             });
         }
         else if (toggleButtonSavecsvVsIntImuVsReference == 1){
             bStart.setOnClickListener(new View.OnClickListener() {
                 public void onClick(View view){
                     REFERENCE_R = euc_dist_R;
                     REFERENCE_PHI = filteredState[3];
                     REFERENCE_THETA = filteredState[4];
                     REFERENCE_PSI = filteredState[5];
                 }
             });
         }
         else if (toggleButtonSavecsvVsIntImuVsReference == 2){
             bStart.setOnClickListener(new View.OnClickListener() {
                 public void onClick(View view) {
                     lastUpdateTime = System.nanoTime();
                     if (isUpdating) {
                         isUpdating = false;
                         lastUpdateTime = System.nanoTime();
                     } else {
                         isUpdating = true;
                         lastUpdateTime = System.currentTimeMillis();
                     }
                 }
             });
         }

         if (toggleButtonSavecsvVsIntImuVsReference == 0) {
             bReset.setOnClickListener(new View.OnClickListener() {
                 @Override
                 public void onClick(View view) {
                     isRecording = false;
                 }
             });
         }
         else if (toggleButtonSavecsvVsIntImuVsReference == 1){
             bReset.setOnClickListener(new View.OnClickListener() {
                 public void onClick(View view){
                 REFERENCE_R =ReferenceInput.referenceR;
                 REFERENCE_PHI =ReferenceInput.referencePhi;
                 REFERENCE_THETA =ReferenceInput.referenceTheta;
                 REFERENCE_PSI =ReferenceInput.referencePsi;
             }
             });
         }
         else if (toggleButtonSavecsvVsIntImuVsReference == 2){
             bReset.setOnClickListener(new View.OnClickListener() {
                 @Override
                 public void onClick(View view) {
                     ImuPositionsData = new float[]{0, 0, 0};
                     ImuVelocitiesData = new float[]{0, 0, 0};
                     ImuOrientationData = new float[]{0, 0, 0};
                     ImuAccelData = new float[]{0, 0, 0};

                     lastUpdateTime = System.currentTimeMillis();
                     lastUpdateTime = System.nanoTime();
                 }
             });
         }

         final int viewMode = mViewMode;
         mRgba = inputFrame.rgba();
         Imgproc.cvtColor(mRgba,mRgba,Imgproc.COLOR_BGRA2BGR);
         Imgproc.medianBlur(mRgba, mRgba, 2* SeekBarHsvSeg.MedianBlur1+1);
         Imgproc.GaussianBlur(mRgba, mRgba, new Size(2* SeekBarHsvSeg.GaussianBlur1+1,2* SeekBarHsvSeg.GaussianBlur1+1),0,0);

         if (viewMode == VIEW_MODE_HSV_SEGMENTATION) {
             Intent intent = new Intent(MainActivity.this, SeekBarHsvSeg.class);
             startActivity(intent);
             finish();
         }
         else if (viewMode == VIEW_MODE_REF_INPUT) {
            Intent intent = new Intent(MainActivity.this, ReferenceInput.class);
            startActivity(intent);
            finish();
         }
         else if (viewMode == VIEW_MODE_POSE_ACT) {
             mShow = MODE_AWAL_MISI0();
         }
         return mShow;
     }
     public boolean onOptionsItemSelected(MenuItem item) {
     Log.i(TAG, "called onOptionsItemSelected; selected item: " + item);
         if (item == mItemHsvSegmentation) {
             mViewMode = VIEW_MODE_HSV_SEGMENTATION;
         } else if (item == mItemHsvRefInput) {
             mViewMode = VIEW_MODE_REF_INPUT;
         } else if (item == mItemPoseAction) {
             mViewMode = VIEW_MODE_POSE_ACT;
         }
         return true;
     }

     public Mat MODE_AWAL_MISI0() {

////////// Object Feature Extraction and Camera Calibration by Mbsbahru - Muhammad Bahru Sholahuddin ////////////
         Scalar hsv_min1 = new Scalar(SeekBarHsvSeg.Hmin1, SeekBarHsvSeg.Smin1, SeekBarHsvSeg.Vmin1, 0);
         Scalar hsv_max1 = new Scalar(SeekBarHsvSeg.Hmax1, SeekBarHsvSeg.Smax1, SeekBarHsvSeg.Vmax1, 0);

         Imgproc.cvtColor(mRgba, mHSV, Imgproc.COLOR_RGB2HSV_FULL, 4);

         Core.inRange(mHSV, hsv_min1, hsv_max1, mBinerBase);

         Imgproc.erode(mBinerBase, mBinerBase, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * SeekBarHsvSeg.Erode1 + 1, 2 * SeekBarHsvSeg.Erode1 + 1), new Point(SeekBarHsvSeg.Erode1, SeekBarHsvSeg.Erode1)));
         Imgproc.dilate(mBinerBase, mBinerBase, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * SeekBarHsvSeg.Dilate1 + 1, 2 * SeekBarHsvSeg.Dilate1 + 1), new Point(SeekBarHsvSeg.Dilate1, SeekBarHsvSeg.Dilate1)));
         Imgproc.dilate(mBinerBase, mBinerBase, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * SeekBarHsvSeg.Dilate1 + 1, 2 * SeekBarHsvSeg.Dilate1 + 1), new Point(SeekBarHsvSeg.Dilate1, SeekBarHsvSeg.Dilate1)));
         Imgproc.erode(mBinerBase, mBinerBase, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * SeekBarHsvSeg.Erode1 + 1, 2 * SeekBarHsvSeg.Erode1 + 1), new Point(SeekBarHsvSeg.Erode1, SeekBarHsvSeg.Erode1)));


         List<MatOfPoint> contours1 = new ArrayList<MatOfPoint>();
         Imgproc.findContours(mBinerBase, contours1, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE, new Point(0, 0));


         if (contours1.size() > 0 ){
         Imgproc.circle (mRgba, new Point(centroid_x1, centroid_y1), 4, RED_COLOR, 3, 8 ,0 );
         Imgproc.circle (mRgba, new Point(mRgba.cols()/2, mRgba.rows()/2), 4, GREEN_COLOR, 3, 8 ,0 );

             int counter1 = getObjek(contours1);
             Imgproc.drawContours(mRgba, contours1, counter1, GREEN_COLOR);

             MatOfInt tmp = new MatOfInt();
             Imgproc.convexHull(contours1.get(counter1), tmp);
             MatOfPoint hullContour = hull2points(tmp, contours1.get(counter1));
             MatOfPoint2f hullContour2f = new MatOfPoint2f(hullContour.toArray());

             double epsilon = Imgproc.arcLength(hullContour2f, true) * 0.03;
             MatOfPoint2f approxCurve = new MatOfPoint2f();
             Imgproc.approxPolyDP(hullContour2f, approxCurve, epsilon, true);

             List<MatOfPoint> hullList = new ArrayList<>();
             hullList.add(hullContour);
             Imgproc.drawContours(mRgba, hullList, -1, BLUE_COLOR, 4);


             Point[] points = approxCurve.toArray();
             Point corner_topLeft = new Point(0, 0);
             Point corner_topRight = new Point(0, 0);
             Point corner_botLeft = new Point(0, 0);
             Point corner_botRight = new Point(0, 0);

             Arrays.sort(points, Comparator.comparingDouble(point -> point.x));
            if (points.length == 4) {
                Point[] leftPoints = {points[0], points[1]};
                Point[] rightPoints = {points[2], points[3]};

                Arrays.sort(leftPoints, Comparator.comparingDouble(point -> point.y));
                Arrays.sort(rightPoints, Comparator.comparingDouble(point -> point.y));

                corner_topLeft = leftPoints[0];
                corner_botLeft = leftPoints[1];
                corner_topRight = rightPoints[0];
                corner_botRight = rightPoints[1];
            }

             if (corner_topLeft.x != 0 || corner_topRight.x != 0.0 || corner_botLeft.x != 0.0 || corner_botRight.x != 0.0) {
                 Point topSideCenter = lineCenter(corner_topLeft, corner_topRight);
                 Point botSideCenter = lineCenter(corner_botLeft, corner_botRight);
                 Point leftSideCenter = lineCenter(corner_topLeft, corner_botLeft);
                 Point rightSideCenter = lineCenter(corner_topRight, corner_botRight);

                 Point rectCenter = calcIntersectionPoints(leftSideCenter, rightSideCenter, topSideCenter, botSideCenter);
                 centroidBase = rectCenter;

                 Point horCenter = lineCenter(leftSideCenter, rightSideCenter);
                 Point verCenter = lineCenter(topSideCenter, botSideCenter);

                 Imgproc.circle(mRgba, horCenter, 4, GREEN_COLOR, 3, 8, 0);
                 Imgproc.circle(mRgba, verCenter, 4, GREEN_COLOR, 3, 8, 0);

                 Imgproc.line(mRgba, topSideCenter, botSideCenter, new Scalar(1, 1, 1), 1, Imgproc.LINE_AA);
                 Imgproc.line(mRgba, leftSideCenter, rightSideCenter, new Scalar(1, 1, 1), 1, Imgproc.LINE_AA);

                 double top_line = drawCalcLine(corner_topLeft, corner_topRight);
                 double bot_line = drawCalcLine(corner_botLeft, corner_botRight);
                 double left_line = drawCalcLine(corner_topLeft, corner_botLeft);
                 double right_line = drawCalcLine(corner_botRight, corner_topRight);
                 double center_line;
                 double yaw_angle2d;

                 if (Math.max(top_line, bot_line) > Math.max(left_line, right_line)) { // The object is Horizontal/Landscape
                     Imgproc.line(mRgba, rectCenter, botSideCenter, GREEN_COLOR, 7, Imgproc.LINE_AA);
                     Imgproc.line(mRgba, rectCenter, rightSideCenter, BLUE_COLOR, 7, Imgproc.LINE_AA);

                     Imgproc.putText(mRgba, "Y", new Point(botSideCenter.x, botSideCenter.y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, RED_COLOR, 3);
                     Imgproc.putText(mRgba, "X", new Point(rightSideCenter.x, rightSideCenter.y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, BLUE_COLOR, 3);

                     center_line = drawCalcLine(topSideCenter, rectCenter);
                     yaw_angle2d = Math.toDegrees(Math.asin((topSideCenter.x - rectCenter.x) / center_line));
                 } else {
                     Imgproc.line(mRgba, rectCenter, rightSideCenter, GREEN_COLOR, 7, Imgproc.LINE_AA);
                     Imgproc.line(mRgba, rectCenter, botSideCenter, BLUE_COLOR, 7, Imgproc.LINE_AA);

                     Imgproc.putText(mRgba, "X", new Point(topSideCenter.x, topSideCenter.y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, BLUE_COLOR, 3);
                     Imgproc.putText(mRgba, "Y", new Point(rightSideCenter.x, rightSideCenter.y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, GREEN_COLOR, 3);

                     center_line = drawCalcLine(rightSideCenter, rectCenter);
                     yaw_angle2d = Math.toDegrees(Math.asin((rightSideCenter.x - rectCenter.x) / center_line));
                 }

                 area1 = Imgproc.contourArea(hullContour);
                 double yawAngleDegrees = Math.abs(yaw_angle2d);
                 int maxZAxisLength = 100;
                 int zAxisLength = (int) (maxZAxisLength * (1 - (yawAngleDegrees / 90)));
                 zAxisLength = Math.max(zAxisLength, 0);
                 Point zAxisEnd = new Point(rectCenter.x, rectCenter.y - zAxisLength);

                 Imgproc.line(mRgba, rectCenter, zAxisEnd, RED_COLOR, 4);
                 Imgproc.putText(mRgba, "Z", new Point(zAxisEnd.x, zAxisEnd.y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, RED_COLOR, 2);

                 Imgproc.putText(mRgba, "c_TL", corner_topLeft, Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(127, 255, 0), 2);
                 Imgproc.circle(mRgba, corner_topLeft, 4, GREEN_COLOR, -1);
                 Imgproc.putText(mRgba, "c_TR", corner_topRight, Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(127, 255, 0), 2);
                 Imgproc.circle(mRgba, corner_topRight, 4, GREEN_COLOR, -1);
                 Imgproc.putText(mRgba, "c_BL", corner_botLeft, Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(127, 255, 0), 2);
                 Imgproc.circle(mRgba, corner_botLeft, 4, GREEN_COLOR, -1);
                 Imgproc.putText(mRgba, "c_BR", corner_botRight, Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(127, 255, 0), 2);
                 Imgproc.circle(mRgba, corner_botRight, 4, GREEN_COLOR, -1);

                 Imgproc.putText(mRgba, String.valueOf((int) yaw_angle2d) + " deg", centroidBase, Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(127, 255, 0), 2);
             }
             else {
                 centroidBase = new Point(centroid_x1, centroid_y1);
             }
             Imgproc.circle(mRgba, centroidBase, 4, BLUE_COLOR, 3, 8, 0);
//             Imgproc.putText(mRgba, "Centroid: " + centroidBase, new Point(10, mRgba.rows() * 0.2 + 170), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 255), 1);
             Imgproc.line(mRgba, new Point(mRgba.cols() / 2, mRgba.rows() / 2), centroidBase, BLUE_COLOR);

             moment1 = Imgproc.moments(hullContour);
             if (moment1.m10 != 0 & moment1.m01 != 0 & moment1.m00 != 0) {
                 centroid_x1 = ((int) moment1.m10 / (int) moment1.m00);
                 centroid_y1 = ((int) moment1.m01 / (int) moment1.m00);
             }

             objectPoints = new MatOfPoint3f(
                     new Point3(-BASE_REAL_LENGTH/2, BASE_REAL_WIDTH/2, 0),
                     new Point3(BASE_REAL_LENGTH/2, BASE_REAL_WIDTH/2, 0),
                     new Point3(BASE_REAL_LENGTH/2, -BASE_REAL_WIDTH/2, 0),
                     new Point3(-BASE_REAL_LENGTH/2, -BASE_REAL_WIDTH/2, 0)
             );

             imagePoints = new MatOfPoint2f(
                     new Point(corner_topLeft.x, corner_topLeft.y),
                     new Point(corner_topRight.x, corner_topRight.y),
                     new Point(corner_botRight.x, corner_botRight.y),
                     new Point(corner_botLeft.x, corner_botLeft.y));

             cameraMatrix.put(0, 0, f_x);
             cameraMatrix.put(0, 1, skewCam);
             cameraMatrix.put(1, 1, f_y);
             cameraMatrix.put(0, 2, c_x);
             cameraMatrix.put(1, 2, c_y);


             distCoeffs.fromArray(radDist_1, radDist_2, tanDist_1, tanDist_2, radDist_3);
//////////////////////////////////////// Mbsbahru ///////////////////////////////////////////////

////////// The Pose Estimations based on formulation by Zizien - Alan (Zih-En) Tseng ////////////
             Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
             Calib3d.Rodrigues(rvec, CamRotationMatrix);


             CamRotationMatrix.get(0, 0, rMat);

             double[][] rMatArr = {
                     {rMat[0], rMat[1], rMat[2]},
                     {rMat[3], rMat[4], rMat[5]},
                     {rMat[6], rMat[7], rMat[8]}
             };


             Core.multiply(tvec, new Scalar(-1), tvecNeg);
             Core.transpose(CamRotationMatrix, rMatTranspose);
             Core.gemm(rMatTranspose, tvecNeg, 1, new Mat(), 0, CamLinPositions); // matrix multiplication: CamLinPositions=rMatTranspose*tvecNeg

             CamLinPositions.get(0, 0, CamLinPositionsData);

             double phi = Math.atan2(rMatArr[2][1], rMatArr[2][2]);
             double theta = Math.atan2(-rMatArr[2][0], Math.sqrt(rMatArr[2][1] * rMatArr[2][1] + rMatArr[2][2] * rMatArr[2][2]));
             double psi = Math.atan2(rMatArr[1][0], rMatArr[0][0]);

             CamAngPositionsData[0] = 180-((float)Math.toDegrees(phi)+360)%360;
             CamAngPositionsData[1] = (float)Math.toDegrees(theta);
             CamAngPositionsData[2] = (float)Math.toDegrees(psi);


     ///// code modified & hinted by Alan (Zizien) ////////
             // correct parameters in world frame
             // R_in_w = R^T
             // t_in_w = Rt
             t_in_w[0] = 0;
             t_in_w[1] = 0;
             t_in_w[2] = 0;
             for (int i = 0; i <= 2; i++) {
                 for (int j = 0; j <= 2; j++) {
                     R_in_w[j][i] = rMatArr[i][j];
                     t_in_w[i] += R_in_w[j][i] * tvecArr[j];
                 }
             }
     ///// code modified & hinted by Alan (Zizien) ////////
////////////////////////////////////////////// zizien /////////////////////////////////////////////

/////////////// The Kalman Filter framework based on formulation by Ushahzad - Usman Shahzad ///////////////////
             if(toggleKalmanFilter) {
                 double[] new_measurement = {CamLinPositionsData[0], CamLinPositionsData[1], CamLinPositionsData[2], CamAngPositionsData[0], CamAngPositionsData[1], CamAngPositionsData[2]};
                 camKF.predict(); // the filter is implemented in "KalmanFilter.java"
                 camKF.correction(new_measurement);
                 filteredState = camKF.getState();
                 euc_dist_R = Math.sqrt(Math.pow(filteredState[0], 2) + Math.pow(filteredState[1], 2) + Math.pow(filteredState[2], 2));
             }
/////////////////////////////// ushahzad //////////////////////////////////////////////////////////////////////////

///// The Control Strategy & Low Pass Filter based on formulation by Mohd - Mohammed Fouad Al-Buhlaigah ///////////
             lastUpdateTime = System.nanoTime();
             if(toggleLowpassFilter) {
                 if (lastUpdateTime != 0) {
                     CamLinPositionsData[0] += (x_last - CamLinPositionsData[0])/1.51;
                     CamLinPositionsData[1] += (y_last - CamLinPositionsData[1])/1.51;
                     CamLinPositionsData[2] += (z_last - CamLinPositionsData[2])/1.51;
                     CamAngPositionsData[0] += (phi_last - CamAngPositionsData[0])/1.51;
                     CamAngPositionsData[1] += (theta_last - CamAngPositionsData[1])/1.51;
                     CamAngPositionsData[2] += (psi_last - CamAngPositionsData[2])/1.51;

                     x_last = CamLinPositionsData[0];
                     y_last = CamLinPositionsData[1];
                     z_last = CamLinPositionsData[2];
                     phi_last = CamAngPositionsData[0];
                     theta_last = CamAngPositionsData[1];
                     psi_last = CamAngPositionsData[2];
                 }
             }
             double R_ref_diff = REFERENCE_R - euc_dist_R;
             double phi_ref_diff = REFERENCE_PHI - filteredState[3];
             double theta_ref_diff = REFERENCE_THETA - filteredState[4];
             double psi_ref_diff = REFERENCE_PSI - filteredState[5];
             if (R_ref_diff > TOLERANCE_R){
                 u_control = new double[][]{
                         {-1},
                         {0},
                         {0},
                         {0}
                 };
                 Imgproc.putText(mRgba, "BACK!", new Point(mRgba.cols()*0.1, mRgba.rows()*0.95), Imgproc.FONT_HERSHEY_SIMPLEX, 2, BLUE_COLOR, 5);
             }
             else if (R_ref_diff < -TOLERANCE_R){
                 u_control = new double[][]{
                         {1},
                         {0},
                         {0},
                         {0}
                 };
                 Imgproc.putText(mRgba, "FORWARD!", new Point(mRgba.cols()*0.1, mRgba.rows()*0.95), Imgproc.FONT_HERSHEY_SIMPLEX, 2, BLUE_COLOR, 5);
             }
             if ( phi_ref_diff > TOLERANCE_PHI){
                 u_control = new double[][]{
                         {0},
                         {1},
                         {0},
                         {0}
                 };
                 Imgproc.putText(mRgba, "DOWN!", new Point(mRgba.cols()*0.4, mRgba.rows()*0.95), Imgproc.FONT_HERSHEY_SIMPLEX, 2, BLUE_COLOR, 5);
             }
             else if (phi_ref_diff < -TOLERANCE_PHI){
                 u_control = new double[][]{
                         {0},
                         {-1},
                         {0},
                         {0}
                 };
                 Imgproc.putText(mRgba, "UP!", new Point(mRgba.cols()*0.4, mRgba.rows()*0.95), Imgproc.FONT_HERSHEY_SIMPLEX, 2, BLUE_COLOR, 5);
             }
             if (theta_ref_diff > TOLERANCE_THETA){
                 u_control = new double[][]{
                         {0},
                         {0},
                         {1},
                         {0}
                 };
                 Imgproc.putText(mRgba, "RIGHT!", new Point(mRgba.cols()*0.6, mRgba.rows()*0.95), Imgproc.FONT_HERSHEY_SIMPLEX, 2, BLUE_COLOR, 5);
             }
             else if (theta_ref_diff < -TOLERANCE_THETA){
                 u_control = new double[][]{
                         {0},
                         {0},
                         {-1},
                         {0}
                 };
                 Imgproc.putText(mRgba, "LEFT!", new Point(mRgba.cols()*0.6, mRgba.rows()*0.95), Imgproc.FONT_HERSHEY_SIMPLEX, 2, BLUE_COLOR, 5);
             }
             if (psi_ref_diff > TOLERANCE_PSI){
                 u_control = new double[][]{
                         {0},
                         {0},
                         {0},
                         {1}
                 };
                 Imgproc.putText(mRgba, "CCWise!", new Point(mRgba.cols()*0.8, mRgba.rows()*0.95), Imgproc.FONT_HERSHEY_SIMPLEX, 2, BLUE_COLOR, 5);
             }
             else if (psi_ref_diff < -TOLERANCE_PSI){
                 u_control = new double[][]{
                         {0},
                         {0},
                         {0},
                         {-1}
                 };
                 Imgproc.putText(mRgba, "CWise!", new Point(mRgba.cols()*0.8, mRgba.rows()*0.95), Imgproc.FONT_HERSHEY_SIMPLEX, 2, BLUE_COLOR, 5);
             }
             if (-TOLERANCE_R < R_ref_diff && R_ref_diff < TOLERANCE_R && -TOLERANCE_PHI < phi_ref_diff && phi_ref_diff < TOLERANCE_PHI && -TOLERANCE_THETA < theta_ref_diff && theta_ref_diff < TOLERANCE_THETA && -TOLERANCE_PSI < psi_ref_diff && psi_ref_diff < TOLERANCE_PSI){
                 Imgproc.putText(mRgba, "Pose Achieved!", new Point(mRgba.cols()*0.3, mRgba.rows()*0.95), Imgproc.FONT_HERSHEY_SIMPLEX, 3, YELLOW_COLOR, 5);
             }
 //////////////////////////////////////////////////////////////// mohd ///////////////////////////////////////////////////////
         }

//         Imgproc.rectangle(mRgba, new Point(mRgba.cols()*0.2, mRgba.rows()*0.2), new Point(mRgba.cols()*0.8, mRgba.rows()*0.8), GREEN_COLOR);

         if (isRecording) {
             String dataString = System.currentTimeMillis() + "," +
                     Arrays.toString(CamLinPositionsData).replaceAll("[\\[\\] ]", "") + "," +
                     Arrays.toString(CamAngPositionsData).replaceAll("[\\[\\] ]", "") + "," +
                     Arrays.toString(ImuAccelData).replaceAll("[\\[\\] ]", "") + "," +
                     Arrays.toString(ImuGyroData).replaceAll("[\\[\\] ]", "") + "," +
                     Arrays.toString(ImuOrientationData).replaceAll("[\\[\\] ]", "") + "," +
                     centroidBase.x + "," +
                     centroidBase.y + "," +
                     filteredState[0] + "," +
                     filteredState[1] + "," +
                     filteredState[2] + ","+
                     filteredState[3] + "," +
                     filteredState[4] + "," +
                     filteredState[5] + ",";

             saveDataToCSV(dataString, "navarch568.csv");
         }

         String refPosText = String.format("ref_R: %.2f ref_phi: %.2f ref_theta: %.2f ref_psi: %.2f", REFERENCE_R, REFERENCE_PHI, REFERENCE_THETA, REFERENCE_PSI);
         String camLinPosText = String.format("x: %.2f y: %.2f z: %.2f R: %.2f", CamLinPositionsData[0], CamLinPositionsData[1], CamLinPositionsData[2], euc_dist_R);
         String camAngPosText = String.format("phi: %.2f theta: %.2f psi: %.2f", CamAngPositionsData[0], CamAngPositionsData[1], CamAngPositionsData[2]);
         String camFilteredPosText = String.format("KalFiltd x: %.2f y: %.2f z: %.2f Phi: %.2f Theta: %.2f Psi: %.2f", filteredState[0], filteredState[1], filteredState[2], filteredState[3], filteredState[4], filteredState[5]);
         String accelGyroText = String.format("ImuRaw U: %.2f V: %.2f W: %.2f p: %.2f  q: %.2f  r: %.2f", ImuAccelData[0], ImuAccelData[1], ImuAccelData[2], ImuGyroData[0], ImuGyroData[1], ImuGyroData[2]);
//         String objDimText = String.format("objLength: %.2f objWidth: %.2f", BASE_REAL_LENGTH, BASE_REAL_WIDTH);
//         String magnetRawText = String.format("ImuMagnetRaw X: %.2f Y: %.2f Z: %.2f", ImuMagnetData[0], ImuMagnetData[1], ImuMagnetData[2]);
//         String orientationText = String.format("ImuAngPos Phi: %.2f Theta: %.2f Psi: %.2f", ImuOrientationData[0], ImuOrientationData[1], ImuOrientationData[2]);
//         String positionsText = String.format("ImuLinPos x: %.2f y: %.2f z: %.2f", ImuPositionsData[0], ImuPositionsData[1], ImuPositionsData[2]);
//         String velocitiesText = String.format("ImuVel u: %.2f v: %.2f w: %.2f", ImuVelocitiesData[0], ImuVelocitiesData[1], ImuVelocitiesData[2]);
         Imgproc.putText(mRgba, refPosText, new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 1, YELLOW_COLOR, 4);
         Imgproc.putText(mRgba, accelGyroText, new Point(10, 60), Imgproc.FONT_HERSHEY_SIMPLEX, 1, YELLOW_COLOR, 4);
         Imgproc.putText(mRgba, camLinPosText, new Point(10, 90), Imgproc.FONT_HERSHEY_SIMPLEX, 1, YELLOW_COLOR, 4);
         Imgproc.putText(mRgba, camAngPosText, new Point(10, 120), Imgproc.FONT_HERSHEY_SIMPLEX, 1, YELLOW_COLOR, 4);
         Imgproc.putText(mRgba, camFilteredPosText, new Point(10, 150), Imgproc.FONT_HERSHEY_SIMPLEX, 1, YELLOW_COLOR, 4);

//         Imgproc.putText(mRgba, objDimText, new Point(10, 180), Imgproc.FONT_HERSHEY_SIMPLEX, 1, YELLOW_COLOR, 4);
//         Imgproc.putText(mRgba, magnetRawText, new Point(10, 210), Imgproc.FONT_HERSHEY_SIMPLEX, 1, BLACK_COLOR, 2);
//         Imgproc.putText(mRgba, orientationText, new Point(10, 240), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 0, 0), 2);
//         Imgproc.putText(mRgba, positionsText, new Point(10, 270), Imgproc.FONT_HERSHEY_SIMPLEX, 1, BLUE_COLOR, 2);
//         Imgproc.putText(mRgba, velocitiesText, new Point(10, 280), Imgproc.FONT_HERSHEY_SIMPLEX, 1, BLUE_COLOR, 2);

     return mRgba;
     }


     public int getObjek(List<MatOfPoint> contours) {
         int maxHeight = 0;
         int counter = 0;

         MatOfPoint approx = new MatOfPoint();

         for (int i = 0; i < contours.size(); i++) {
             MatOfPoint tempContour = contours.get(i);
             MatOfPoint2f newMat = new MatOfPoint2f(tempContour.toArray());
             MatOfPoint2f newApprox = new MatOfPoint2f(approx.toArray());

             Imgproc.approxPolyDP(newMat, newApprox, Imgproc.arcLength(newMat, true) * 0.03, true);

             Rect rect = Imgproc.boundingRect(contours.get(i));

             if (rect.height > maxHeight && newApprox.toArray().length > 3 && rect.height > limT && rect.width > limL) {
                 maxHeight = rect.height;
                 counter = i;
             }
         }
         return counter;
     }

    MatOfPoint hull2points(MatOfInt hull, MatOfPoint contour) {
        List<Integer> indexes = hull.toList();
        List<Point> points = new ArrayList<>();
        MatOfPoint point= new MatOfPoint();
        for(Integer index:indexes) {
            points.add(contour.toList().get(index));
        }
        point.fromList(points);
        return point;
    }

    double lengthBetweenPoint(Point centerA, Point centerB){
        return Math.sqrt(Math.pow(centerA.x-centerB.x,2) + Math.pow(centerA.y-centerB.y,2));
    }

    Point lineCenter(Point point1, Point point2){
         return new Point(point1.x + (point2.x - point1.x)/2, point1.y + (point2.y - point1.y)/2);
    }

    public double drawCalcLine (Point point1, Point point2){
        double length = lengthBetweenPoint(point1, point2);
        Imgproc.putText(mRgba, String.valueOf((int)length), lineCenter(point1, point2), Imgproc.FONT_HERSHEY_SIMPLEX, 1, GREEN_COLOR, 2);
        Imgproc.line(mRgba, point1, point2, new Scalar(1, 1, 1), 1, Imgproc.LINE_AA);

        return length;
    }

    public Point calcIntersectionPoints(Point point1, Point point2, Point point3, Point point4){
        // a1x + b1y = c1
        double a1 = point2.y - point1.y;
        double b1 = point1.x - point2.x;
        double c1 = a1*(point1.x) + b1*(point1.y);

        // a2x + b2y = c2
        double a2 = point4.y - point3.y;
        double b2 = point3.x - point4.x;
        double c2 = a2*(point3.x)+ b2*(point3.y);

        double determinant = a1*b2 - a2*b1;

        if (determinant == 0)
        {
            return new Point(mRgba.cols()/2, mRgba.rows()/2);
        }
        else
        {
            double x = (b2*c1 - b1*c2)/determinant;
            double y = (a1*c2 - a2*c1)/determinant;
            return new Point(x, y);
        }
    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        switch (sensorEvent.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                System.arraycopy(sensorEvent.values, 0, ImuAccelData, 0, sensorEvent.values.length);
                break;
            case Sensor.TYPE_GYROSCOPE:
                System.arraycopy(sensorEvent.values, 0, ImuGyroData, 0, sensorEvent.values.length);
                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                ImuMagnetData = sensorEvent.values.clone();
                break;
        }
        noiseEstimator.addSample(ImuAccelData, ImuGyroData);

        lastUpdateTime = sensorEvent.timestamp;
        if (lastUpdateTime != 0 && isUpdating) {
            long currentTime = System.currentTimeMillis();
            float deltaTime = (currentTime - lastUpdateTime) / 1000.0f;
            lastUpdateTime = currentTime;

            for (int i = 0; i < 3; i++) {
                velUnfiltered[i] += ImuAccelData[i] * deltaTime;
                ImuVelocitiesData[i] = alpha * velUnfiltered[i] + (1 - alpha) * ImuVelocitiesData[i];

        /////// code modified by Alan (Zizien) ////////
                // Apply high-pass filter to velocities
                velUnfiltered[i] += ImuAccelData[i] * deltaTime;
                ImuVelocitiesData[i] = alpha * velUnfiltered[i] - (1 - alpha) * ImuVelocitiesData[i];
        ////// code modified by Alan (Zizien) ////////

                posUncorrected[i] += ImuVelocitiesData[i] * deltaTime;
                float driftCorrection = driftRate * deltaTime;
                ImuPositionsData[i] = posUncorrected[i] - driftCorrection;
            }
        }

            if (sensorEvent.sensor.getType() == Sensor.TYPE_ACCELEROMETER || sensorEvent.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
                if (SensorManager.getRotationMatrix(ImuRotationMatrix, ImuInclinationMatrix, ImuAccelData, ImuMagnetData)) {
                    SensorManager.getOrientation(ImuRotationMatrix, ImuOrientationData);
                    ImuOrientationData[0] = (float)Math.toDegrees(ImuOrientationData[0]);
                    ImuOrientationData[1] = (float)Math.toDegrees(ImuOrientationData[1]);
                    ImuOrientationData[2] = (float)Math.toDegrees(ImuOrientationData[2]);
                }
            }
        }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }
    public void saveDataToCSV(String dataString, String fileName) {
        String baseDir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS).getAbsolutePath();
        String filePath = baseDir + File.separator + fileName;
        File file = new File(filePath);
        FileWriter fileWriter = null;

        boolean isNewFile = !file.exists();

        try {
            fileWriter = new FileWriter(file, true);

            if (isNewFile) {
                fileWriter.append("Timestamp(ms),CamLinPositionsX(cm),CamLinPositionsY(cm),CamLinPositionsZ(cm),CamAngPositionsPhi(deg),CamAngPositionsTheta(deg),CamAngPositionsPsi(deg),ImuAccelX(m/s^2),ImuAccelY(m/s^2),ImuAccelZ(m/s^2),ImuGyroX(rad/s),ImuGyroY(rad/s),ImuGyroZ(rad/s),ImuOrientationPhi(deg),ImuOrientationTheta(deg),ImuOrientationPsi(deg),CentroidBaseX(px),CentroidBaseY(px),KalmanFiltered_X,KalmanFiltered_Y,KalmanFiltered_Z,KalmanFiltered_Phi,KalmanFiltered_Theta,KalmanFiltered_Psi\n");
            }

            fileWriter.append(dataString);
            fileWriter.append("\n");

            Log.d("CSV", "Data saved to " + filePath);
        } catch (IOException e) {
            Log.e("CSV", "Error writing to file", e);
        } finally {
            if (fileWriter != null) {
                try {
                    fileWriter.flush();
                    fileWriter.close();
                } catch (IOException e) {
                    Log.e("CSV", "Error closing writer", e);
                }
            }
        }
    }
}