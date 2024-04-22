package com.mbsbahru.na568Teamproject_MohammedAlanUsmanBahru;

import java.util.List;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import android.annotation.SuppressLint;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.SeekBar;

import androidx.appcompat.app.AppCompatActivity;

public class SeekBarHsvSeg extends AppCompatActivity implements CvCameraViewListener2, OnTouchListener {
   private static final String TAG = "SeekActivity";
	private static final int    VIEW_MODE_HSV_SEGMENTATION   = 0;
   private static final int    VIEW_MODE_REF_INPUT   = 1;
   private static final int    VIEW_MODE_POSE_ACT   = 2;
   private MenuItem     mItemHsvSegmentation; 
   private MenuItem     mItemHsvRefInput; 
   private MenuItem     mItemPoseAction;
   private int          mViewMode;
	private CameraBridgeViewBase mOpenCvCameraView;
   
   private Mat mRgba;
   private Mat mHSV;
   public static int Hmin1 = 0;
   public static int Hmax1 = 10;
   public static int Smin1 = 69;
   public static int Smax1 = 262;
   public static int Vmin1 = 92;
   public static int Vmax1 = 246;
   public static int Erode1 = 1;
   public static int Dilate1 = 1;
   public static int MedianBlur1 = 0;
   public static int GaussianBlur1 = 0;
	public static int BilBlurDval = 0;
	public static double BilBlurSigSpace = 0;


   private SeekBar seekHmin1;
   private SeekBar seekHmax1;
   private SeekBar seekSmin1;
   private SeekBar seekSmax1;
   private SeekBar seekVmin1;
   private SeekBar seekVmax1;
   private SeekBar seekErode;
   private SeekBar seekDilate;
	private SeekBar seekMedianBlur;
	private SeekBar seekGaussianBlur;
	private SeekBar seekBilBlurCol;
	private SeekBar seekBilBlurSpa;

   private boolean mIsDisplayTouched;
   private SeekAdapter sH;
   private SeekAdapter sS;
   private SeekAdapter sV;
   private SeekAdapter sED;
   private SeekAdapter sMG;
	private SeekAdapter sBf;
   private Mat mSpectrum;
   private Scalar mBlobColorRgba;
   private Scalar mBlobColorHsv;
   private Size SPECTRUM_SIZE;
   private BlobDetector mDetector;  
   protected boolean bIsHPressed;
   protected boolean bIsSPressed;
   protected boolean bIsVPressed;
   protected boolean bIsEDPressed;
   private boolean bIsMGPressed;
	private boolean bIsBfPressed;
   private Mat mDisplay;
   private Mat mBiner;
   private boolean idxBiner =false;
	private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {  
		
		   @Override  
		     public void onManagerConnected(int status) {  
		       switch (status) {  
		         case LoaderCallbackInterface.SUCCESS:  
		         {  
		        	 Log.i(TAG, "OpenCV loaded successfully");
		           mOpenCvCameraView.enableView();
		           mOpenCvCameraView.setOnTouchListener(SeekBarHsvSeg.this);
		         } break;  
		         default:
		         {  
		           super.onManagerConnected(status);  
		         } break;  
		       }  
		     }  
		   };
	private Scalar CONTOUR_COLOR;
		   public void SeekBarValBase() {
			     Log.i(TAG, "Instantiated new " + this.getClass());
			   }
		   
	@Override
    public void onCreate(Bundle savedInstanceState) {
    	 Log.i(TAG, "called onCreate"); 
    	super.onCreate(savedInstanceState); 
    	getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		setContentView(R.layout.biner_base);

		mOpenCvCameraView = findViewById(R.id.activity_surface_view);
		mOpenCvCameraView.setVisibility(CameraBridgeViewBase.VISIBLE);
//		mOpenCvCameraView.setMaxFrameSize(2400, 1080);
        mOpenCvCameraView.setCvCameraViewListener(this);
    }
    
    @Override  
    public boolean onCreateOptionsMenu(Menu menu) {  
    	Log.i(TAG, "called onCreateOptionsMenu");
    	mItemHsvSegmentation = menu.add("HSV Object Segmentation");
    	mItemHsvRefInput = menu.add("Reference Input");
    	mItemPoseAction = menu.add("Pose Perfect Action");
    	return true;
    }  
    
    @Override  
    public void onPause()  
    {  
      super.onPause();  
      if (mOpenCvCameraView != null)  
        mOpenCvCameraView.disableView();
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
	}

	@Override
	public void onDestroy() {
		super.onDestroy();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	@Override
	public void onCameraViewStarted(int width, int height) {
		// TODO Auto-generated method stub
		mRgba = new Mat(height, width, CvType.CV_8UC4);
    	mHSV = new Mat(height, width, CvType.CV_8UC3);
    	mDetector = new BlobDetector();
	    sH = new SeekAdapter();
	    sS = new SeekAdapter();
	    sV = new SeekAdapter();
	    sED = new SeekAdapter();
	    sMG = new SeekAdapter();
		sBf = new SeekAdapter();
	    mSpectrum = new Mat();
	    mBlobColorRgba = new Scalar(255);
	    mBlobColorHsv = new Scalar(255);
	    SPECTRUM_SIZE = new Size(mRgba.cols()/1.5, 64);
	    CONTOUR_COLOR = new Scalar(255,0,0,255);
    	mDisplay = new Mat();
    	mBiner = new Mat(height,width,CvType.CV_8UC1);
	}

	@Override
	public void onCameraViewStopped() {
		// TODO Auto-generated method stub
		mRgba.release();
	    mHSV.release();
	    mDisplay.release();
	    mBiner.release();
	}

	@Override
	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
		// TODO Auto-generated method stub
		final int viewMode = mViewMode;
		mRgba = inputFrame.rgba();
		Imgproc.cvtColor(mRgba,mRgba,Imgproc.COLOR_BGRA2BGR);
		Imgproc.medianBlur(mRgba, mRgba, 2*MedianBlur1+1);
//		Imgproc.bilateralFilter(mRgba, mHSV, BilBlurDval, BilBlurSigSpace, BilBlurSigSpace, Core.BORDER_DEFAULT);
		Imgproc.GaussianBlur(mRgba, mRgba, new Size(2*GaussianBlur1+1,2*GaussianBlur1+1),0,0);

		if (viewMode==VIEW_MODE_REF_INPUT){	
			Intent intent = new Intent(SeekBarHsvSeg.this, ReferenceInput.class);
        	startActivity(intent);
        	finish();
        }
		else if (viewMode==VIEW_MODE_POSE_ACT){
			Intent intent = new Intent(SeekBarHsvSeg.this, MainActivity.class);
        	startActivity(intent);
        	finish();
		}
		else if (viewMode==VIEW_MODE_HSV_SEGMENTATION){
			Button bH = (Button)findViewById(R.id.tombolH);
    		Button bS = (Button)findViewById(R.id.tombolS);
			Button bV = (Button)findViewById(R.id.tombolV);
			Button bED = (Button)findViewById(R.id.tombolED);
			Button bMG = (Button)findViewById(R.id.tombolMG);

			seekHmin1=(SeekBar) findViewById(R.id.min);
			seekHmax1=(SeekBar) findViewById(R.id.max);
			seekSmin1=(SeekBar) findViewById(R.id.min); 
			seekSmax1=(SeekBar) findViewById(R.id.max);
			seekVmin1=(SeekBar) findViewById(R.id.min);
			seekVmax1=(SeekBar) findViewById(R.id.max);
			seekErode=(SeekBar) findViewById(R.id.min);
			seekDilate=(SeekBar) findViewById(R.id.max);
			seekMedianBlur=(SeekBar) findViewById(R.id.min);
			seekGaussianBlur=(SeekBar) findViewById(R.id.max);
			seekBilBlurCol=(SeekBar) findViewById(R.id.min);
			seekBilBlurSpa=(SeekBar) findViewById(R.id.max);

			bH.setOnClickListener(new View.OnClickListener(){
				@Override
				public void onClick(View v) {
					// TODO Auto-generated method stub
					sH.setProgress(seekHmin1, seekHmax1, Hmin1, Hmax1, 262);
					sH.seekProgress();
					bIsHPressed = true;
					bIsSPressed = false;
					bIsVPressed = false;
					bIsEDPressed = false;
					bIsMGPressed = false;
					bIsBfPressed = false;
					mIsDisplayTouched = false;
				}
			});
			bS.setOnClickListener(new View.OnClickListener(){
				@Override
				public void onClick(View v) {
					// TODO Auto-generated method stub
					sS.setProgress(seekSmin1, seekSmax1, Smin1, Smax1, 262);
					sS.seekProgress();
					bIsSPressed = true;
					bIsHPressed = false;
					bIsVPressed = false;
					bIsEDPressed = false;
					bIsMGPressed = false;
					bIsBfPressed = false;
					mIsDisplayTouched = false;
				}
			});
			bV.setOnClickListener(new View.OnClickListener(){
				@Override
				public void onClick(View v) {
					// TODO Auto-generated method stub
					sV.setProgress(seekVmin1, seekVmax1, Vmin1, Vmax1, 262);
					sV.seekProgress();
					sV.seekBarChange();
					bIsHPressed = false;
					bIsSPressed = false;
					bIsEDPressed = false;
					bIsMGPressed = false;
					bIsBfPressed = false;
					bIsVPressed = true;
					mIsDisplayTouched = false;
				}
			});
			bED.setOnClickListener(new View.OnClickListener(){
				@Override
				public void onClick(View v) {
					// TODO Auto-generated method stub
					sED.setProgress(seekErode, seekDilate, Erode1, Dilate1, 21);
					sED.seekProgress();
					sED.seekBarChange();
					bIsEDPressed = true;
					bIsHPressed = false;
					bIsSPressed = false;
					bIsVPressed = false;
					bIsMGPressed = false;
					bIsBfPressed = false;
					mIsDisplayTouched = false;
					
					if(idxBiner)
						idxBiner = false;
						else
							idxBiner = true;
				}
			});

			bMG.setOnClickListener(new View.OnClickListener(){
				@Override
				public void onClick(View v) {
					// TODO Auto-generated method stub
					sMG.setProgress(seekMedianBlur, seekGaussianBlur, MedianBlur1, GaussianBlur1, 16);
					sMG.seekProgress();
					sMG.seekBarChange();
					bIsEDPressed = false;
					bIsHPressed = false;
					bIsSPressed = false;
					bIsVPressed = false;
					bIsMGPressed = true;
					bIsBfPressed = false;
					mIsDisplayTouched = false;

					if(idxBiner)
						idxBiner = false;
					else
						idxBiner = true;
				}
			});


			if(bIsHPressed){
				Hmin1 = sH.getFirstVal();
				Hmax1 = sH.getSecondVal();
				sH.setProgress(seekHmin1, seekHmax1, Hmin1, Hmax1, 262);
				sH.seekProgress();
				sH.seekBarChange();
			}
			
			if(bIsSPressed){
				Smin1 = sS.getFirstVal();
				Smax1 = sS.getSecondVal();
				sS.setProgress(seekSmin1, seekSmax1, Smin1, Smax1, 262);
				sS.seekProgress();
				sS.seekBarChange();
			}
			
			if(bIsVPressed){
				Vmin1 = sV.getFirstVal();
				Vmax1 = sV.getSecondVal();
				sV.setProgress(seekVmin1, seekVmax1, Vmin1, Vmax1, 262);
				sV.seekProgress();
				sV.seekBarChange();
			}
			
			if(bIsEDPressed){
				Erode1 = sED.getFirstVal();
				Dilate1 = sED.getSecondVal();
				sED.setProgress(seekErode, seekDilate, Erode1, Dilate1, 21);
				sED.seekProgress();
				sED.seekBarChange();
			}

			if(bIsMGPressed){
				MedianBlur1 = sMG.getFirstVal();
				GaussianBlur1 = sMG.getSecondVal();
				sMG.setProgress(seekMedianBlur, seekGaussianBlur, MedianBlur1, GaussianBlur1, 16);
				sMG.seekProgress();
				sMG.seekBarChange();
			}

			if(idxBiner){
		        Scalar hsv_min2 = new Scalar(Hmin1, Smin1, Vmin1, 0);  
		        Scalar hsv_max2 = new Scalar(Hmax1, Smax1, Vmax1, 0);

				Imgproc.cvtColor(mRgba, mHSV, Imgproc.COLOR_RGB2HSV_FULL,4);
		        Core.inRange(mHSV, hsv_min2, hsv_max2, mBiner);
//				Imgproc.medianBlur(mBiner, mBiner, 2*MedianBlur1+1);
		        Imgproc.erode(mBiner, mBiner, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size (2*Erode1+1,2*Erode1+1), new Point (Erode1,Erode1)));
		        Imgproc.dilate(mBiner, mBiner, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size (2*Dilate1+1,2*Dilate1+1), new Point (Dilate1,Dilate1)));
				Imgproc.dilate(mBiner, mBiner, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size (2*Dilate1+1,2*Dilate1+1), new Point (Dilate1,Dilate1)));
				Imgproc.erode(mBiner, mBiner, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size (2*Erode1+1,2*Erode1+1), new Point (Erode1,Erode1)));
				mDisplay = mBiner;
			}
			else{
				if(mIsDisplayTouched){
					Hmin1 = (int) mDetector.getHmin();
					Hmax1 = (int) mDetector.getHmax();
					Smin1 = (int) mDetector.getSmin();
					Smax1 = (int) mDetector.getSmax();
					Vmin1 = (int) mDetector.getVmin();
					Vmax1 = (int) mDetector.getVmax();
					}			
            mDetector.process(mRgba, new Scalar(Hmin1, Smin1, Vmin1), new Scalar(Hmax1, Smax1, Vmax1), Dilate1, Erode1);
            List<MatOfPoint> contours = mDetector.getContours();
//            Imgproc.fillPoly(mRgba, contours, new Scalar((mBlobColorRgba.val[0]+63>255)?mBlobColorRgba.val[0]-63:mBlobColorRgba.val[0]+63, (mBlobColorRgba.val[1]+63>255)?mBlobColorRgba.val[1]-63:mBlobColorRgba.val[1]+63, (mBlobColorRgba.val[2]+63>255)?mBlobColorRgba.val[2]-63:mBlobColorRgba.val[2]+63));
            Imgproc.fillPoly(mRgba, contours, CONTOUR_COLOR);
            
            Log.e(TAG, "Contours count: " + contours.size());
            
            
            MatOfPoint approx =  new MatOfPoint();
            
            double max = 0;
            int cornerCnt = 0;
            for (int i = 0; i < contours.size(); i++){
            	MatOfPoint tempContour = contours.get(i);
            	MatOfPoint2f newMat = new MatOfPoint2f( tempContour.toArray() );
            	MatOfPoint2f newApprox = new MatOfPoint2f( approx.toArray() );
                       
            	Imgproc.approxPolyDP(newMat, newApprox, Imgproc.arcLength(newMat, true)*0.03, true);
     	   
     	   if (Imgproc.contourArea(contours.get(i)) > max){
     		   max = Imgproc.contourArea(contours.get(i));
      		   cornerCnt = newApprox.toArray().length;
     	   }
           }

            Imgproc.putText(mRgba, "Curve:   " + cornerCnt, new Point (10, mRgba.rows() * 0.7), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(0, 255, 0), 1);

            Imgproc.drawContours(mRgba, contours, -1, new Scalar((mBlobColorRgba.val[0]+63>255)?mBlobColorRgba.val[0]-63:mBlobColorRgba.val[0]+63, (mBlobColorRgba.val[1]+63>255)?mBlobColorRgba.val[1]-63:mBlobColorRgba.val[1]+63, (mBlobColorRgba.val[2]+63>255)?mBlobColorRgba.val[2]-63:mBlobColorRgba.val[2]+63), 5);
            Imgproc.drawContours(mRgba, contours, -1, new Scalar(0, 255, 0, 255), 2);

            Imgproc.putText(mRgba, "HSVmin:   " + Hmin1 + ",  " + Smin1 + ",  " + Vmin1 + ",  " + Dilate1, new Point (10, mRgba.rows() * 0.2+20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(0, 255, 0), 1);
			Imgproc.putText(mRgba, "HSVmax:   " + Hmax1 + ",  " + Smax1 + ",  " + Vmax1 + ",  " + Erode1, new Point (10, mRgba.rows() * 0.2+50), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(0, 255, 0), 1);
			Imgproc.putText(mRgba, "Ero: " + Erode1 + ",  Dil: " + Dilate1 + ",  Ga: " + GaussianBlur1 + ",  Me: " + MedianBlur1, new Point (10, mRgba.rows() * 0.2+80), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(0, 255, 0), 1);

//            Mat colorLabel = mRgba.submat(2, 34, 446, 478);
//            colorLabel.setTo(mBlobColorRgba);
            Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
            mSpectrum.copyTo(spectrumLabel);
			mDisplay = mRgba;
			}
			Imgproc.putText(mRgba, "HSV Base", new Point (10, mRgba.rows()*0.2), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 0, 255), 2);
		}
        return mDisplay;	
		 
}

public boolean onOptionsItemSelected(MenuItem item) {  
    
    	if (item == mItemHsvSegmentation) {  
      mViewMode = VIEW_MODE_HSV_SEGMENTATION;  
    } else if (item == mItemHsvRefInput) {
      mViewMode = VIEW_MODE_REF_INPUT;      
    }
		else if (item == mItemPoseAction){
      mViewMode = VIEW_MODE_POSE_ACT;      
    }
    return true;
}

@SuppressLint("ClickableViewAccessibility")
@Override
public boolean onTouch(View v, MotionEvent event) {
	int cols = mRgba.cols();
    int rows = mRgba.rows();

    int x = (int)(event.getX() - (v.getMeasuredWidth() - cols)/2);// * 0.25);
    int y = (int)(event.getY());// * 0.296296296);
    
    Log.i(TAG, "Touch image coordinates: (" + x + ", " + y + ")" + v.getMeasuredWidth());

    if ((x < 0) || (y < 0) || (x > cols) || (y > rows)) return false;

    Rect touchedRect = new Rect();

    touchedRect.x = (x>4) ? x-4 : 0;
    touchedRect.y = (y>4) ? y-4 : 0;

    touchedRect.width = (x+4 < cols) ? x + 4 - touchedRect.x : cols - touchedRect.x;
    touchedRect.height = (y+4 < rows) ? y + 4 - touchedRect.y : rows - touchedRect.y;

    Mat touchedRegionRgba = mRgba.submat(touchedRect);

    Mat touchedRegionHsv = new Mat();
    Imgproc.cvtColor(touchedRegionRgba, touchedRegionHsv, Imgproc.COLOR_RGB2HSV_FULL);

    int pointCount = touchedRect.width*touchedRect.height;
    
    mBlobColorHsv = Core.sumElems(touchedRegionHsv);
    
    for (int i = 0; i < mBlobColorHsv.val.length; i++)
        mBlobColorHsv.val[i] /= pointCount;
    
    mBlobColorRgba = convertScalarHsv2Rgba(mBlobColorHsv);
    
    Log.i(TAG, "Touched rgba color: (" + mBlobColorRgba.val[0] + ", " + mBlobColorRgba.val[1] +
            ", " + mBlobColorRgba.val[2] + ", " + mBlobColorRgba.val[3] + ")");

    mDetector.setHsvColor(mBlobColorHsv);
    
    Imgproc.resize(mDetector.getSpectrum(), mSpectrum, SPECTRUM_SIZE);

    mDetector.process(mRgba);
    
//    mIsColorSelected = true;
	bIsHPressed = false;
	bIsSPressed = false;
	bIsVPressed = false;
	bIsEDPressed = false;
    mIsDisplayTouched = true;

    touchedRegionRgba.release();
    touchedRegionHsv.release();
	
	return false;
}


private Scalar convertScalarHsv2Rgba(Scalar hsvColor) {
    Mat pointMatRgba = new Mat();
    Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
    Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

    return new Scalar(pointMatRgba.get(0, 0));
}
}