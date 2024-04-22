////////////////////// written based on Python files by Ushahzad - Usman Shahzad /////////////////////////////////

package com.mbsbahru.na568Teamproject_MohammedAlanUsmanBahru;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.CvType;

public class KalmanFilter {
    private Mat state;
    private int arrDim = 6;
    private Mat P;
    private Mat Q;
    private Mat R;
    private Mat F;
    private Mat H;

    public KalmanFilter(double[] initialState, double[] processNoiseStd, double[] measurementNoiseStd) {
        state = new Mat(arrDim, 1, CvType.CV_64F);
        state.put(0, 0, initialState);

        P = Mat.eye(arrDim, arrDim, CvType.CV_64F);
        Q = Mat.zeros(arrDim, arrDim, CvType.CV_64F);
        R = Mat.zeros(arrDim, arrDim, CvType.CV_64F);
        for (int i = 0; i < arrDim; i++) {
            Q.put(i, i, processNoiseStd[i] * processNoiseStd[i]);
            R.put(i, i, measurementNoiseStd[i] * measurementNoiseStd[i]);
        }

        F = Mat.eye(arrDim, arrDim, CvType.CV_64F);
        H = Mat.eye(arrDim, arrDim, CvType.CV_64F);
    }

    public void predict() {
        Mat F_transposed = new Mat();
        Core.transpose(F, F_transposed);
        Mat temp = new Mat();
        Core.gemm(F, state, 1, new Mat(), 0, temp);
        state = temp.clone();
        Core.gemm(F, P, 1, new Mat(), 0, temp); // Matrix multiplication: temp = F * P
        Core.gemm(temp, F_transposed, 1, new Mat(), 0, P); // Matrix multiplication: P = temp * F_transposed
        Core.add(P, Q, P);
    }

    public void correction(double[] measurement) {
        Mat y = new Mat(arrDim, 1, CvType.CV_64F);
        y.put(0, 0, measurement);

        if (state.rows() != arrDim || state.cols() != 1) {
            System.out.println("State matrix has incorrect dimensions.");
            return;
        }

        Mat measurementMat = new Mat();
        Core.gemm(H, state, 1, new Mat(), 0, measurementMat);

        Core.subtract(y, measurementMat, y);

        Mat Ht = new Mat();
        Core.transpose(H, Ht);
        Mat PHt = new Mat();
        Core.gemm(P, Ht, 1, new Mat(), 0, PHt);

        Mat S = new Mat();
        Core.gemm(H, PHt, 1, new Mat(), 0, S);
        Core.add(S, R, S);

        Mat S_inv = S.inv(Core.DECOMP_SVD);
        Mat K = new Mat();
        Core.gemm(PHt, S_inv, 1, new Mat(), 0, K);

        Mat Ky = new Mat();
        Core.gemm(K, y, 1, new Mat(), 0, Ky);
        Core.add(state, Ky, state);

        Mat KH = new Mat();
        Core.gemm(K, H, 1, new Mat(), 0, KH);
        Mat I = Mat.eye(P.rows(), P.cols(), CvType.CV_64F);
        Mat temp = new Mat();
        Core.subtract(I, KH, temp);
        Core.gemm(temp, P, 1, new Mat(), 0, P);
    }

    public double[] getState() {
        return new double[]{
                state.get(0, 0)[0],
                state.get(1, 0)[0],
                state.get(2, 0)[0],
                state.get(3, 0)[0],
                state.get(4, 0)[0],
                state.get(5, 0)[0],
        };
    }

    public void setProcessNoiseStd(double[] processNoiseStd) {
        for (int i = 0; i < processNoiseStd.length; i++) {
            double variance = Math.pow(processNoiseStd[i], 2);
            Q.put(i, i, variance*0.5);
        }
    }

    public void setMeasurementNoiseStd(double[] measurementNoiseStd) {
        for (int i = 0; i < measurementNoiseStd.length; i++) {
            double variance = Math.pow(measurementNoiseStd[i], 2);
            R.put(i, i, variance*0.5);
        }
    }
}