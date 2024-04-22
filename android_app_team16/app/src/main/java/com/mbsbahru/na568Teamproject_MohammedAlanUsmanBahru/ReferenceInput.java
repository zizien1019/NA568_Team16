package com.mbsbahru.na568Teamproject_MohammedAlanUsmanBahru;

import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import androidx.appcompat.app.AppCompatActivity;

public class ReferenceInput extends AppCompatActivity {
    private static final String TAG = "RefInputActivity";
    private static final int    VIEW_MODE_HSV_SEGMENTATION   = 2;
    private static final int    VIEW_MODE_REF_INPUT   = 0;
    private static final int    VIEW_MODE_POSE_ACT   = 1;
    private int          mViewMode;
    private MenuItem mItemHsvSegmentation;
    private MenuItem     mItemHsvRefInput;
    private MenuItem     mItemPoseAction;
    private EditText editTextObjectWidth;
    private EditText editTextObjectLength;
    private EditText editTextReferenceR;
    private EditText editTextReferencePhi;
    private EditText editTextReferenceTheta;
    private EditText editTextReferencePsi;
    private Button btnSave;
    public static double objectLength = 34.5;
    public static double objectWidth = 15.5;
    public static double referenceR = 115.3;
    public static double referencePhi = -17;
    public static double referenceTheta = 21.3;
    public static double referencePsi = 13.3;


    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        Log.i(TAG, "called onCreateOptionsMenu");
        mItemHsvSegmentation = menu.add("HSV Object Segmentation");
        mItemHsvRefInput = menu.add("Reference Input");
        mItemPoseAction = menu.add("Pose Perfect Action");
        return true;
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
        updateActivityBasedOnViewMode();
        return true;
    }
    private void updateActivityBasedOnViewMode() {
        Intent intent;
        switch (mViewMode) {
            case VIEW_MODE_HSV_SEGMENTATION:
                intent = new Intent(this, SeekBarHsvSeg.class);
                startActivity(intent);
                finish();
                break;
            case VIEW_MODE_POSE_ACT:
                intent = new Intent(this, MainActivity.class);
                startActivity(intent);
                finish();
                break;
        }
    }
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_reference_input);

        editTextObjectLength = findViewById(R.id.editTextObjectLength);
        editTextObjectWidth = findViewById(R.id.editTextObjectWidth);
        editTextReferenceR = findViewById(R.id.editTextReferenceR);
        editTextReferencePhi = findViewById(R.id.editTextReferencePhi);
        editTextReferenceTheta = findViewById(R.id.editTextReferenceTheta);
        editTextReferencePsi = findViewById(R.id.editTextReferencePsi);
        btnSave = findViewById(R.id.btnSave);
            btnSave.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View view) {
//                    objectLength = Double.parseDouble(editTextObjectLength.getText().toString());
//                    objectWidth = Double.parseDouble(editTextObjectWidth.getText().toString());
//                    referenceR = Double.parseDouble(editTextReferenceR.getText().toString());
//                    referencePhi = Double.parseDouble(editTextReferencePhi.getText().toString());
//                    referenceTheta = Double.parseDouble(editTextReferenceTheta.getText().toString());
//                    referencePsi = Double.parseDouble(editTextReferencePsi.getText().toString());

                    objectLength = getDoubleFromEditText(editTextObjectLength, objectLength);
                    objectWidth = getDoubleFromEditText(editTextObjectWidth, objectWidth);
                    referenceR = getDoubleFromEditText(editTextReferenceR, referenceR);
                    referencePhi = getDoubleFromEditText(editTextReferencePhi, referencePhi);
                    referenceTheta = getDoubleFromEditText(editTextReferenceTheta, referenceTheta);
                    referencePsi = getDoubleFromEditText(editTextReferencePsi, referencePsi);

                    saveReferenceValues(objectLength, objectWidth, referenceR, referencePhi, referenceTheta, referencePsi);
//                finish();
                }

            });
        mViewMode = getIntent().getIntExtra("viewMode", VIEW_MODE_REF_INPUT);
        updateActivityBasedOnViewMode();
        }

//    }

    private void saveReferenceValues(double objectLength, double objectWidth, double referenceR, double referencePhi, double referenceTheta, double referencePsi) {
        SharedPreferences sharedPreferences = getSharedPreferences("AppSettings", MODE_PRIVATE);
        SharedPreferences.Editor editor = sharedPreferences.edit();
        editor.putFloat("OBJECT_LENGTH", (float) objectLength);
        editor.putFloat("OBJECT_WIDTH", (float) objectWidth);
        editor.putFloat("REFERENCE_R", (float) referenceR);
        editor.putFloat("REFERENCE_PHI", (float) referencePhi);
        editor.putFloat("REFERENCE_THETA", (float) referenceTheta);
        editor.putFloat("REFERENCE_PSI", (float) referencePsi);
        editor.apply();
    }

    private double getDoubleFromEditText(EditText editText, double defaultValue) {
        String text = editText.getText().toString();
        return text.isEmpty() ? defaultValue : Double.parseDouble(text);
    }

}
