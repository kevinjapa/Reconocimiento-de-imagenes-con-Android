package ups.vision.practica31recfiguras;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.provider.MediaStore;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;


public class Parte2Activity extends AppCompatActivity {

    private Bitmap imagenBitmap, outputBitmap;
    private ImageView verImgOriginal, verImgProcesada;
    private static final int REQUEST_IMAGE_CAPTURE = 1;
    private static final int REQUEST_CAMERA_PERMISSION = 100;

    static {
        System.loadLibrary("parte2-lib");
    }


    @Override
    protected void onCreate(Bundle savedInstanceState){
        super.onCreate(savedInstanceState);
        setContentView(R.layout.parte2);
        verImgOriginal=findViewById(R.id.imgOriginal2);
        verImgProcesada= findViewById(R.id.imgResultante2);
        Button btnCamera = findViewById(R.id.btnCamara2);
        Button btnProbar = findViewById(R.id.btnMomentoHU);

        btnCamera.setOnClickListener(v -> {
            if (ContextCompat.checkSelfPermission(Parte2Activity.this, android.Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(Parte2Activity.this, new String[]{Manifest.permission.CAMERA}, REQUEST_CAMERA_PERMISSION);
            } else {
                openCamera();
            }
        });
        btnProbar.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v) {
                Intent intentHU = new Intent(Parte2Activity.this, MainActivity.class);
                startActivity(intentHU);
                finish();
            }
        });
    }
    @Override
    protected void onActivityResult(int requestCode, int resultCode, @Nullable Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == REQUEST_IMAGE_CAPTURE && resultCode == RESULT_OK) {
            Bundle extras = data.getExtras();
            imagenBitmap = (Bitmap) extras.get("data");
            verImgOriginal.setImageBitmap(imagenBitmap);
        }
    }
    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQUEST_CAMERA_PERMISSION) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                openCamera();
            } else {
                Toast.makeText(this, "Camera permission is required to use camera", Toast.LENGTH_SHORT).show();
            }
        }
    }
    private void openCamera() {
        Intent takePictureIntent = new Intent(MediaStore.ACTION_IMAGE_CAPTURE);
        if (takePictureIntent.resolveActivity(getPackageManager()) != null) {
            startActivityForResult(takePictureIntent, REQUEST_IMAGE_CAPTURE);
        }
    }

    public native String stringFromJNI2();
}
