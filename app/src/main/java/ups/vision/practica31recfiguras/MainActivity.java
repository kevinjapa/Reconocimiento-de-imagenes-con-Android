package ups.vision.practica31recfiguras;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.core.content.FileProvider;

import android.Manifest;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.provider.MediaStore;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import ups.vision.practica31recfiguras.databinding.ActivityMainBinding;

public class MainActivity extends AppCompatActivity {

    // Used to load the 'practica31recfiguras' library on application startup.
    static {
        System.loadLibrary("practica31recfiguras");
    }

    private static final int REQUEST_IMAGE_CAPTURE = 1;
    private static final int REQUEST_CAMERA_PERMISSION = 100;
    private ImageView verImgOriginal,verImg;
    private Uri photoURI;
    private ActivityMainBinding binding;
    private Bitmap imagenBitmap, outputBitmap;;
    String tipo=" ", monhu=" ", monzernikel=" ";
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        Button btnCamera = findViewById(R.id.btnCamara);
        Button btnCalHu = findViewById(R.id.btnCalHu);
        TextView txtTipo = findViewById(R.id.tipo);
        TextView lblMonHU = findViewById(R.id.lblMomentosHu);
        TextView lblMonZernikel = findViewById(R.id.lblMonZernike);
        verImg= findViewById(R.id.imgViewCamara);
        verImgOriginal=findViewById(R.id.imgOriginal);
        btnCamera.setOnClickListener(v -> {
            if (ContextCompat.checkSelfPermission(MainActivity.this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.CAMERA}, REQUEST_CAMERA_PERMISSION);
            } else {
                openCamera();
            }
        });
        btnCalHu.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                outputBitmap = imagenBitmap.copy(imagenBitmap.getConfig(), true);
                tipo=TipoFigura(imagenBitmap,outputBitmap);
                monhu=CalculoMomentosHU(outputBitmap,outputBitmap);
                monzernikel=CalculoMomentosZernike(outputBitmap,outputBitmap);
                verImg.setImageBitmap(outputBitmap);
                txtTipo.setText(tipo);
                lblMonHU.setText(monhu);
                lblMonZernikel.setText(monzernikel);
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
private native String TipoFigura(Bitmap in, Bitmap out);
private native String CalculoMomentosHU(Bitmap in, Bitmap out);
private native String CalculoMomentosZernike(Bitmap in, Bitmap out);

}