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

//    private static final int REQUEST_PERMISSION_CAMERA = 101;
//    private static final int REQUEST_IMAGE_CAMERA = 101;
//    private static final int REQUEST_PERMISSION_WRITE_EXTERNAL_STORAGE = 2;

    private static final int REQUEST_IMAGE_CAPTURE = 1;
    private static final int REQUEST_CAMERA_PERMISSION = 100;
    private ImageView verImg;
    private Uri photoURI;
    private ActivityMainBinding binding;
    private Bitmap imagenBitmap, outputBitmap;;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        Button btnCamera = findViewById(R.id.btnCamara);
        Button btnCalHu = findViewById(R.id.btnCalHu);
        verImg= findViewById(R.id.imgViewCamara);
//        btnCamera.setOnClickListener(new View.OnClickListener() {
//            @Override
//            public void onClick(View v) {
//                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
//                    if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED &&
//                            ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.WRITE_EXTERNAL_STORAGE) == PackageManager.PERMISSION_GRANTED) {
//                        goToCamera();
//                    } else {
//                        ActivityCompat.requestPermissions(MainActivity.this,
//                                new String[]{Manifest.permission.CAMERA, Manifest.permission.WRITE_EXTERNAL_STORAGE},
//                                REQUEST_PERMISSION_CAMERA);
//                    }
//                } else {
//                    goToCamera();
//                }
//            }
//        });
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
                CalculoMomentos(imagenBitmap,imagenBitmap);
                verImg.setImageBitmap(outputBitmap);
            }
        });

        // Example of a call to a native method
        TextView tv = binding.sampleText;
        tv.setText(stringFromJNI());
    }
    @Override
    protected void onActivityResult(int requestCode, int resultCode, @Nullable Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == REQUEST_IMAGE_CAPTURE && resultCode == RESULT_OK) {
            Bundle extras = data.getExtras();
            imagenBitmap = (Bitmap) extras.get("data");
            verImg.setImageBitmap(imagenBitmap);
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

//    private void goToCamera() {
//        Intent cameraIntent = new Intent(MediaStore.ACTION_IMAGE_CAPTURE);
//        if (cameraIntent.resolveActivity(getPackageManager()) != null) {
//            File photoFile = null;
//            try {
//                photoFile = createImageFile();
//            } catch (IOException ex) {
//                // Error occurred while creating the File
//                ex.printStackTrace();
//            }
//            if (photoFile != null) {
//                photoURI = FileProvider.getUriForFile(this,
//                        "com.yourdomain.yourapp.fileprovider",
//                        photoFile);
//                cameraIntent.putExtra(MediaStore.EXTRA_OUTPUT, photoURI);
//                startActivityForResult(cameraIntent, REQUEST_IMAGE_CAMERA);
//            }
//        }
//    }

//    @Override
//    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
//        if(requestCode == REQUEST_PERMISSION_CAMERA){
//            if(permissions.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED){
//                goToCamera();
//            }else{
//                Toast.makeText(this, "You need to enable permissions", Toast.LENGTH_SHORT).show();
//            }
//        }
//        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
//    }
//
//    @Override
//    protected void onActivityResult(int requestCode, int resultCode, @Nullable Intent data) {
//        if (requestCode == REQUEST_IMAGE_CAMERA && resultCode == Activity.RESULT_OK) {
//            try {
//                imagenBitmap = MediaStore.Images.Media.getBitmap(this.getContentResolver(), photoURI);
//                Matrix matrix = new Matrix();
//                matrix.postRotate(90); // Rotar 90 grados
//
//                imagenBitmap = Bitmap.createBitmap(imagenBitmap, 0, 0, imagenBitmap.getWidth(), imagenBitmap.getHeight(), matrix, true);
//
//                verImg.setImageBitmap(imagenBitmap);
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//        }
//        super.onActivityResult(requestCode, resultCode, data);
//    }
public native String stringFromJNI();
//    private File createImageFile() throws IOException {
//        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
//        String imageFileName = "JPEG_" + timeStamp + "_";
//        File storageDir = getExternalFilesDir(Environment.DIRECTORY_PICTURES);
//        File image = File.createTempFile(
//                imageFileName,
//                ".jpg",
//                storageDir
//        );
//        return image;
//    }

    private void openCamera() {
        Intent takePictureIntent = new Intent(MediaStore.ACTION_IMAGE_CAPTURE);
        if (takePictureIntent.resolveActivity(getPackageManager()) != null) {
            startActivityForResult(takePictureIntent, REQUEST_IMAGE_CAPTURE);
        }
    }
    private native void CalculoMomentos(android.graphics.Bitmap in, android.graphics.Bitmap out);
}