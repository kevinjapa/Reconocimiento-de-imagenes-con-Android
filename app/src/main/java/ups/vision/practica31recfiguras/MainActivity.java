package ups.vision.practica31recfiguras;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.net.Uri;
import android.os.Bundle;
import android.provider.MediaStore;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.TextView;
import android.widget.Toast;

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
    private Bitmap imagenBitmap, outputBitmap;
    String tipo=" ";
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        Button btnCamera = findViewById(R.id.btnCamara);
        Button btnCalHu = findViewById(R.id.btnCalHu);
        Button btnTextura= findViewById(R.id.btnTexturas);
        TextView txtTipo = findViewById(R.id.tipo);
        verImg= findViewById(R.id.imgViewCamara);
        verImgOriginal=findViewById(R.id.imgOriginal);
        RadioGroup grupo0 =findViewById(R.id.metodos);
        RadioButton opcionHu=findViewById(R.id.radio_hu);
        RadioButton opcionZernike=findViewById(R.id.radio_zernike);
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
                if (opcionZernike.isChecked()){
                    tipo=MomentosZernike(imagenBitmap,outputBitmap);
                    verImg.setImageBitmap(outputBitmap);
                    txtTipo.setText(tipo);
                }
                else if(opcionHu.isChecked()){
                    tipo=MomentosHU(imagenBitmap,outputBitmap);
                    verImg.setImageBitmap(outputBitmap);
                    txtTipo.setText(tipo);
                }
            }
        });

        btnTextura.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intentTexturas = new Intent(MainActivity.this, Parte2Activity.class);
                startActivity(intentTexturas);
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
    private native String MomentosHU(Bitmap in, Bitmap out);
    private native String MomentosZernike(Bitmap in, Bitmap out);

}