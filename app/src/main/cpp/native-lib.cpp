#include <jni.h>
#include <string>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/video.hpp>
#include <android/log.h>

#include "android/bitmap.h"

using namespace cv;
using namespace std;

extern "C" JNIEXPORT jstring

JNICALL
Java_ups_vision_practica31recfiguras_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

void bitmapToMat(JNIEnv * env, jobject bitmap, cv::Mat &dst, jboolean needUnPremultiplyAlpha){
    AndroidBitmapInfo info;
    void* pixels = 0;
    try {
        CV_Assert( AndroidBitmap_getInfo(env, bitmap, &info) >= 0 );
        CV_Assert( info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 ||
                   info.format == ANDROID_BITMAP_FORMAT_RGB_565 );
        CV_Assert( AndroidBitmap_lockPixels(env, bitmap, &pixels) >= 0 );
        CV_Assert( pixels );
        dst.create(info.height, info.width, CV_8UC4);
        if( info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 )
        {
            cv::Mat tmp(info.height, info.width, CV_8UC4, pixels);
            if(needUnPremultiplyAlpha) cvtColor(tmp, dst, cv::COLOR_mRGBA2RGBA);
            else tmp.copyTo(dst);
        } else {
            // info.format == ANDROID_BITMAP_FORMAT_RGB_565
            cv::Mat tmp(info.height, info.width, CV_8UC2, pixels);
            cvtColor(tmp, dst, cv::COLOR_BGR5652RGBA);
        }
        AndroidBitmap_unlockPixels(env, bitmap);
        return;
    } catch(const cv::Exception& e) {
        AndroidBitmap_unlockPixels(env, bitmap);
        //jclass je = env->FindClass("org/opencv/core/CvException");
        jclass je = env->FindClass("java/lang/Exception");
        //if(!je) je = env->FindClass("java/lang/Exception");
        env->ThrowNew(je, e.what());
        return;
    } catch (...) {
        AndroidBitmap_unlockPixels(env, bitmap);
        jclass je = env->FindClass("java/lang/Exception");
        env->ThrowNew(je, "Unknown exception in JNI code {nBitmapToMat}");
        return;
    }
}

void matToBitmap(JNIEnv * env, cv::Mat src, jobject bitmap, jboolean needPremultiplyAlpha) {
    AndroidBitmapInfo info;
    void *pixels = 0;
    try {
        CV_Assert(AndroidBitmap_getInfo(env, bitmap, &info) >= 0);
        CV_Assert(info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 ||
                  info.format == ANDROID_BITMAP_FORMAT_RGB_565);
        CV_Assert(src.dims == 2 && info.height == (uint32_t) src.rows &&
                  info.width == (uint32_t) src.cols);
        CV_Assert(src.type() == CV_8UC1 || src.type() == CV_8UC3 || src.type() == CV_8UC4);
        CV_Assert(AndroidBitmap_lockPixels(env, bitmap, &pixels) >= 0);
        CV_Assert(pixels);
        if (info.format == ANDROID_BITMAP_FORMAT_RGBA_8888) {
            cv::Mat tmp(info.height, info.width, CV_8UC4, pixels);
            if (src.type() == CV_8UC1) {
                cvtColor(src, tmp, cv::COLOR_GRAY2RGBA);
            } else if (src.type() == CV_8UC3) {
                cvtColor(src, tmp, cv::COLOR_RGB2RGBA);
            } else if (src.type() == CV_8UC4) {
                if (needPremultiplyAlpha) cvtColor(src, tmp, cv::COLOR_RGBA2mRGBA);
                else src.copyTo(tmp);
            }
        } else {
            // info.format == ANDROID_BITMAP_FORMAT_RGB_565
            cv::Mat tmp(info.height, info.width, CV_8UC2, pixels);
            if (src.type() == CV_8UC1) {
                cvtColor(src, tmp, cv::COLOR_GRAY2BGR565);
            } else if (src.type() == CV_8UC3) {
                cvtColor(src, tmp, cv::COLOR_RGB2BGR565);
            } else if (src.type() == CV_8UC4) {
                cvtColor(src, tmp, cv::COLOR_RGBA2BGR565);
            }
        }
        AndroidBitmap_unlockPixels(env, bitmap);
        return;
    } catch (const cv::Exception &e) {
        AndroidBitmap_unlockPixels(env, bitmap);
        //jclass je = env->FindClass("org/opencv/core/CvException");
        jclass je = env->FindClass("java/lang/Exception");
        //if(!je) je = env->FindClass("java/lang/Exception");
        env->ThrowNew(je, e.what());
        return;
    } catch (...) {
        AndroidBitmap_unlockPixels(env, bitmap);
        jclass je = env->FindClass("java/lang/Exception");
        env->ThrowNew(je, "Unknown exception in JNI code {nMatToBitmap}");
        return;
    }
}


// Valores de referencia ajustados de los momentos de Hu
double momentosHuCircle[7] = {1.60625325e-01, 4.35777910e-04, 7.03658368e-06, 3.09147731e-08, -1.33794872e-14, -6.22958925e-10, 5.37512611e-15};
double momentosHuSquare[7] = {1.69139028e-01, 4.20432180e-04, 7.26247928e-05, 2.39599503e-06, -2.98203693e-11, -4.52677160e-08, -1.04734442e-11};
double momentosHuTriangle[7] = {2.21053266e-01, 1.18374855e-03, 1.01162740e-02, 1.94435975e-04, 2.68714545e-07, 4.52135269e-06, 4.64153988e-08};

double distanciaEuclidea(double momentosHu[7], double momentosHuReferencia[7]) {
    double suma = 0;
    for (int i = 0; i < 7; i++) {
        suma += ((momentosHuReferencia[i] - momentosHu[i]) * (momentosHuReferencia[i] - momentosHu[i]));
    }
    return sqrt(suma);
}
extern "C"
JNIEXPORT jstring JNICALL
Java_ups_vision_practica31recfiguras_MainActivity_MomentosHU
        (JNIEnv* env,
         jobject /*this*/,
         jobject bitmapIn,
         jobject bitmapOut){
    Mat src;
    bitmapToMat(env, bitmapIn, src, false);
    String tipo;
    Mat gray, edges, imagen;
    cvtColor(src, gray, COLOR_BGR2GRAY);

    // Aplicar un filtro gaussiano para reducir el ruido
    GaussianBlur(gray, gray, Size(5, 5), 0);

    // Detectar bordes usando Canny
    Canny(gray, edges, 50, 150, 3);

    // Binarizar la imagen
    double thresh_value = 150; // Valor del umbral
    double max_value = 255;    // Valor máximo para el umbral
    threshold(edges, edges, thresh_value, max_value, THRESH_BINARY);

    // Crear una imagen para mostrar el resultado final
    Mat result = Mat::zeros(edges.size(), CV_8UC1);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    Mat result2 = Mat::zeros(edges.size(), CV_8UC1);

    vector<vector<Point>> contours2;
    vector<Vec4i> hierarchy2;
    findContours(edges, contours2, hierarchy2, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // Rellenar el interior del contorno con color blanco
    for (size_t i = 0; i < contours2.size(); i++) {
        // Filtrar contornos pequeños
        if (contourArea(contours2[i]) < 100)
            continue;
        // Rellenar el contorno con blanco en la imagen resultante
        drawContours(result2, contours2, (int)i, Scalar(255), FILLED, LINE_8);
    }
    // Calcular momentos de Hu para la imagen binaria resultante
    Moments momentos = moments(result2, true);
    double momentosHu[7];
    HuMoments(momentos, momentosHu);

    double distanciaCircle = distanciaEuclidea(momentosHu, momentosHuCircle);
    double distanciaSquare = distanciaEuclidea(momentosHu, momentosHuSquare);
    double distanciaTriangle = distanciaEuclidea(momentosHu, momentosHuTriangle);

    // Ajustar los umbrales de decisión
    if (distanciaCircle < distanciaSquare && distanciaCircle < distanciaTriangle) {
//        putText(src, "circulo", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        tipo="Circulo";
    }
    else if (distanciaSquare < distanciaCircle && distanciaSquare < distanciaTriangle) {
//        putText(src, "cuadrado", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        tipo="Cuadrado";
    }
    else if (distanciaTriangle < distanciaCircle && distanciaTriangle < distanciaSquare) {
//        putText(src, "triangulo", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        tipo="Triangulo";
    }
    else
    {
        tipo="Desconocido";
    }

    matToBitmap(env, result2, bitmapOut, false);
    string salida ="        Tipo de Figura: "+tipo+"\n"+"\nMomentos de HU\nM1: "+to_string(momentosHu[0])+"\nM2: "+to_string(momentosHu[1]) +"\nM3: "+to_string(momentosHu[2]) +"\nM4: "+to_string(momentosHu[3])+"\nM5: "+to_string(momentosHu[4]) +"\nM6: "+to_string(momentosHu[5]) +"\nM7: "+to_string(momentosHu[6]);
    return env->NewStringUTF(salida.c_str());
}



//Zernike

// Función para calcular los momentos de Zernike
void calculateZernikeMoments(Mat& image, vector<double>& zernikeMoments) {
    // Implementa la función de cálculo de momentos de Zernike aquí
    // Esta función requiere una implementación específica o una biblioteca de terceros en C++
    // Aquí, se usa un lugar reservado para ilustrar
    zernikeMoments.resize(36); // Suponiendo que estamos calculando 36 momentos de Zernike
    for (int i = 0; i < 36; ++i) {
        zernikeMoments[i] = rand() / double(RAND_MAX); // Valores aleatorios de ejemplo
    }
}

double distanciaEuclidea(const vector<double>& moments1, const vector<double>& moments2) {
    double suma = 0;
    for (size_t i = 0; i < moments1.size(); ++i) {
        suma += (moments1[i] - moments2[i]) * (moments1[i] - moments2[i]);
    }
    return sqrt(suma);
}

extern "C"
JNIEXPORT jstring JNICALL
Java_ups_vision_practica31recfiguras_MainActivity_MomentosZernike
        (JNIEnv* env,
         jobject /*this*/,
         jobject bitmapIn,
         jobject bitmapOut){
    Mat src;
    bitmapToMat(env, bitmapIn, src, false);
    String tipo;

    Mat gray, edges, imagen;
    cvtColor(src, gray, COLOR_BGR2GRAY);

    // Aplicar un filtro gaussiano para reducir el ruido
    GaussianBlur(gray, gray, Size(5, 5), 0);

    // Detectar bordes usando Canny
    Canny(gray, edges, 50, 150, 3);

    // Binarizar la imagen
    double thresh_value = 150; // Valor del umbral
    double max_value = 255;    // Valor máximo para el umbral
    threshold(edges, edges, thresh_value, max_value, THRESH_BINARY);

    // Crear una imagen para mostrar el resultado final
    Mat result = Mat::zeros(edges.size(), CV_8UC1);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    Mat result2 = Mat::zeros(edges.size(), CV_8UC1);

    vector<vector<Point>> contours2;
    vector<Vec4i> hierarchy2;
    findContours(edges, contours2, hierarchy2, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // Rellenar el interior del contorno con color blanco
    for (size_t i = 0; i < contours2.size(); i++) {
        // Filtrar contornos pequeños
        if (contourArea(contours2[i]) < 100)
            continue;

        // Rellenar el contorno con blanco en la imagen resultante
        drawContours(result2, contours2, (int)i, Scalar(255), FILLED, LINE_8);
    }

    // Calcular momentos de Zernike para la imagen binaria resultante
    vector<double> zernikeMoments;
    calculateZernikeMoments(result2, zernikeMoments);

    // Momentos de Zernike de referencia (valores aleatorios de ejemplo)
    vector<double> zernikeCircle(36, 0.5);
    vector<double> zernikeSquare(36, 0.3);
    vector<double> zernikeTriangle(36, 0.7);

    double distanciaCircle = distanciaEuclidea(zernikeMoments, zernikeCircle);
    double distanciaSquare = distanciaEuclidea(zernikeMoments, zernikeSquare);
    double distanciaTriangle = distanciaEuclidea(zernikeMoments, zernikeTriangle);

//    cout << "Distancia Circle: " << distanciaCircle << endl;
//    cout << "Distancia Square: " << distanciaSquare << endl;
//    cout << "Distancia Triangle: " << distanciaTriangle << endl;

    // Ajustar los umbrales de decisión
    if (distanciaCircle < distanciaSquare && distanciaCircle < distanciaTriangle) {
        putText(src, "circulo", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        tipo="Circulo";
    }
    else if (distanciaSquare < distanciaCircle && distanciaSquare < distanciaTriangle) {
//        putText(src, "cuadrado", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        tipo="Cuadrado";
    }
    else if (distanciaTriangle < distanciaCircle && distanciaTriangle < distanciaSquare) {
//        putText(src, "triangulo", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        tipo="Triangulo";
    }
    else
    {
        tipo="Desconocido";
    }

    matToBitmap(env, result2, bitmapOut, false);
    string salida ="        Tipo de Figura: "+tipo+"\n";
    return env->NewStringUTF(salida.c_str());
}