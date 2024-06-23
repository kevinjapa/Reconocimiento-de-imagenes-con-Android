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

void matToBitmapString(JNIEnv* env, cv::Mat src, jobject bitmap, jboolean needPremultiplyAlpha, std::string& result) {
    AndroidBitmapInfo info;
    void* pixels = nullptr;
    try {
        CV_Assert(AndroidBitmap_getInfo(env, bitmap, &info) >= 0);
        CV_Assert(info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 || info.format == ANDROID_BITMAP_FORMAT_RGB_565);
        CV_Assert(src.dims == 2 && info.height == (uint32_t)src.rows && info.width == (uint32_t)src.cols);
        CV_Assert(src.type() == CV_8UC1 || src.type() == CV_8UC3 || src.type() == CV_8UC4);
        CV_Assert(AndroidBitmap_lockPixels(env, bitmap, &pixels) >= 0);
        CV_Assert(pixels);

        if (info.format == ANDROID_BITMAP_FORMAT_RGBA_8888) {
            cv::Mat tmp(info.height, info.width, CV_8UC4, pixels);
            if (src.type() == CV_8UC1) {
                cv::cvtColor(src, tmp, cv::COLOR_GRAY2RGBA);
            } else if (src.type() == CV_8UC3) {
                cv::cvtColor(src, tmp, cv::COLOR_RGB2RGBA);
            } else if (src.type() == CV_8UC4) {
                if (needPremultiplyAlpha) {
                    cv::cvtColor(src, tmp, cv::COLOR_RGBA2mRGBA);
                } else {
                    src.copyTo(tmp);
                }
            }
        } else {
            cv::Mat tmp(info.height, info.width, CV_8UC2, pixels);
            if (src.type() == CV_8UC1) {
                cv::cvtColor(src, tmp, cv::COLOR_GRAY2BGR565);
            } else if (src.type() == CV_8UC3) {
                cv::cvtColor(src, tmp, cv::COLOR_RGB2BGR565);
            } else if (src.type() == CV_8UC4) {
                cv::cvtColor(src, tmp, cv::COLOR_RGBA2BGR565);
            }
        }

        AndroidBitmap_unlockPixels(env, bitmap);
        result = "Conversion successful";
        return;
    } catch (const cv::Exception& e) {
        AndroidBitmap_unlockPixels(env, bitmap);
        jclass je = env->FindClass("java/lang/Exception");
        env->ThrowNew(je, e.what());
        result = "cv::Exception: " + std::string(e.what());
        return;
    } catch (...) {
        AndroidBitmap_unlockPixels(env, bitmap);
        jclass je = env->FindClass("java/lang/Exception");
        env->ThrowNew(je, "Unknown exception in JNI code {matToBitmap}");
        result = "Unknown exception in JNI code {matToBitmap}";
        return;
    }
}


extern "C"
JNIEXPORT jstring JNICALL
Java_ups_vision_practica31recfiguras_MainActivity_TipoFigura
        (JNIEnv* env,
         jobject /*this*/,
         jobject bitmapIn,
         jobject bitmapOut) {
    Mat src, filtro;
    int mascara = 5;
    bitmapToMat(env, bitmapIn, src, false);

    Mat gray, edges, imagen;
    String tipo;
    vector<Point> approx;

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

    //
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
    //
    // Recorrer cada contorno
    for (size_t i = 0; i < contours.size(); i++) {
        // Filtrar contornos pequeños
        if (contourArea(contours[i]) < 100)
            continue;

        // Aproximar el contorno

        approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.04, true);

        // Rellenar el contorno con blanco en la imagen resultante
        drawContours(result, vector<vector<Point>>{approx}, -1, Scalar(255), FILLED, LINE_8);

        // Identificar la forma en base al número de vértices del contorno aproximado

        if (approx.size() == 3) {
            // Verificar si los ángulos son aproximadamente iguales
            vector<double> angles;
            for (int j = 0; j < 3; j++) {
                Point pt1 = approx[j];
                Point pt2 = approx[(j + 1) % 3];
                Point pt3 = approx[(j + 2) % 3];

                double angle = abs(atan2(pt3.y - pt2.y, pt3.x - pt2.x) - atan2(pt1.y - pt2.y, pt1.x - pt2.x)) * 180 / CV_PI;
                if (angle > 180)
                    angle = 360 - angle;
                angles.push_back(angle);
            }
            bool isTriangle = true;
            for (double angle : angles) {
                if (angle < 20 || angle > 160) {
                    isTriangle = false;
                    break;
                }
            }
            if (isTriangle) {
                tipo = "Triangulo";
            }
        } else if (approx.size() == 4) {
            // Verificar si es un cuadrado o un rectángulo
            Rect boundingBox = boundingRect(approx);
            float aspectRatio = (float)boundingBox.width / (float)boundingBox.height;
            if (aspectRatio >= 0.8 && aspectRatio <= 1.2) {
                tipo = "Cuadrado";
            } else {
                tipo = "Rectangulo";
            }
        } else {
            // Usar el área y el perímetro para determinar si es un círculo
            double area = contourArea(contours[i]);
            double perimeter = arcLength(contours[i], true);
            double circularity = 4 * CV_PI * (area / (perimeter * perimeter));
            if (circularity > 0.7) {
                tipo = "Circulo";
            }
              else {
                 tipo = "Desconocido";
             }
        }
    }
//    putText(edges, shape, approx[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
    matToBitmap(env, result2, bitmapOut, false);

    string hello = "Tipo de Figura: "+tipo;
    return env->NewStringUTF(hello.c_str());
}


//double momentosHuBase[7] = {0.278654,0.00471249,0.000578773,0.00138675,1.16567e-06,2.2131e-05,4.29781e-07};
//double distanciaEuclidea(double momentosHu[7]){
//    double suma = 0;
//    for(int i=0;i<7;i++){
//        suma+=((momentosHuBase[i]-momentosHu[i])*(momentosHuBase[i]-momentosHu[i]));
//    }
//    return sqrt(suma);
//}
//
//extern "C"
//JNIEXPORT jstring JNICALL
//Java_ups_vision_practica31recfiguras_MainActivity_CalculoMomentosHU
//                (JNIEnv* env,
//                 jobject /*this*/,
//                 jobject bitmapIn,
//                 jobject bitmapOut) {
//    Mat src, filtro;
//    bitmapToMat(env, bitmapIn, src, false);
//    Mat frame = src;
//    String tipo;
//    Mat imgHSV, binaria;
//    double Cx = 0, Cy = 0;
//    double distancia = 0;
//
//    Moments momentos;
//    double momentosHu[7];
//
//    // Convertir la imagen de BGR a HSV
//    cvtColor(frame, imgHSV, COLOR_BGR2HSV);
//
//    // Calcular los momentos
//    momentos = moments(binaria, true);
//    Cx = momentos.m10 / momentos.m00;
//    Cy = momentos.m01 / momentos.m00;
//
//    if(momentos.m00 > 100){
////        cout << "Cx: " << Cx << " Cy: " << Cy << " área: " << momentos.m00 << endl;
//
//        HuMoments(momentos, momentosHu);
//        distancia = distanciaEuclidea(momentosHu);
////        cout << "Distancia: " << distancia << endl;
//
//        if(distancia < 0.11){
//            putText(frame, "cuadrado", Point(Cx, Cy), FONT_HERSHEY_SIMPLEX, 1, Scalar(233, 1, 1), 2);
//            tipo="cuadrado";
//        }
//    }
//    matToBitmap(env, frame, bitmapOut, false);
//    //String salida=tipo+"Distancia: "+distancia+"Area: "+ momentos.m00 ;
//    std::string salida = tipo + " Distancia: " + std::to_string(distancia) + " Area: " + std::to_string(momentos.m00);
//
//
//    std::string hello = salida;
//    return env->NewStringUTF(hello.c_str());
//}


// Valores de los momentos de Hu promedio para cada categoría
double momentosHuBaseTriangle[7] = {0.05797532, 0.00445132, 0.01587491, 0.0040534, 0.00141166, 0.00132272, -0.00013352};
double momentosHuBaseSquare[7] = {0.0774586042, 0.0210825573, 0.0043398799, 0.00310416062, 0.000323355775, 0.00156343498, -0.0000849751092};
double momentosHuBaseCircle[7] = {0.05911688, 0.01537497, 0.00772469, 0.00786365, 0.00456395, 0.00742623, 0.00020278};

// Umbrales para las áreas de cada figura
double areaThresholdTriangle[2] = {13000, 18000};  // Intervalo de área típico para triángulos
double areaThresholdSquare[2] = {9000, 14000};     // Intervalo de área típico para cuadrados
double areaThresholdCircle[2] = {7000, 15000};     // Intervalo de área típico para círculos

double distanciaEuclidea(double momentosHu[7], double momentosHuBase[7]) {
    double suma = 0;
    for (int i = 0; i < 7; i++) {
        suma += ((momentosHuBase[i] - momentosHu[i]) * (momentosHuBase[i] - momentosHu[i]));
    }
    return sqrt(suma);
}

string determinarFigura(double momentosHu[7], double area) {
    double distTriangle = distanciaEuclidea(momentosHu, momentosHuBaseTriangle);
    double distSquare = distanciaEuclidea(momentosHu, momentosHuBaseSquare);
    double distCircle = distanciaEuclidea(momentosHu, momentosHuBaseCircle);

    if (distTriangle < distSquare && distTriangle < distCircle && area >= areaThresholdTriangle[0] && area <= areaThresholdTriangle[1]) {
        return "triángulo";
    } else if (distSquare < distTriangle && distSquare < distCircle && area >= areaThresholdSquare[0] && area <= areaThresholdSquare[1]) {
        return "cuadrado";
    } else if (distCircle < distTriangle && distCircle < distSquare && area >= areaThresholdCircle[0] && area <= areaThresholdCircle[1]) {
        return "círculo";
    } else {
        return "indeterminado";  // Si no coincide con ninguno de los umbrales
    }
}

extern "C"
JNIEXPORT jstring JNICALL
Java_ups_vision_practica31recfiguras_MainActivity_CalculoMomentosHU
        (JNIEnv* env,
         jobject /*this*/,
         jobject bitmapIn,
         jobject bitmapOut) {
    Mat src, filtro;
    bitmapToMat(env, bitmapIn, src, false);
    Mat frame = src;
    string tipo;
    Mat imgHSV, binaria;
    double Cx = 0, Cy = 0;
    double distancia = 0;

    Moments momentos;
    double momentosHu[7];

    // Convertir la imagen de BGR a HSV
    cvtColor(frame, imgHSV, COLOR_BGR2HSV);

    // Binarización (por ejemplo, umbralización simple)
    inRange(imgHSV, Scalar(0, 0, 0), Scalar(180, 255, 30), binaria);

    // Calcular los momentos
    momentos = moments(binaria, true);
    Cx = momentos.m10 / momentos.m00;
    Cy = momentos.m01 / momentos.m00;

    if (momentos.m00 > 100) {
        // Calcular los momentos de Hu
        HuMoments(momentos, momentosHu);

        // Determinar el tipo de figura basado en los momentos de Hu y el área
        tipo = determinarFigura(momentosHu, momentos.m00);

        // Poner el texto en la imagen
        putText(frame, tipo, Point(Cx, Cy), FONT_HERSHEY_SIMPLEX, 1, Scalar(233, 1, 1), 2);
    }

    // Convertir la imagen procesada de nuevo a bitmap
//    matToBitmap(env, frame, bitmapOut, false);

    // Crear la cadena de salida con la información de la figura
    string salida ="Momentos de HU\nM1: "+to_string(momentosHu[0])+"\nM2: "+to_string(momentosHu[1]) +"\nM3: "+to_string(momentosHu[2]) +"\nM4: "+to_string(momentosHu[3])+"\nM5: "+to_string(momentosHu[4]) +"\nM6: "+to_string(momentosHu[5]) +"\nM7: "+to_string(momentosHu[6]);
    return env->NewStringUTF(salida.c_str());
}


double factorial(int n) {
    if (n == 0 || n == 1)
        return 1.0;
    double result = 1.0;
    for (int i = 2; i <= n; ++i)
        result *= i;
    return result;
}

// Función para calcular el polinomio radial de Zernike
double radialPolynomial(int n, int m, double r) {
    double radial = 0.0;
    for (int s = 0; s <= (n - abs(m)) / 2; s++) {
        double c = pow(-1, s) * factorial(n - s) /
                   (factorial(s) * factorial((n + abs(m)) / 2 - s) * factorial((n - abs(m)) / 2 - s));
        radial += c * pow(r, n - 2 * s);
    }
    return radial;
}

// Función para calcular los momentos de Zernike
complex<double> zernikeMoment(const Mat& img, int n, int m) {
    double cx = img.cols / 2.0;
    double cy = img.rows / 2.0;
    double radius = min(cx, cy);

    complex<double> moment(0.0, 0.0);
    for (int y = 0; y < img.rows; y++) {
        for (int x = 0; x < img.cols; x++) {
            double dx = (x - cx) / radius;
            double dy = (cy - y) / radius;
            double r = sqrt(dx * dx + dy * dy);
            if (r > 1.0)
                continue;
            double theta = atan2(dy, dx);
            double radial = radialPolynomial(n, m, r);
            complex<double> zernike = radial * exp(complex<double>(0.0, -m * theta));
            moment += static_cast<double>(img.at<uchar>(y, x)) * zernike; // Convertir a double
        }
    }

    double norm_factor = (n + 1) / CV_PI;
    return moment * norm_factor;
}

extern "C"
JNIEXPORT jstring JNICALL
Java_ups_vision_practica31recfiguras_MainActivity_CalculoMomentosZernike
        (JNIEnv* env,
         jobject /*this*/,
         jobject bitmapIn,
         jobject bitmapOut) {

    Mat src, filtro;
    bitmapToMat(env, bitmapIn, src, false);

    // Binarizar la imagen
    threshold(src, src, 128, 255, THRESH_BINARY);

    // Calcular momentos de Zernike para n = 4, m = 2
    complex<double> zm = zernikeMoment(src, 4, 2);

    // Crear la cadena de salida con la información de la figura
    string salida ="Momento de Zernike (4, 2): \n "+to_string(zm.real())+" + "+to_string(zm.imag())+"i" ;
    return env->NewStringUTF(salida.c_str());
}

