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
Java_ups_vision_practica31recfiguras_MainActivity_CalculoMomentos
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

    std::string hello = tipo;
    return env->NewStringUTF(hello.c_str());
}

extern "C"
JNIEXPORT jstring JNICALL
Java_ups_vision_practica31recfiguras_MainActivity_CalculoMomentosHU
                (JNIEnv* env,
                 jobject /*this*/,
                 jobject bitmapIn,
                 jobject bitmapOut) {
    double momentosHuBase[7] = {0.278654,0.00471249,0.000578773,0.00138675,1.16567e-06,2.2131e-05,4.29781e-07};
    double distanciaEuclidea(double momentosHu[7]){
        double suma = 0;
        for(int i=0;i<7;i++){
            suma+=((momentosHuBase[i]-momentosHu[i])*(momentosHuBase[i]-momentosHu[i]));
        }
        return sqrt(suma);
    }
    std::string hello = "tipo";
    return env->NewStringUTF(hello.c_str());
}
