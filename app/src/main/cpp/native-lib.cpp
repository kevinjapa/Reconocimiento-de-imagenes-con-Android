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

extern "C"
JNIEXPORT void JNICALL
Java_ups_vision_practica31recfiguras_MainActivity_CalculoMomentos
        (JNIEnv* env,
         jobject /*this*/,
         jobject bitmapIn,
         jobject bitmapOut) {
    Mat src, filtro;
    int mascara = 5;
    bitmapToMat(env, bitmapIn, src, false);
//    medianBlur(src,filtro,mascara);
    Mat gray,edges;;
    cvtColor(src, gray, COLOR_BGR2GRAY);
    // Aplicar un filtro gaussiano para reducir el ruido

    double thresh_value = 150; // valor del umbral
    double max_value = 255;    // valor máximo para el umbral
    threshold(edges, edges, thresh_value, max_value, THRESH_BINARY);
    GaussianBlur(gray, gray, Size(5, 5), 0);
//
    Canny(gray, edges, 50, 150, 3);
//
    int morph_size = 2;
    Mat element = getStructuringElement(MORPH_RECT,
                                        Size(2 * morph_size + 1, 2 * morph_size + 1),
                                        Point(morph_size, morph_size));


    dilate(edges,edges,element);
//
//
    // Encontrar los contornos
    vector<vector<Point>> contours;
    findContours(edges, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // Recorrer cada contorno y determinar la figura
    for (size_t i = 0; i < contours.size(); i++) {
        // Filtrar contornos pequeños
        if (contourArea(contours[i]) < 100)
            continue;

        // Aproximar el contorno
        vector<Point> approx;
        approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.04, true);
        Mat mask = Mat::zeros(src.rows + 2, src.cols + 2, CV_8UC1);
        // Asegurarse de que el contorno sea convexo
        if (!isContourConvex(approx))
            continue;

        // Dibujar los contornos
        drawContours(edges, contours, (int) i, Scalar(0, 255, 0), 2, LINE_8);

        // Rellenar la figura usando floodFill

        Point seed = contours[i][0];
        floodFill(edges, mask, seed, Scalar(0, 0, 255), 0, Scalar(10, 10, 10), Scalar(10, 10, 10));
        // Determinar la figura según el número de vértices
        if (approx.size() == 3) {
            putText(src, "Triangulo", approx[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
        } else if (approx.size() == 4) {
            // Verificar si es un cuadrado o un rectángulo
            Rect boundingBox = boundingRect(contours[i]);
            float aspectRatio = (float) boundingBox.width / (float) boundingBox.height;
            if (aspectRatio >= 0.9 && aspectRatio <= 1.1) {
                putText(edges, "Cuadrado", approx[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0),
                        2);
                 } else {
                     putText(edges, "Rectangulo", approx[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
                 }
            } else if (approx.size() > 4) {
                // Asumir que es un círculo si tiene muchos vértices
                putText(edges, "Circulo", approx[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
            }
        else {
            putText(edges, "No se reconoce", approx[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
        }
        }
        matToBitmap(env, edges, bitmapOut, false);
    }
