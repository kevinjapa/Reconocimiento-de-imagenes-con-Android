//
// Created by Kevin japa on 23/6/24.
//
#include <jni.h>
#include <string>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/video.hpp>
#include <android/log.h>

#include "android/bitmap.h"
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#include <opencv2/ml.hpp>

using namespace cv;
using namespace std;
using namespace cv::ml;

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

extern "C" JNIEXPORT jstring JNICALL
Java_ups_vision_practica31recfiguras_Parte2Activity_stringFromJNI2(
        JNIEnv* env,
        jobject /* this */) {
    std::string hello = "Hello from C++ in Parte2Activity";
    return env->NewStringUTF(hello.c_str());
}

//modelo importado
//
//using namespace cv::ml;
//
//#define compab_mask_inc(ptr, shift) \
//    { value |= ((unsigned int)(cntr - *ptr) & 0x80000000) >> (31 - shift); ptr++; }
//
//// Declaración de la función LBP8
//int* LBP8(const int* data, int rows, int columns);
//
//extern "C"
//JNIEXPORT jstring JNICALL
//Java_ups_vision_practica31recfiguras_Parte2Activity_Modelo(JNIEnv* env,
//                   jobject /*this*/,
//                   jobject bitmapIn,
//                   jobject assetManager) {
//
//    Mat src;
//    bitmapToMat(env, bitmapIn, src, false);
//
//    // Convertir a escala de grises si es necesario
//    if (src.channels() == 3) {
//        cvtColor(src, src, COLOR_BGR2GRAY);
//    }
//
//    // Obtener el asset manager
//    AAssetManager* mgr = AAssetManager_fromJava(env, assetManager);
//    if (mgr == nullptr) {
//        return env->NewStringUTF("Error al obtener AssetManager.");
//    }
//
//    // Leer el archivo XML del modelo desde assets
//    AAsset* asset = AAssetManager_open(mgr, "modelo_svm.xml", AASSET_MODE_BUFFER);
//    if (asset == nullptr) {
//        return env->NewStringUTF("Error al abrir el archivo modelo_svm.xml.");
//    }
//    size_t length = AAsset_getLength(asset);
//    char* buffer = new char[length + 1];
//    AAsset_read(asset, buffer, length);
//    buffer[length] = '\0';
//    AAsset_close(asset);
//
//    // Crear un archivo temporal para guardar el modelo SVM
//    std::string tempModelPath = "/sdcard/modelo_svm_temp.xml";
//    FILE* tempFile = fopen(tempModelPath.c_str(), "w");
//    if (tempFile == nullptr) {
//        delete[] buffer;
//        return env->NewStringUTF("Error al crear el archivo temporal.");
//    }
//    fwrite(buffer, sizeof(char), length, tempFile);
//    fclose(tempFile);
//    delete[] buffer;
//
//    // Cargar el modelo SVM desde el archivo XML temporal
//    Ptr<SVM> svm = SVM::load(tempModelPath);
//    if (svm.empty()) {
//        return env->NewStringUTF("Error al cargar el modelo SVM.");
//    }
//
//    int* datos = (int*)malloc(src.rows * src.cols * sizeof(int));
//    if (datos == nullptr) {
//        return env->NewStringUTF("Error al asignar memoria para datos.");
//    }
//
//    for (int i = 0, k = 0; i < src.rows; i++) {
//        for (int j = 0; j < src.cols; j++) {
//            datos[k++] = src.at<uchar>(i, j);
//        }
//    }
//
//    int* res = LBP8(datos, src.rows, src.cols);
//    if (res == nullptr) {
//        free(datos);
//        return env->NewStringUTF("Error al calcular LBP.");
//    }
//
//    vector<int> histo(256, 0);
//    for (int i = 0; i < 256; i++) {
//        histo[i] = res[i];
//    }
//
//    free(datos);
//    free(res);
//
//    // Convertir el histograma a formato OpenCV (Mat)
//    Mat histogramaEntradaMat(1, 256, CV_32FC1);
//    for (int j = 0; j < 256; ++j) {
//        histogramaEntradaMat.at<float>(0, j) = static_cast<float>(histo[j]);
//    }
//
//    // Clasificar la imagen de entrada usando el modelo SVM
//    float resultado = svm->predict(histogramaEntradaMat);
//
//    std::string resultadoStr;
//    if (resultado == 0) {
//        resultadoStr = "La imagen de entrada pertenece a la clase 1 (rocas).";
//    } else if (resultado == 1) {
//        resultadoStr = "La imagen de entrada pertenece a la clase 2 (tejidos).";
//    } else {
//        resultadoStr = "Clasificación desconocida.";
//    }
//    resultadoStr=resultadoStr + "\n Descriptor LBP: " ;
//    return env->NewStringUTF(resultadoStr.c_str());
//}
//
//// Definición de la función LBP8
//int* LBP8(const int* data, int rows, int columns) {
//    if (data == nullptr || rows < 3 || columns < 3) {
//        return nullptr;
//    }
//
//    const int
//            *p0 = data,
//            *p1 = p0 + 1,
//            *p2 = p1 + 1,
//            *p3 = p2 + columns,
//            *p4 = p3 + columns,
//            *p5 = p4 - 1,
//            *p6 = p5 - 1,
//            *p7 = p6 - columns,
//            *center = p7 + 1;
//    int r, c, cntr;
//    unsigned int value;
//    int* result = (int*)malloc(256 * sizeof(int));
//    if (result == nullptr) {
//        return nullptr;
//    }
//    memset(result, 0, 256 * sizeof(int));
//    for (r = 0; r < rows - 2; r++) {
//        for (c = 0; c < columns - 2; c++) {
//            value = 0;
//            cntr = *center - 1;
//            compab_mask_inc(p0, 0);
//            compab_mask_inc(p1, 1);
//            compab_mask_inc(p2, 2);
//            compab_mask_inc(p3, 3);
//            compab_mask_inc(p4, 4);
//            compab_mask_inc(p5, 5);
//            compab_mask_inc(p6, 6);
//            compab_mask_inc(p7, 7);
//            center++;
//            result[value]++;
//        }
//        p0 += 2;
//        p1 += 2;
//        p2 += 2;
//        p3 += 2;
//        p4 += 2;
//        p5 += 2;
//        p6 += 2;
//        p7 += 2;
//        center += 2;
//    }
//    return result;
//}



using namespace std;
using namespace cv;
using namespace cv::ml;

#define compab_mask_inc(ptr, shift) \
    { value |= ((unsigned int)(cntr - *ptr) & 0x80000000) >> (31 - shift); ptr++; }

// Declaraciones de funciones
vector<float> calcularHistogramaLBP(const Mat& imagen);
int* LBP8(const int* data, int rows, int columns);

extern "C"
JNIEXPORT jstring JNICALL
Java_ups_vision_practica31recfiguras_Parte2Activity_Modelo(JNIEnv* env, jobject /*this*/, jobject bitmapIn, jobject assetManager) {
    Mat src;
    bitmapToMat(env, bitmapIn, src, false);

    // Convertir a escala de grises si es necesario
    if (src.channels() == 3) {
        cvtColor(src, src, COLOR_BGR2GRAY);
    }

    // Obtener el asset manager
    AAssetManager* mgr = AAssetManager_fromJava(env, assetManager);
    if (mgr == nullptr) {
        return env->NewStringUTF("Error al obtener AssetManager.");
    }

    // Leer el archivo XML del modelo desde assets
    AAsset* asset = AAssetManager_open(mgr, "modelo_svm.xml", AASSET_MODE_BUFFER);
    if (asset == nullptr) {
        return env->NewStringUTF("Error al abrir el archivo modelo_svm.xml.");
    }
    size_t length = AAsset_getLength(asset);
    char* buffer = new char[length + 1];
    AAsset_read(asset, buffer, length);
    buffer[length] = '\0';
    AAsset_close(asset);

    // Crear un archivo temporal para guardar el modelo SVM
    std::string tempModelPath = "/sdcard/modelo_svm_temp.xml";
    FILE* tempFile = fopen(tempModelPath.c_str(), "w");
    if (tempFile == nullptr) {
        delete[] buffer;
        return env->NewStringUTF("Error al crear el archivo temporal.");
    }
    fwrite(buffer, sizeof(char), length, tempFile);
    fclose(tempFile);
    delete[] buffer;

    // Cargar el modelo SVM desde el archivo XML temporal
    Ptr<SVM> svm = SVM::load(tempModelPath);
    if (svm.empty()) {
        return env->NewStringUTF("Error al cargar el modelo SVM.");
    }

    // Calcular el histograma LBP de la imagen de entrada
    vector<float> histo = calcularHistogramaLBP(src);

    // Convertir el histograma a formato OpenCV (Mat)
    Mat histogramaEntradaMat(1, 256, CV_32FC1);
    for (int j = 0; j < 256; ++j) {
        histogramaEntradaMat.at<float>(0, j) = histo[j];
    }

    // Normalizar el histograma de entrada
    normalize(histogramaEntradaMat, histogramaEntradaMat, 1, 0, NORM_L1);

    // Clasificar la imagen de entrada usando el modelo SVM
    float resultado = svm->predict(histogramaEntradaMat);

    // Crear el resultado de la clasificación como cadena
    std::string resultadoStr;
    if (resultado == 0) {
        resultadoStr = "La imagen de entrada pertenece a la clase 1 (rocas).";
    } else if (resultado == 1) {
        resultadoStr = "La imagen de entrada pertenece a la clase 2 (seda).";
    } else {
        resultadoStr = "Clasificación desconocida.";
    }

    return env->NewStringUTF(resultadoStr.c_str());
}

vector<float> calcularHistogramaLBP(const Mat& imagen) {
    // Implementación de LBP8 para generar el histograma LBP
    vector<int> datos(imagen.rows * imagen.cols);
    for (int i = 0, k = 0; i < imagen.rows; i++) {
        for (int j = 0; j < imagen.cols; j++) {
            datos[k++] = imagen.at<uchar>(i, j);
        }
    }

    unique_ptr<int[]> res(LBP8(datos.data(), imagen.rows, imagen.cols));
    vector<float> histo(256, 0.0f);
    for (int i = 0; i < 256; i++) {
        histo[i] = static_cast<float>(res[i]);
    }

    return histo;
}

int* LBP8(const int* data, int rows, int columns) {
    const int
            *p0 = data,
            *p1 = p0 + 1,
            *p2 = p1 + 1,
            *p3 = p2 + columns,
            *p4 = p3 + columns,
            *p5 = p4 - 1,
            *p6 = p5 - 1,
            *p7 = p6 - columns,
            *center = p7 + 1;
    int r, c, cntr;
    unsigned int value;
    int* result = (int*)malloc(256 * sizeof(int));
    memset(result, 0, 256 * sizeof(int));
    for (r = 0; r < rows - 2; r++) {
        for (c = 0; c < columns - 2; c++) {
            value = 0;
            cntr = *center - 1;
            compab_mask_inc(p0, 0);
            compab_mask_inc(p1, 1);
            compab_mask_inc(p2, 2);
            compab_mask_inc(p3, 3);
            compab_mask_inc(p4, 4);
            compab_mask_inc(p5, 5);
            compab_mask_inc(p6, 6);
            compab_mask_inc(p7, 7);
            center++;
            result[value]++;
        }
        p0 += 2;
        p1 += 2;
        p2 += 2;
        p3 += 2;
        p4 += 2;
        p5 += 2;
        p6 += 2;
        p7 += 2;
        center += 2;
    }
    return result;
}
