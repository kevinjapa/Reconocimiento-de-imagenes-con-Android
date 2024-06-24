//
// Created by Kevin japa on 23/6/24.
//
#include <jni.h>
#include <string>
#include <sstream>
//#include <opencv2/core.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/dnn.hpp>
//#include <opencv2/video.hpp>
//#include <android/log.h>
//
//#include "android/bitmap.h"

//using namespace cv;
using namespace std;

//void bitmapToMat(JNIEnv * env, jobject bitmap, cv::Mat &dst, jboolean needUnPremultiplyAlpha){
//    AndroidBitmapInfo info;
//    void* pixels = 0;
//    try {
//        CV_Assert( AndroidBitmap_getInfo(env, bitmap, &info) >= 0 );
//        CV_Assert( info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 ||
//                   info.format == ANDROID_BITMAP_FORMAT_RGB_565 );
//        CV_Assert( AndroidBitmap_lockPixels(env, bitmap, &pixels) >= 0 );
//        CV_Assert( pixels );
//        dst.create(info.height, info.width, CV_8UC4);
//        if( info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 )
//        {
//            cv::Mat tmp(info.height, info.width, CV_8UC4, pixels);
//            if(needUnPremultiplyAlpha) cvtColor(tmp, dst, cv::COLOR_mRGBA2RGBA);
//            else tmp.copyTo(dst);
//        } else {
//            // info.format == ANDROID_BITMAP_FORMAT_RGB_565
//            cv::Mat tmp(info.height, info.width, CV_8UC2, pixels);
//            cvtColor(tmp, dst, cv::COLOR_BGR5652RGBA);
//        }
//        AndroidBitmap_unlockPixels(env, bitmap);
//        return;
//    } catch(const cv::Exception& e) {
//        AndroidBitmap_unlockPixels(env, bitmap);
//        //jclass je = env->FindClass("org/opencv/core/CvException");
//        jclass je = env->FindClass("java/lang/Exception");
//        //if(!je) je = env->FindClass("java/lang/Exception");
//        env->ThrowNew(je, e.what());
//        return;
//    } catch (...) {
//        AndroidBitmap_unlockPixels(env, bitmap);
//        jclass je = env->FindClass("java/lang/Exception");
//        env->ThrowNew(je, "Unknown exception in JNI code {nBitmapToMat}");
//        return;
//    }
//}
//
//void matToBitmap(JNIEnv * env, cv::Mat src, jobject bitmap, jboolean needPremultiplyAlpha) {
//    AndroidBitmapInfo info;
//    void *pixels = 0;
//    try {
//        CV_Assert(AndroidBitmap_getInfo(env, bitmap, &info) >= 0);
//        CV_Assert(info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 ||
//                  info.format == ANDROID_BITMAP_FORMAT_RGB_565);
//        CV_Assert(src.dims == 2 && info.height == (uint32_t) src.rows &&
//                  info.width == (uint32_t) src.cols);
//        CV_Assert(src.type() == CV_8UC1 || src.type() == CV_8UC3 || src.type() == CV_8UC4);
//        CV_Assert(AndroidBitmap_lockPixels(env, bitmap, &pixels) >= 0);
//        CV_Assert(pixels);
//        if (info.format == ANDROID_BITMAP_FORMAT_RGBA_8888) {
//            cv::Mat tmp(info.height, info.width, CV_8UC4, pixels);
//            if (src.type() == CV_8UC1) {
//                cvtColor(src, tmp, cv::COLOR_GRAY2RGBA);
//            } else if (src.type() == CV_8UC3) {
//                cvtColor(src, tmp, cv::COLOR_RGB2RGBA);
//            } else if (src.type() == CV_8UC4) {
//                if (needPremultiplyAlpha) cvtColor(src, tmp, cv::COLOR_RGBA2mRGBA);
//                else src.copyTo(tmp);
//            }
//        } else {
//            // info.format == ANDROID_BITMAP_FORMAT_RGB_565
//            cv::Mat tmp(info.height, info.width, CV_8UC2, pixels);
//            if (src.type() == CV_8UC1) {
//                cvtColor(src, tmp, cv::COLOR_GRAY2BGR565);
//            } else if (src.type() == CV_8UC3) {
//                cvtColor(src, tmp, cv::COLOR_RGB2BGR565);
//            } else if (src.type() == CV_8UC4) {
//                cvtColor(src, tmp, cv::COLOR_RGBA2BGR565);
//            }
//        }
//        AndroidBitmap_unlockPixels(env, bitmap);
//        return;
//    } catch (const cv::Exception &e) {
//        AndroidBitmap_unlockPixels(env, bitmap);
//        //jclass je = env->FindClass("org/opencv/core/CvException");
//        jclass je = env->FindClass("java/lang/Exception");
//        //if(!je) je = env->FindClass("java/lang/Exception");
//        env->ThrowNew(je, e.what());
//        return;
//    } catch (...) {
//        AndroidBitmap_unlockPixels(env, bitmap);
//        jclass je = env->FindClass("java/lang/Exception");
//        env->ThrowNew(je, "Unknown exception in JNI code {nMatToBitmap}");
//        return;
//    }
//}

extern "C" JNIEXPORT jstring JNICALL
Java_ups_vision_practica31recfiguras_Parte2Activity_stringFromJNI2(
        JNIEnv* env,
        jobject /* this */) {
    std::string hello = "Hello from C++ in Parte2Activity";
    return env->NewStringUTF(hello.c_str());
}

//extern "C"
//JNIEXPORT jstring JNICALL
//Java_ups_vision_practica31recfiguras_Parte2Activity_Modelo
//        (JNIEnv* env,
//         jobject /*this*/,
//         jobject bitmapIn,
//         jobject bitmapOut) {
//
//    Mat src, filtro;
//    bitmapToMat(env, bitmapIn, src, false);
//
//}
