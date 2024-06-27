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

//Metodo de Moemtnos de Hu

double momentosHuCirculo[7] = {1.60625325e-01, 4.35777910e-04, 7.03658368e-06, 3.09147731e-08, -1.33794872e-14, -6.22958925e-10, 5.37512611e-15};
double momentosHuCuadrado[7] = {1.69139028e-01, 4.20432180e-04, 7.26247928e-05, 2.39599503e-06, -2.98203693e-11, -4.52677160e-08, -1.04734442e-11};
double momentosHuTriangulo[7] = {2.21053266e-01, 1.18374855e-03, 1.01162740e-02, 1.94435975e-04, 2.68714545e-07, 4.52135269e-06, 4.64153988e-08};

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
    Mat src,gray, edges;
    bitmapToMat(env, bitmapIn, src, false);
    String tipo;
    cvtColor(src, gray, COLOR_BGR2GRAY);

    GaussianBlur(gray, gray, Size(5, 5), 0);

    Canny(gray, edges, 50, 150, 3);

    threshold(edges, edges, 150, 255, THRESH_BINARY);

//    Mat result = Mat::zeros(edges.size(), CV_8UC1);

//    vector<vector<Point>> contours;
//    vector<Vec4i> hierarchy;
//    findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    Mat result2 = Mat::zeros(edges.size(), CV_8UC1);

    vector<vector<Point>> contours2;
    vector<Vec4i> hierarchy2;
    findContours(edges, contours2, hierarchy2, RETR_TREE, CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours2.size(); i++) {
        if (contourArea(contours2[i]) < 100)
            continue;
        drawContours(result2, contours2, (int)i, Scalar(255), FILLED, LINE_8);
    }

    Moments momentos = moments(result2, true);
    double momentosHu[7];
    HuMoments(momentos, momentosHu);

    double distanciaCirculo = distanciaEuclidea(momentosHu, momentosHuCirculo);
    double distanciaCuadrado = distanciaEuclidea(momentosHu, momentosHuCuadrado);
    double distanciaTriangulo = distanciaEuclidea(momentosHu, momentosHuTriangulo);

    if (distanciaCirculo < distanciaCuadrado && distanciaCirculo < distanciaTriangulo)
        tipo="Circulo" ;
    else if (distanciaCuadrado < distanciaCirculo && distanciaCuadrado < distanciaTriangulo)
        tipo="Cuadrado";
    else if (distanciaTriangulo < distanciaCirculo && distanciaTriangulo < distanciaCuadrado)
        tipo="Triangulo";
    else
        tipo="Desconocido";

    matToBitmap(env, result2, bitmapOut, false);
    string salida ="        Tipo de Figura: "+tipo+"\n"+"\nMomentos de HU\nM1: "+to_string(momentosHu[0])+"\nM2: "+to_string(momentosHu[1]) +"\nM3: "+to_string(momentosHu[2]) +"\nM4: "+to_string(momentosHu[3])+"\nM5: "+to_string(momentosHu[4]) +"\nM6: "+to_string(momentosHu[5]) +"\nM7: "+to_string(momentosHu[6]);
    return env->NewStringUTF(salida.c_str());
}



complex<double> ZernikePolynomial(int n, int m, double rho, double theta) {
    complex<double> zernike(0, 0);
    int abs_m = abs(m);

    for (int k = 0; k <= (n - abs_m) / 2; ++k) {
        double coefficient = pow(-1, k) * tgamma(n - k + 1) /
                             (tgamma(k + 1) * tgamma((n + abs_m) / 2 - k + 1) * tgamma((n - abs_m) / 2 - k + 1));
        zernike += coefficient * pow(rho, n - 2 * k);
    }

    zernike *= exp(complex<double>(0, m * theta));
    return zernike;
}

void calculateZernikeMoments(const Mat& image, vector<double>& zernikeMoments, int maxOrder) {
    int rows = image.rows;
    int cols = image.cols;
    double centerX = cols / 2.0;
    double centerY = rows / 2.0;
    double radius = min(centerX, centerY);

    zernikeMoments.clear();

    for (int n = 0; n <= maxOrder; ++n) {
        for (int m = -n; m <= n; ++m) {
            if ((n - abs(m)) % 2 != 0) continue;

            complex<double> moment(0, 0);
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    double dx = (x - centerX) / radius;
                    double dy = (y - centerY) / radius;
                    double rho = sqrt(dx * dx + dy * dy);
                    double theta = atan2(dy, dx);

                    if (rho <= 1.0) {
                        complex<double> pixelValue(image.at<uchar>(y, x), 0);
                        moment += pixelValue * ZernikePolynomial(n, m, rho, theta);
                    }
                }
            }
            moment *= (n + 1) / M_PI;
            zernikeMoments.push_back(abs(moment));
        }
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
    Mat src,gray, edges;
    bitmapToMat(env, bitmapIn, src, false);


    GaussianBlur(src, gray, Size(5, 5), 0);
    Canny(gray, edges, 50, 150, 3);
    threshold(edges, edges, 150, 255, THRESH_BINARY);

    // Detección de contornos
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // Filtrar contornos pequeños
    Mat result = Mat::zeros(edges.size(), CV_8UC1);
    for (size_t i = 0; i < contours.size(); i++) {
        if (contourArea(contours[i]) < 100)
            continue;
        drawContours(result, contours, (int)i, Scalar(255), FILLED, LINE_8);
    }
    vector<double> zernikeMoments;
    int maxOrder = 4;
    calculateZernikeMoments(result, zernikeMoments, maxOrder);


    vector<double> zernikeCircle = {0.31830989, 0.00525086, 0.00565314, 0.00508664, 0.00600752, 0.00480541, 0.00531992, 0.00541207, 0.00584415, 0.0066657, 0.0065023, 0.00544734, 0.00622078, 0.00676908, 0.00726558, 0.00516562, 0.0076142, 0.00730557, 0.00658719, 0.00651636, 0.00606301, 0.00658364, 0.00706461, 0.00636922, 0.00651369};
    vector<double> zernikeSquare = {0.31830989, 0.00340306, 0.0032316, 0.00368807, 0.00430454, 0.00430744, 0.00406463, 0.00440127, 0.00471398, 0.00576748, 0.00594801, 0.00604264, 0.0049119, 0.00553571, 0.00591176, 0.00538938, 0.00716755, 0.00745487, 0.00682963, 0.00744759, 0.00537216, 0.00626483, 0.00605198, 0.00600737, 0.00591336};
    vector<double> zernikeTriangle = {0.31830989, 0.01366708, 0.01437503, 0.01417456, 0.01314209, 0.01345273, 0.01475687, 0.01472338, 0.01369239, 0.01293096, 0.01463444, 0.01398792, 0.01267337, 0.01406643, 0.01348948, 0.01324206, 0.01357679, 0.01357228, 0.01386642, 0.01449781, 0.01251291, 0.01398507, 0.01296867, 0.01259025, 0.01219276};

    double distanciaCirculo = distanciaEuclidea(zernikeMoments, zernikeCircle);
    double distanciaCuadrado = distanciaEuclidea(zernikeMoments, zernikeSquare);
    double distanciaTriangulo = distanciaEuclidea(zernikeMoments, zernikeTriangle);

    string tipo;
    if (distanciaCirculo < distanciaCuadrado && distanciaCirculo < distanciaTriangulo)
        tipo="Circulo" ;
    else if (distanciaCuadrado < distanciaCirculo && distanciaCuadrado < distanciaTriangulo)
        tipo="Cuadrado";
    else if (distanciaTriangulo < distanciaCirculo && distanciaTriangulo < distanciaCuadrado)
        tipo="Triangulo";
    else
        tipo="Desconocido";

    matToBitmap(env, result, bitmapOut, false);
    string salida ="        Tipo de Figura: "+tipo;
    return env->NewStringUTF(salida.c_str());

}

//Zernike

//
//void calculateZernikeMoments(const Mat& image, vector<double>& zernikeMoments) {
//    // Predefined Zernike moments for each category (these values should be calculated beforehand)
//    static const vector<double> zernikeCircle = {0.2, 0.1, 0.3, 0.1, 0.2, 0.4, 0.3, 0.1, 0.2, 0.4};
//    static const vector<double> zernikeSquare = {0.4, 0.2, 0.3, 0.1, 0.4, 0.2, 0.1, 0.3, 0.4, 0.1};
//    static const vector<double> zernikeTriangle = {0.3, 0.3, 0.2, 0.2, 0.3, 0.3, 0.2, 0.1, 0.3, 0.4};
//
//    // Calculate moments and assign predefined values
//    zernikeMoments = zernikeCircle; // Replace with actual calculation if needed
//}
//
//double distanciaEuclidea(const vector<double>& moments1, const vector<double>& moments2) {
//    double suma = 0;
//    for (size_t i = 0; i < moments1.size(); ++i) {
//        suma += (moments1[i] - moments2[i]) * (moments1[i] - moments2[i]);
//    }
//    return sqrt(suma);
//}
//
//extern "C"
//JNIEXPORT jstring JNICALL
//Java_ups_vision_practica31recfiguras_MainActivity_MomentosZernike
//        (JNIEnv* env,
//         jobject /*this*/,
//         jobject bitmapIn,
//         jobject bitmapOut) {
//    Mat src;
//    bitmapToMat(env, bitmapIn, src, false);
//    String tipo;
//
//    Mat gray, edges, imagen;
//    cvtColor(src, gray, COLOR_BGR2GRAY);
//    GaussianBlur(gray, gray, Size(5, 5), 0);
//    Canny(gray, edges, 50, 150, 3);
//    threshold(edges, edges, 150, 255, THRESH_BINARY);
//
//    Mat result = Mat::zeros(edges.size(), CV_8UC1);
//
//    vector<vector<Point>> contours;
//    vector<Vec4i> hierarchy;
//    findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
//
//    Mat result2 = Mat::zeros(edges.size(), CV_8UC1);
//
//    vector<vector<Point>> contours2;
//    vector<Vec4i> hierarchy2;
//    findContours(edges, contours2, hierarchy2, RETR_TREE, CHAIN_APPROX_SIMPLE);
//
//    for (size_t i = 0; i < contours2.size(); i++) {
//        if (contourArea(contours2[i]) < 100)
//            continue;
//        drawContours(result2, contours2, (int)i, Scalar(255), FILLED, LINE_8);
//    }
//
//    vector<double> zernikeMoments;
//    calculateZernikeMoments(result2, zernikeMoments);
//
//    // Predefined Zernike moments for each category
//    vector<double> zernikeCircle = {0.2, 0.1, 0.3, 0.1, 0.2, 0.4, 0.3, 0.1, 0.2, 0.4};
//    vector<double> zernikeSquare = {0.4, 0.2, 0.3, 0.1, 0.4, 0.2, 0.1, 0.3, 0.4, 0.1};
//    vector<double> zernikeTriangle = {0.3, 0.3, 0.2, 0.2, 0.3, 0.3, 0.2, 0.1, 0.3, 0.4};
//
//    double distanciaCircle = distanciaEuclidea(zernikeMoments, zernikeCircle);
//    double distanciaSquare = distanciaEuclidea(zernikeMoments, zernikeSquare);
//    double distanciaTriangle = distanciaEuclidea(zernikeMoments, zernikeTriangle);
//
//    if (distanciaCircle < distanciaSquare && distanciaCircle < distanciaTriangle) {
//        putText(src, "circulo", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
//        tipo = "Circulo";
//    }
//    else if (distanciaSquare < distanciaCircle && distanciaSquare < distanciaTriangle) {
//        tipo = "Cuadrado";
//    }
//    else if (distanciaTriangle < distanciaCircle && distanciaTriangle < distanciaSquare) {
//        tipo = "Triangulo";
//    }
//    else {
//        tipo = "Desconocido";
//    }
//
//    matToBitmap(env, result2, bitmapOut, false);
//    string salida = "        Tipo de Figura: " + tipo + "\n";
//    return env->NewStringUTF(salida.c_str());
//}