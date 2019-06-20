#include <iostream>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <chrono>


using namespace Eigen;
using namespace std;
using namespace cv; 

int main()
{

    cout <<  "Generate 2 and add two matrices." << endl;
    Matrix<char,3,3> m0;
    m0 << 120, 20, -20,
         -120, -20, 20,
          120, 20, -20;
    Matrix<char,2,2> m1;
    m1 << 20, 20,
         -20, -20;
    cout << "First Matrix:" << endl << m0.cast<int>() << endl;
    cout << "Second Matrix:" << endl << m1.cast<int>() << endl;
    m0.block(0,0,2,2) += m1;
    cout << "Matrix Addition:" << endl << m0.cast<int>() << endl;

    cout << "---------------------------" << endl;

    cout << "Convert to cv::Mat:" << endl;
    Mat m2;
    eigen2cv(m0,m2);
    cout << m2 << endl;
    cout << "Convert back to Eigen:Matrix:" << endl;
    Matrix<char,Dynamic,Dynamic> m3;
    cv2eigen(m2,m3);
    cout << m3.cast<int>() << endl;

    cout << "---------------------------" << endl;

    cout << "Add to big Matrices (Size 500x500 / 60x60) and compare performace of Eigen and OpenCV:" << endl;

    Mat a(500,500,CV_8SC1);
    randu(a, Scalar::all(-127), Scalar::all(128));
    Mat b(60,60,CV_8SC1);
    randu(b, Scalar::all(-127), Scalar::all(128));
   	auto start = std::chrono::system_clock::now();
    a(Rect(352,111,60,60)) += b;
	
  	double duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - start).count();
    cout << "OpenCV: " << duration << " us." << endl;

    Matrix<char,Dynamic,Dynamic> c = Matrix<char,500,500>::Random();
    Matrix<char,Dynamic,Dynamic> d = Matrix<char,60,60>::Random();
    start = std::chrono::system_clock::now();
    c.block(352,111,60,60) += d; 
	
    duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - start).count();
    cout << "Eigen: " << duration << " us." << endl;

}
