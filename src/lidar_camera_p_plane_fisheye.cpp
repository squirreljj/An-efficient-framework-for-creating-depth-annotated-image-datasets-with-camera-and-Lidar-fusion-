#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <ceres/ceres.h>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
#include "rotation.h"
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud1;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using namespace std;
using namespace cv;
using namespace pcl;
cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
cv::Mat rvec_l_c = cv::Mat::zeros(3, 1, CV_64F);
cv::Mat tvec_l_c = cv::Mat::zeros(3, 1, CV_64F);
double rx = 0;
double ry = 0;
double rz = 0;
double tx = 0;
double ty = 0;
double tz = 0;
struct CostFunctor {
    CostFunctor(double x, double y, double z, double r1, double r2, double r3, double t1, double t2,
                double t3)
        : _x(x), _y(y), _z(z), _r1(r1), _r2(r2), _r3(r3), _t1(t1), _t2(t2), _t3(t3) {}
    template <typename T>
    bool operator()(const T* const rx, const T* const ry, const T* const rz, const T* const tx,
                    const T* const ty, const T* const tz, T* residual) const {
        T point_r[3];  // lidar coordinate
        point_r[0] = T(_x);
        point_r[1] = T(_y);
        point_r[2] = T(_z);
        T point_c[3];  // camera coordinate
        T r[3];
        r[0] = *rx;
        r[1] = *ry;
        r[2] = *rz;
        AngleAxisRotatePoint(r, point_r, point_c);

        // trans
        point_c[0] += *tx;
        point_c[1] += *ty;
        point_c[2] += *tz;
        T point_b[3];

        T r_[3];
        r_[0] = T(_r1);
        r_[1] = T(_r2);
        r_[2] = T(_r3);
        AngleAxisRotatePoint(r_, point_c, point_b);
        // trans
        point_b[0] += T(_t1);
        point_b[1] += T(_t2);
        point_b[2] += T(_t3);

        // point to plane residual
        residual[0] = sqrt(point_b[2] * point_b[2]);
        return true;
    }
    const double _x, _y, _z, _r1, _r2, _r3, _t1, _t2, _t3;
};

int main(int argc, char** argv) {
    //1.load intrinsic
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(1, 4, CV_64F);
    FileStorage f_calib;
    f_calib.open("/home/ljj/work/lidar_camera_p_plane/config/calib_fisheye.yaml",
                 FileStorage::READ);
    if (!f_calib.isOpened()) {
        cout << "Cannot open f_calib" << endl;
        return -1;
    }
    f_calib["intrinsics"] >> cameraMatrix;
    double nums, boardSize;
    int board_w, board_h;
    f_calib["nums"] >> nums;
    f_calib["banzi_row"] >> board_w;
    f_calib["banzi_col"] >> board_h;
    f_calib["squareSize"] >> boardSize;
    f_calib["distortion_coeffs"] >> distCoeffs;
    std::vector<cv::Vec3d> rotation_vectors;
    std::vector<cv::Vec3d> translation_vectors;
    const int NPoints = board_w * board_h;  //total corners numbers 
    Size ChessBoardSize = cv::Size(board_w, board_h);
    vector<Point2f> tempcorners;
    int flag = 0;
    double image_cols, image_rows;
    flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    flag |= cv::fisheye::CALIB_FIX_SKEW;
    vector<Point3f> object;
    for (int j = 0; j < NPoints; j++) {
        object.push_back(Point3f((j % board_w) * boardSize, (j / board_w) * boardSize, 0));
    }
    vector<vector<Point3f>> objectv;
    vector<vector<Point2f>> imagev;

    google::InitGoogleLogging(argv[0]);
    // create optimize problems
    ceres::Problem problem;
    for (int i = 0; i < nums; i++) {
        cv::Mat image;
        cv::Mat img;
        cv::Mat image_dst;

        image_dst = cv::imread("/home/ljj/work/lidar_camera_p_plane/img_fisheye/" +
                               std::to_string(i + 1) + ".png");
        image_cols = image_dst.cols;
        image_rows = image_dst.rows;

        //cv::rotate(image, image_dst, cv::ROTATE_90_COUNTERCLOCKWISE);

        //cv::rotate(img, img_dst, cv::ROTATE_90_COUNTERCLOCKWISE);
        // detect corners

        bool find_corners_result = findChessboardCorners(image_dst, ChessBoardSize, tempcorners, 3);
        if (find_corners_result) {
            cout << i + 1 << endl;
            //cornerSubPix(image_dst, tempcorners, cvSize(5, 5), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(image_dst, ChessBoardSize, tempcorners, find_corners_result);
            cv::imshow("corner_image", image_dst);
            cv::waitKey(0);
            objectv.push_back(object);
            imagev.push_back(tempcorners);
        }
        tempcorners.clear();
    }
    cv::fisheye::calibrate(objectv, imagev, cv::Size(image_cols, image_rows), cameraMatrix,
                           distCoeffs, rotation_vectors, translation_vectors, flag,
                           cv::TermCriteria(3, 20, 1e-6));
    for (int i = 0; i < rotation_vectors.size(); i++) {
        cv::Mat img_dst;
        img_dst = cv::imread("/home/ljj/work/lidar_camera_p_plane/img_fisheye/" +
                             std::to_string(i + 1) + ".png");
        cout << rotation_vectors[i] << endl;
        cout << translation_vectors[i] << endl;
        rvec.at<double>(0, 0) = rotation_vectors[i][0];
        rvec.at<double>(1, 0) = rotation_vectors[i][1];
        rvec.at<double>(2, 0) = rotation_vectors[i][2];
        tvec.at<double>(0, 0) = translation_vectors[i][0];
        tvec.at<double>(1, 0) = translation_vectors[i][1];
        tvec.at<double>(2, 0) = translation_vectors[i][2];
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        cv::Mat R_inv;
        cv::invert(R, R_inv, cv::DECOMP_SVD);
        cv::Mat rotation_vector;
        cv::Mat trans_vector;
        cv::Rodrigues(R_inv, rotation_vector);
        trans_vector = -R_inv * tvec;
        // project pointcloud to image 
        std::vector<cv::Point2f> projectedPoints;
        cv::fisheye::projectPoints(objectv[i], projectedPoints, rvec, tvec, cameraMatrix,
                                   distCoeffs);
        for (const auto& p : projectedPoints) {
            cv::circle(img_dst, p, 3, cv::Scalar(0, 255, 0), cv::FILLED);
        }
        cv::imshow("Projected Points", img_dst);
        cv::waitKey(0);
        PointCloud1::Ptr cloud_c1(new PointCloud1);  
        //add code
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(
                "/home/ljj/work/lidar_camera_p_plane/pcd_fisheye/" + std::to_string(i + 1) + ".pcd",
                *cloud_c1) == -1)  //load data path
        {
            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
            system("PAUSE");
        }
        for (size_t i = 0; i < cloud_c1->points.size(); i++) {
            double x = cloud_c1->points[i].x;
            double y = cloud_c1->points[i].y;
            double z = cloud_c1->points[i].z;
            double r1 = rotation_vector.at<double>(0, 0);
            double r2 = rotation_vector.at<double>(1, 0);
            double r3 = rotation_vector.at<double>(2, 0);
            double t1 = trans_vector.at<double>(0, 0);
            double t2 = trans_vector.at<double>(1, 0);
            double t3 = trans_vector.at<double>(2, 0);
            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1, 1, 1, 1, 1>(
                    new CostFunctor(x, y, z, r1, r2, r3, t1, t2, t3));
            problem.AddResidualBlock(cost_function, nullptr, &rx, &ry, &rz, &tx, &ty, &tz);
        }
    }
    cout << "start optimze" << endl;
    ceres::Solver::Options options;
    options.max_num_iterations = 1000;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // print optimize results
    //std::cout << summary.FullReport() << std::endl;
    std::cout << rx << " " << ry << " " << rz << " " << tx << " " << ty << " " << tz << std::endl;
}
