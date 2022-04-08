#include <vector>
#include "utils/zeus_vis.hpp"

namespace zeus_pcl {

static double interpolate(double val, double y0, double x0, double y1, double x1 ) {
    return (val-x0)*(y1-y0)/(x1-x0) + y0;
}

static double base(double val) {
    if ( val <= -0.75 )
        return 0;
    else if ( val <= -0.25 )
        return interpolate( val, 0.0, -0.75, 1.0, -0.25 );
    else if ( val <= 0.25 )
        return 1.0;
    else if ( val <= 0.75 )
        return interpolate( val, 1.0, 0.25, 0.0, 0.75 );
    else
        return 0.0;
}

static double red(double gray) {
    return base( gray - 0.5 );
}
static double green(double gray) {
    return base( gray );
}
static double blue(double gray) {
    return base( gray + 0.5 );
}

static cv::Scalar getColour(float v, float vmin, float vmax) {
    float vnorm = (v - floor(vmax/2)) / ceil(vmax/2);
    float r = 0, g = 0, b = 0;
    r = red(vnorm);
    g = green(vnorm);
    b = blue(vnorm);
    cv::Scalar cOut = cv::Scalar(int(b * 255), int(g * 255), int(r * 255));
    return cOut;
}

static void getMinMax3D(PointCloudPtr pc, Point &min, Point &max) {
    float bignum = 1000000;
    min.x = bignum;
    min.y = bignum;
    min.z = bignum;
    max.x = -1 * bignum;
    max.y = -1 * bignum;
    max.z = -1 * bignum;
    for (uint i = 0; i < pc->size(); i++) {
        if (pc->points[i].x < min.x)
            min.x = pc->points[i].x;
        if (pc->points[i].y < min.y)
            min.y = pc->points[i].y;
        if (pc->points[i].z < min.z)
            min.z = pc->points[i].z;
        if (pc->points[i].x > max.x)
            max.x = pc->points[i].x;
        if (pc->points[i].y > max.y)
            max.y = pc->points[i].y;
        if (pc->points[i].z > max.z)
            max.z = pc->points[i].z;
    }
}

void project_onto_image(cv::Mat &img, PointCloudPtr uv) {
    Point minPt, maxPt;
    int W = img.size().width;
    int H = img.size().height;
    getMinMax3D(uv, minPt, maxPt);
    float dmax = maxPt.z;
    for (int i = 0; i < (int)uv->size(); i++) {
        int u = (int)uv->points[i].x;
        int v = (int)uv->points[i].y;
        cv::Scalar c = getColour(uv->points[i].z, 0, dmax);
        if (u >= 0.5 && u <= W && v >= 0.5 && v <= H)
            cv::drawMarker(img, cv::Point(u, v), c, 3, 3, 3);
    }
}

// points are in camera frame
void create_bev(PointCloudPtr velo, float h_metric, float w_metric, float meter_to_pixel,
    cv::Mat &bev, Eigen::Matrix4d T_bev_v) {
    int w = int(w_metric * meter_to_pixel);
    int h = int(h_metric * meter_to_pixel);
    bev = cv::Mat::zeros(h, w, CV_8UC3);
    int grid_divide = 10 * meter_to_pixel;
    // Draw gridlines
    cv::line(bev, cv::Point(w/2, h), cv::Point(w/2, 0), cv::Scalar(255, 255, 255), 1);
    for (int i = h - grid_divide; i > 0; i -= grid_divide) {
        cv::line(bev, cv::Point(0, i), cv::Point(w, i), cv::Scalar(255, 255, 255), 1);
    }
    for (int i = grid_divide; i < w; i += grid_divide) {
        cv::line(bev, cv::Point(i, 0), cv::Point(i, h), cv::Scalar(255, 255, 255), 1);
    }

    for (int i = 0; i < (int)velo->size(); i++) {
        Eigen::Matrix<double, 4, 1> xbar;
        xbar << velo->points[i].x, velo->points[i].y, velo->points[i].z, 1;
        xbar = T_bev_v * xbar;
        int u = int(meter_to_pixel * xbar(0, 0));
        int v = int(meter_to_pixel * xbar(1, 0));
        if (0 < u && u < w && 0 < v && v < h) {
            bev.at<cv::Vec3b>(v, u) = cv::Vec3b(255, 255, 255);
        }
    }
}

void draw_hfov(cv::Mat& bev, int image_width, double fx, cv::Point cam_center) {
    int bevw = bev.size().width;
    double hfov = 2 * atan(image_width / (2 * fx));
    double theta = (M_PI - hfov) / 2;
    double h_cross = cam_center.y - (bevw / 2) * tan(theta);
    cv::Point P2(0, h_cross);
    cv::Point P3(bevw, h_cross);
    cv::line(bev, cam_center, P2, cv::Scalar(238, 130, 238), 2);
    cv::line(bev, cam_center, P3, cv::Scalar(238, 130, 238), 2);
}

void draw_cube(cv::Mat &img, std::vector<float> cuboid_parameters, Eigen::Matrix4f P,
        float viz_divide, cv::Scalar color) {
    std::vector<cv::Point> points2d;
    std::vector<Eigen::Vector4f> points3d;

    float min_x, min_y, min_z, max_x, max_y, max_z;
    float x, y, z, w, l, h;
    Eigen::Vector4f p3d = Eigen::MatrixXf::Ones(4, 1);

    x = cuboid_parameters[0];
    y = cuboid_parameters[1];
    z = cuboid_parameters[2];
    w = cuboid_parameters[3];
    l = cuboid_parameters[4];
    h = cuboid_parameters[5];

    min_x = x - w / 2;
    max_x = x + w / 2;
    min_y = y - h / 2;
    max_y = y + h / 2;
    min_z = z - l / 2;
    max_z = z + l / 2;

    p3d(0, 0) = max_x;
    p3d(1, 0) = min_y;
    p3d(2, 0) = min_z;
    points3d.push_back(p3d);

    p3d(0, 0) = min_x;
    p3d(1, 0) = min_y;
    p3d(2, 0) = min_z;
    points3d.push_back(p3d);

    p3d(0, 0) = min_x;
    p3d(1, 0) = max_y;
    p3d(2, 0) = min_z;
    points3d.push_back(p3d);

    p3d(0, 0) = max_x;
    p3d(1, 0) = max_y;
    p3d(2, 0) = min_z;
    points3d.push_back(p3d);

    p3d(0, 0) = max_x;
    p3d(1, 0) = min_y;
    p3d(2, 0) = max_z;
    points3d.push_back(p3d);

    p3d(0, 0) = min_x;
    p3d(1, 0) = min_y;
    p3d(2, 0) = max_z;
    points3d.push_back(p3d);

    p3d(0, 0) = min_x;
    p3d(1, 0) = max_y;
    p3d(2, 0) = max_z;
    points3d.push_back(p3d);

    p3d(0, 0) = max_x;
    p3d(1, 0) = max_y;
    p3d(2, 0) = max_z;
    points3d.push_back(p3d);

    for (int i = 0; i < (int)points3d.size(); i++) {
        Eigen::Vector4f p = points3d[i];
        p = p / p(2, 0);
        p = P * p;
        p /= viz_divide;
        cv::Point p2 = cv::Point(p(0, 0), p(1, 0));
        points2d.push_back(p2);
    }

    for (int i = 0; i < (int)points2d.size(); i++) {
        // Top and bottom of cube
        if (i == 3 || i == 7)
            cv::line(img, points2d[i], points2d[i-3], color, 2);
        else
            cv::line(img, points2d[i], points2d[i+1], color, 2);
        // up-right vertical lines
        if (i < 4)
            cv::line(img, points2d[i], points2d[i+4], color, 2);
    }
}

}  // namespace zeus_pcl
