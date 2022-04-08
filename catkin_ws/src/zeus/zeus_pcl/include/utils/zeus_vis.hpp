// Author: Keenan Burnett
// Copyright (C) 2020 aUToronto
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!
   \file zeus_vis.hpp
   \brief Helper functions for visualization pointclouds in perspective and BEV.
*/
#pragma once
#include <Eigen/Geometry>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "types/zeus_pcl_types.hpp"

namespace zeus_pcl {

/*!
   \brief Given a cloud of points which have already been projected onto image coordinates (u,v), draw them on the image
   \param img The image on which to draw the points.
   \param uv a pointcloud in which the first two dims (x,y) are the position in the image (u,v) in pixels.
*/
void project_onto_image(cv::Mat &img, PointCloudPtr uv);

/*!
   \brief Given a top-down image, draw lines for the horizontal field of view of a camera.
   \param bev The Bird's Eye View image on which to draw the field of view.
   \param image_width The width of the camera image.
   \param fx the horizontal focal length of the camera.
   \param cam_center the position of the camera in the Bird's Eye View image in pixels (u,v).
*/
void draw_hfov(cv::Mat& bev, int image_width, double fx, cv::Point cam_center);

/*!
   \brief Draws a 3D cube, projected onto a perspective image.
   \param img The image on which to draw the cube.
   \param cuboid_parameters [x, y, z, l, w, h]
   \param P 4x4 camera project matrix
   \param viz_divide cube dims = 1 / viz_divide when projected onto the image.
   \param color The color to draw the cube with.
*/
void draw_cube(cv::Mat &img, std::vector<float> cuboid_parameters, Eigen::Matrix4f P,
        float viz_divide, cv::Scalar color);

/*!
   \brief Given a cloud points, this function create a black image with white points for each lidar point from top-down.
   \param velo The cloud of points that will be drawn.
   \param h_metric The height of the bird's eye view image in metric.
   \param w_metric The width of the bird's eye view image in metric.
   \param meter_to_pixel The conversion from meters to pixels Ex: 20 --> 1 m = 20 pixels.
   \param bev [out] The image that will be drawn onto.
   \param T_bev_v The 4x4 transformation matrix from the sensor frame (v) to the bev frame.
*/
void create_bev(PointCloudPtr velo, float h_metric, float w_metric, float meter_to_pixel,
    cv::Mat &bev, Eigen::Matrix4d T_bev_v);
}  // namespace zeus_pcl
