//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;

        // width = image_data.cols /2;
        // height = image_data.rows /2;
        // cv::pyrDown(image_data, image_data, cv::Size(width ,height)); //把texture下采样一倍，变为原来的二分之一分辨率
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        // auto  u_img = u * width;
        // auto v_img = (1 - v) * height;
        int u_img = static_cast<int>(u * width);
        int v_img = static_cast<int>((1 - v) * height);
        if (u_img < 0) u_img = 0;
        if (u_img >= width) u_img = width-1;
        if (v_img < 0) v_img = 0;
        if (v_img >= height) v_img = height-1;

        cv::Vec3b color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    int rangeSafe(int x,bool sel){
        if(x<0) return 0;
        if(x>width && sel) return width-1;
        if(x>width && !sel) return height-1; 
        return x;
    }

    Eigen::Vector3f getColorBilinear(float u,float v)   //双线性插值
    {
        float u_img = u * width;
        float v_img = (1-v) * height;

        int min_u=rangeSafe(floor(u_img),true);
        int max_u=rangeSafe(ceil(u_img),true);
        int min_v=rangeSafe(floor(v_img),false);
        int max_v=rangeSafe(ceil(v_img),false);

        cv::Vec3b u00= image_data.at<cv::Vec3b>(max_v,min_u); //opencv的坐标在左上角，并且读取的时候先y,后x
        cv::Vec3b u01= image_data.at<cv::Vec3b>(min_v,min_u);
        cv::Vec3b u11= image_data.at<cv::Vec3b>(min_v,max_u);
        cv::Vec3b u10= image_data.at<cv::Vec3b>(max_v,max_u);

        cv::Vec3b lerp_s_bot= (u_img-min_u)/(max_u-min_u)*u10 + (1-(u_img-min_u))/(max_u-min_u)* u00;
        cv::Vec3b lerp_s_top= (u_img-min_u)/(max_u-min_u)*u11 + (1-(u_img-min_u))/(max_u-min_u)* u01;

        cv::Vec3b lerp_t=(v_img-min_v)/(max_v-min_v)*lerp_s_bot + (1-(v_img-min_v))/(max_v-min_v)* lerp_s_top;
        return Eigen::Vector3f (lerp_t[0],lerp_t[1],lerp_t[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
