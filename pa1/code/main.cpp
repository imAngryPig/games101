#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

float getRad(float angle){
    return (angle/180.0)*MY_PI;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f translate;
    float rad=getRad(rotation_angle);
    translate << cos(rad), -sin(rad), 0, 0, sin(rad), cos(rad), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    
    model = translate * model;

    return model;
}

// Rodrigues' Rotation
Eigen::Matrix4f get_model_matrix(Eigen::Vector3f axis,float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the target axis.
    // Then return it.
    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
    float rad=getRad(rotation_angle);


    Eigen::Vector3f normalize_axis= axis.normalized();
    Eigen::Matrix3f Skew_symmetric_Matrix;
    Skew_symmetric_Matrix<< 0, -normalize_axis.z(), normalize_axis.y(), 
                            normalize_axis.z(), 0, -normalize_axis.x(),
                            -normalize_axis.y(), normalize_axis.x(), 0;
    
    Eigen::Matrix3f model_matrix = Eigen::Matrix3f::Zero();
    model_matrix=cos(rad)* Eigen::Matrix3f::Identity() + (1-cos(rad))*normalize_axis * normalize_axis.transpose() 
            + sin(rad) * Skew_symmetric_Matrix;
    
    translate.topLeftCorner(3,3)=model_matrix;
    
    model = translate * model;

    return model;
}

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle) {
    angle = angle / 180 * MY_PI;
    Eigen::Matrix4f any_rotation = Eigen::Matrix4f::Zero();
    any_rotation(3, 3) = 1;
    
    Eigen::Vector3f normal_axis = axis.normalized();

    Eigen::Matrix3f mult_factor;
    mult_factor << 0, -normal_axis.z(), normal_axis.y(),
        normal_axis.z(), 0, -normal_axis.x(),
        -normal_axis.y(), normal_axis.x(), 0;

    mult_factor = cos(angle) * Eigen::Matrix3f::Identity()
        + (1 - cos(angle)) * normal_axis * normal_axis.transpose()
        + sin(angle) * mult_factor;

    any_rotation.block(0, 0, 3, 3) = mult_factor.block(0,0,3,3);
    //any_rotation.topLeftCorner(3,3)=mult_factor.block(0,0,3,3);
    // 注意  block函数取的是 0,0 ~ (3-1,3-1)这个范围的matrix 不包含3,3
    return any_rotation;
}



Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    zNear=-zNear;
    zFar=-zFar;
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float height=abs(zNear)*tan(getRad(eye_fov/2))*2;
    float width=aspect_ratio*height;
    float r=width/2, l=-(width/2), t=height/2, b=-(height/2), n=zNear, f=zFar;

    Eigen::Matrix4f perspect;

    perspect << zNear, 0, 0, 0, 0, zNear, 0, 0, 0, 0, zNear+zFar, -zNear*zFar, 0, 0, 1, 0;

    Eigen::Matrix4f ortho_trans,ortho_rotate,ortho;
    ortho_trans <<  1,0,0,-(r+l)/2,0,1,0,-(t+b)/2,0,0,1,-(n+f)/2,0,0,0,1;
    ortho_rotate << 2/(r-l),0,0,0, 0,2/(t-b),0,0, 0,0,2/(n-f),0, 0,0,0,1;
    ortho = ortho_rotate*ortho_trans;

    projection = ortho * perspect * projection;

    return projection;
}

Eigen::Matrix4f get_projection_matrix11(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    eye_fov = eye_fov / 180 * MY_PI;
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f aspect_fovY;
    float ty = -1.0f / tan(eye_fov / 2.0f);
    aspect_fovY << (ty / aspect_ratio), 0, 0, 0,
        0, ty, 0, 0,
        0, 0, (zNear+zFar)/(zNear-zFar), (-2*zNear*zFar)/(zNear-zFar),
        0, 0, 1, 0;
    projection = aspect_fovY * projection;
    return projection;
}


int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1f, 50.0f));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    //    r.set_model(get_model_matrix(angle));
    //    r.set_model(get_model_matrix(Eigen::Vector3f(0,1,0),angle));
        r.set_model(get_rotation(Eigen::Vector3f(0,1,0),angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0f, 1.0f, 0.1f, 50.0f));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
