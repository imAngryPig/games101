// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

float getZfromCrossDot(Eigen::Vector2f p1,Eigen::Vector2f p2){
    return (p1[0]*p2[1]) - (p1[1] * p2[0]);
}

static bool insideTriangle(float x, float y, const Vector4f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector2f p;
    p<<x,y;
    Eigen::Vector2f AB = _v[1].head(2) - _v[0].head(2);
    Eigen::Vector2f BC = _v[2].head(2) - _v[1].head(2);
    Eigen::Vector2f CA = _v[0].head(2) - _v[2].head(2);

    Eigen::Vector2f AP = p - _v[0].head(2);
    Eigen::Vector2f BP = p - _v[1].head(2);
    Eigen::Vector2f CP = p - _v[2].head(2);

    if(getZfromCrossDot(AB,AP)>0 && getZfromCrossDot(BC,BP)>0 && getZfromCrossDot(CA,CP)>0) return true;
    if(getZfromCrossDot(AB,AP)<0 && getZfromCrossDot(BC,BP)<0 && getZfromCrossDot(CA,CP)<0) return true;
    return false;

}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0; 
    float f2 = (50 + 0.1) / 2.0; 

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            float tempW=vec.w();
            vec /= vec.w(); // 此处有误，vec.w/vec.w=1 ,损失了z深度值
            vec.w()=tempW;  // 恢复一下w()的深度值，注意此处w()应该是负数
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            //std::cout<<vert.x()<<" "<<vert.y()<<" "<<vert.z()<<std::endl;
            std::cout<<vert.w()<<std::endl;
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;  //这里是，把-1~1 给拉伸成0.1~50这个范围 再加 -1~0这个对应0.1~50这个范围的距离
                                            //意思是，把标准观察体移到标准视口坐标，把z深度拉伸成0.1~50这个范围的值
            //std::cout<<vert.z()<<std::endl;
            vert.z()=-vert.z(); //为了下面的深度计算，代表离摄像机的远近 ?此处存疑
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i]);
            t.setVertex(i, v[i]);
            t.setVertex(i, v[i]);
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    float min_x = std::min(v[0][0],std::min(v[1][0],v[2][0]));
    float max_x = std::max(v[0][0],std::max(v[1][0],v[2][0]));
    float min_y = std::min(v[0][1],std::min(v[1][1],v[2][1]));
    float max_y = std::max(v[0][0],std::max(v[1][0],v[2][0]));

    min_x=(int)std::floor(min_x);
    max_x=(int)std::ceil(max_x);
    min_y=(int)std::floor(min_y);
    max_y=(int)std::ceil(max_y);

    bool MSAA=true;
    if(MSAA){ //MSAA2x
        std::vector<Eigen::Vector2f> pos{
            {0.25,0.25},
            {0.75,0.25},
            {0.25,0.75},
            {0.75,0.75},
        };
        for(int x=min_x;x<=max_x;x++){
            for(int y=min_y;y<=max_y;y++){
                float minDepth=FLT_MAX-10;
                int count=0;
                for(int i=0;i<4;i++){
                    if(insideTriangle((float)x+pos[i][0],(float)y+pos[i][1],t.v)){
                    auto [alpha, beta, gamma] = computeBarycentric2D((float)x+pos[i][0], (float)y+pos[i][1], t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal; 
                    minDepth=std::min(minDepth,abs(w_reciprocal));
                    count++;
                    }
                }
                if(count){
                    if(depth_buf[get_index(x,y)]>minDepth){
                        Eigen::Vector3f color= t.getColor() * count/4;
                        Vector3f point;
                        point<< (float)x,(float)y,minDepth;
                        depth_buf[get_index(x,y)]=minDepth;
                        set_pixel(point,color);
                    }
                }
            }
        }
    }
    else{
        for(int x=min_x;x<=max_x;x++){
            for(int y=min_y;y<=max_y;y++){
                if(insideTriangle((float)x+0.5,(float)y+0.5,t.v)){
                    auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal; //这里的z深度取了绝对值，越小越靠近相机

                    if(depth_buf[get_index(x,y)] > abs(w_reciprocal)){
                        depth_buf[get_index(x,y)]=abs(w_reciprocal);
                        Vector3f color=t.getColor();
                        Vector3f point;
                        point<< x,y,abs(w_reciprocal);
                        set_pixel(point,color);

                    }
                }
            }
        }
    }
    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x; // cv以左上角为原点，注意y轴翻转
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x(); // cv以左上角为原点，注意y轴翻转
    frame_buf[ind] = color;

}

// clang-format on