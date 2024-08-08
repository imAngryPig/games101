#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

const int pt_num=4;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < pt_num) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if(control_points.size()==1) return control_points[0];
    std::vector<cv::Point2f> tempVec;
    for(int i=0;i<control_points.size()-1;i++){
        cv::Point2f tempP=control_points[i+1]-control_points[i];
        auto temp1=t*tempP + control_points[i];
        tempVec.emplace_back(temp1);
    }
    return recursive_bezier(tempVec,t);

    //return cv::Point2f();

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    auto &p_0 = control_points[0];
    auto &p_1 = control_points[1];
    auto &p_2 = control_points[2];
    auto &p_3 = control_points[3];

     for (float t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point=recursive_bezier(control_points,t);

        int min_u=floor(point.x);
        int max_u=ceil(point.x);
        int min_v=floor(point.y);
        int max_v=ceil(point.y);

        cv::Point2f p00(min_u,min_v);  //此反走样是根据邻近四个点做插值，可以根据最邻近8个点做插值
        cv::Point2f p01(max_u,min_v);
        cv::Point2f p10(min_u,max_v);
        cv::Point2f p11(max_u,max_v);

        int d1=sqrt(pow(point.x-p00.x,2)+pow(point.y-p00.y,2));
        int d2=sqrt(pow(point.x-p01.x,2)+pow(point.y-p01.y,2));
        int d3=sqrt(pow(point.x-p10.x,2)+pow(point.y-p10.y,2));
        int d4=sqrt(pow(point.x-p11.x,2)+pow(point.y-p11.y,2));

        window.at<cv::Vec3b>(p00.y, p00.x)[1] = MAX(255 * (1-1/sqrt(2)*d1),window.at<cv::Vec3b>(p00.y, p00.x)[1]);
        window.at<cv::Vec3b>(p01.y, p01.x)[1] = MAX(255 * (1-1/sqrt(2)*d2),window.at<cv::Vec3b>(p01.y, p01.x)[1]);
        window.at<cv::Vec3b>(p10.y, p10.x)[1] = MAX(255 * (1-1/sqrt(2)*d3),window.at<cv::Vec3b>(p10.y, p10.x)[1]);
        window.at<cv::Vec3b>(p11.y, p11.x)[1] = MAX(255 * (1-1/sqrt(2)*d4),window.at<cv::Vec3b>(p11.y, p11.x)[1]);
        

        //window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    // cv::cvtColor(window, window, cv::COLOR_BGR2RGB);  无意义
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == pt_num) 
        {
            naive_bezier(control_points, window);
            //bezier(control_points, window);

            cv::cvtColor(window, window, cv::COLOR_RGB2BGR); // cv默认 BGR 转为 RGB
            cv::cvtColor(window, window, cv::COLOR_BGR2RGB); // 刚刚转为了 RGB ，再转回 BGR
            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
