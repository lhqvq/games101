#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

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

    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float angle = rotation_angle / 180.0 * MY_PI;
    model << std::cos(angle), -std::sin(angle), 0.0, 0.0,
           std::sin(angle), std::cos(angle),  0.0, 0.0,
           0.0,             0.0,              1.0, 0.0,
           0.0,             0.0,              0.0, 1.0;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f persp2orth = Eigen::Matrix4f::Identity();
    persp2orth << zNear, 0.0, 0.0, 0.0,
                 0.0, zNear, 0.0, 0.0,
                 0.0, 0.0, zNear + zFar, -(zNear * zFar),
                 0.0, 0.0, 1.0, 0.0;

    float height = std::tan(eye_fov / 180.0 * MY_PI / 2) * std::abs(zNear) * 2;
    float width = aspect_ratio * height;
    // float l = -(width / 2), r = width / 2;
    // float b = -(height / 2), t = height / 2;
    Eigen::Matrix4f orth_translate = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f orth_scale = Eigen::Matrix4f::Identity();
    orth_translate << 1.0, 0.0, 0.0, 0.0,
                      0.0, 1.0, 0.0, 0.0,
                      0.0, 0.0, 1.0, -((zNear + zFar) / 2),
                      0.0, 0.0, 0.0, 1.0;
    orth_scale << 2.0 / width, 0.0, 0.0, 0.0,
                   0.0, 2.0 / height, 0.0, 0.0,
                   0.0, 0.0, 2.0 / (zNear - zFar), 0.0,
                   0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix4f orth;
    orth = orth_scale * orth_translate;
    projection = orth * persp2orth;

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    Eigen::Matrix4f axis2z = Eigen::Matrix4f::Identity();

    float ang = angle / 180.0 * MY_PI;

    Eigen::Vector3f aZ = axis / std::sqrt(axis.dot(axis)); // 归一化，作为 z 轴
    Eigen::Vector3f aX = aZ.cross((Eigen::Vector3f){1.0, 0.0, 0.0});
    Eigen::Vector3f aY = aZ.cross(aX);
    aX = aX / std::sqrt(aX.dot(aX));
    aY = aY / std::sqrt(aY.dot(aY));

    Eigen::Matrix4f axis_transform;
    axis_transform << aX[0], aY[0], aZ[0], 0.0,
                      aX[1], aY[1], aZ[1], 0.0,
                      aX[2], aY[2], aZ[2], 0.0,
                      0.0, 0.0, 0.0, 0.0;
    
    Eigen::Matrix4f rotation = axis_transform.inverse()
                                 * get_model_matrix(angle)
                                 * axis_transform;
    return rotation;
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
        else
            return 0;
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
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

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
