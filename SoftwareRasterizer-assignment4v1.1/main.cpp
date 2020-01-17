#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix()
{

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, -2,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // Students will implement this function
    float f = cosf((eye_fov * MY_PI/180.0f)/2.0f) / sinf((eye_fov * MY_PI / 180.0f)/2.0f);

    Eigen::Matrix4f projection;
    projection <<  f/aspect_ratio, 0.0, 0.0, 0.0,
            0.0, f, 0.0, 0.0,
            0.0, 0.0, (zNear+zFar)/(zNear-zFar), 2*zFar*zNear/(zNear-zFar),
            0.0, 0.0, -1.0, 0.0;

    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f kd = payload.color/255.0;
    
    if (payload.texture) // just to make sure that the texture has been set
    {
        Eigen::Vector3f texture_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
        kd = texture_color;
        Eigen::Vector3f ka = Eigen::Vector3f(0.1, 0.1, 0.1);
        Eigen::Vector3f ks = Eigen::Vector3f(1.5, 1.5, 1.5);
        Eigen::Vector3f light_position{10, 10, 10};
        Eigen::Vector3f light_intensity{50000, 50000, 50000};
        Eigen::Vector3f amb_light_intensity{150, 150, 150};
        Eigen::Vector3f eye_pos(0.0,0.0,0.0);
        float p=150.0;
        
        Eigen::Vector3f color = payload.color;
        Eigen::Vector3f point = payload.view_pos;
        Eigen::Vector3f normal = payload.normal;
        
        
        Eigen::Vector3f ambient;
        ambient << ka.x() * amb_light_intensity.x(), ka.y() * amb_light_intensity.y(), ka.z() * amb_light_intensity.z();
        
        Eigen::Vector3f diffuser;
        Eigen::Vector3f diffuse;
        int radius;
        radius = sqrt(pow(point.x()-light_position.x(),2) + pow(point.y()-light_position.y(),2) + pow(point.z()-light_position.z(),2));
        
        Eigen::Vector3f intensity;
        intensity << light_intensity.x()/radius, light_intensity.y()/radius, light_intensity.z()/radius;
        diffuser << kd.x() * intensity.x(), kd.y() * intensity.y(), kd.z() * intensity.z();
        int maxer = 0;
        if(maxer < (normal.x()*light_position.x() + normal.y()*light_position.y() + normal.z()*light_position.z()))
            maxer = normal.x()*light_position.x() + normal.y()*light_position.y() + normal.z()*light_position.z();
        diffuse << diffuser.x()* maxer, diffuser.y()* maxer, diffuser.z()* maxer;
        
        Eigen::Vector3f spec;
        Eigen::Vector3f specular;
        spec << ks.x() * intensity.x(), ks.y() * intensity.y(), ks.z() * intensity.z();
        int maxed = 0;
        Eigen::Vector3f bisector;
        bisector << eye_pos.x() + light_position.x(), eye_pos.y() + light_position.y(), eye_pos.z() + light_position.z();
        Eigen::Vector3f h;
        int magnitude = sqrt(pow(bisector.x(),2) + pow(bisector.y(),2) + pow(bisector.z(),2));
        h << bisector.x()/magnitude, bisector.y()/magnitude, bisector.z()/magnitude;
        if (maxed < (normal.x()*h.x() + normal.y()*h.y() + normal.z()*h.z())) {
            maxed = normal.x()*h.x() + normal.y()*h.y() + normal.z()*h.z();
        }
        maxed =  pow(maxed,p);
        specular << spec.x() * maxed, spec.y() * maxed, spec.z() * maxed;
        
        color << ambient.x() + diffuse.x() + specular.x(), ambient.y() + diffuse.y() + specular.y(), ambient.z() + diffuse.z() + specular.z();
        return color;
        
    }
    
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.1, 0.1, 0.1);
    Eigen::Vector3f kd = payload.color/255.0;
    Eigen::Vector3f ks = Eigen::Vector3f(1.5, 1.5, 1.5);
    Eigen::Vector3f light_position{10, 10, 10};
    Eigen::Vector3f light_intensity{50000, 50000, 50000};
    Eigen::Vector3f amb_light_intensity{150, 150, 150};
    Eigen::Vector3f eye_pos(0.0,0.0,0.0);
    float p=150.0;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;
    
    Eigen::Vector3f ambient;
    ambient << ka.x() * amb_light_intensity.x(), ka.y() * amb_light_intensity.y(), ka.z() * amb_light_intensity.z();
    
    Eigen::Vector3f diffuser;
    Eigen::Vector3f diffuse;
    int radius;
    radius = sqrt(pow(point.x()-light_position.x(),2) + pow(point.y()-light_position.y(),2) + pow(point.z()-light_position.z(),2));
    radius = radius * radius;
    Eigen::Vector3f intensity;
    intensity << light_intensity.x()/radius, light_intensity.y()/radius, light_intensity.z()/radius;
    diffuser << kd.x() * intensity.x(), kd.y() * intensity.y(), kd.z() * intensity.z();
    int maxer = 0;
    if(maxer < (normal.x()*light_position.x() + normal.y()*light_position.y() + normal.z()*light_position.z()))
        maxer = normal.x()*light_position.x() + normal.y()*light_position.y() + normal.z()*light_position.z();
    diffuse << diffuser.x()* maxer, diffuser.y()* maxer, diffuser.z()* maxer;
    
    Eigen::Vector3f spec;
    Eigen::Vector3f specular;
    spec << ks.x() * intensity.x(), ks.y() * intensity.y(), ks.z() * intensity.z();
    int maxed = 0;
    Eigen::Vector3f bisector;
    bisector << eye_pos.x() + light_position.x(), eye_pos.y() + light_position.y(), eye_pos.z() + light_position.z();
    Eigen::Vector3f h;
    int magnitude = sqrt(pow(bisector.x(),2) + pow(bisector.y(),2) + pow(bisector.z(),2));
    h << bisector.x()/magnitude, bisector.y()/magnitude, bisector.z()/magnitude;
    if (maxed < (normal.x()*h.x() + normal.y()*h.y() + normal.z()*h.z())) {
        maxed = normal.x()*h.x() + normal.y()*h.y() + normal.z()*h.z();
    }
    maxed =  pow(maxed,p);
    specular << spec.x() * maxed, spec.y() * maxed, spec.z() * maxed;
    
    color << ambient.x() + diffuse.x() + specular.x(), ambient.y() + diffuse.y() + specular.y(), ambient.z() + diffuse.z() + specular.z();

    //TODO : Find the color of the fragment, and return the value.

  return color; //the final color : ambient + diffuse + specular;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 0.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/rock/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/rock/rock.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X,mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    for (auto& mtl : Loader.LoadedMaterials)
    {
        if (!mtl.map_Kd.empty())
        {
            r.set_texture(Texture(obj_path + mtl.map_Kd));
        }
    }

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            active_shader = texture_fragment_shader;
        }
        else
        {
            active_shader = phong_fragment_shader;
        }
    }
    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix());
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix());
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

    }
    return 0;
}
