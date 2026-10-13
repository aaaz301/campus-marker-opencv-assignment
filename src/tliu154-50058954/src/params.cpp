#include "armor_detector/params.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace armor_detector {
    
/**
 * @brief 读取并检测值是否为空
 */
template <typename T>
T readValue(const YAML::Node &node ,const std::string &key,T default_value){
    if(!node[key].IsDefined()){
        std::cerr<<key<<" is not defined. Using default value: "<<default_value<<std::endl;
        return default_value;
    }
    if(!node[key].IsScalar()){
        std::cerr<<key<<" : please intput a scalar, not a string. Using default value: "<<default_value<<std::endl;
        return default_value;
    }

    return node[key].as<T>();
}

DetectorParams loadParams(const std::string& yaml_path) {
    if (!std::filesystem::exists(yaml_path)) {
        throw std::runtime_error("Params file not found: " + yaml_path);
    }

    YAML::Node config = YAML::LoadFile(yaml_path);
    DetectorParams params;

    //读取参数
    params.red_hue_low_min=readValue<int>(config,"red_hue_low_min",0);
    params.red_hue_low_max=readValue<int>(config,"red_hue_low_max",20);
    params.red_hue_high_min=readValue<int>(config,"red_hue_high_min",160);
    params.red_hue_high_max=readValue<int>(config,"red_hue_high_max",179);
    params.blue_hue_min=readValue<int>(config,"blue_hue_min",100);
    params.blue_hue_max=readValue<int>(config,"blue_hue_max",140);
    params.saturation_min=readValue<int>(config,"saturation_min",80);
    params.value_min=readValue<int>(config,"value_min",80);
    params.morph_kernel_size=readValue<int>(config,"morph_kernel_size",3);
    params.morph_iterations=readValue<int>(config,"morph_iterations",1);
    params.fill_holes=readValue<bool>(config,"fill_holes",true);
    params.min_light_bar_area=readValue<float>(config,"min_light_bar_area",20.0f);
    params.max_light_bar_area=readValue<float>(config,"max_light_bar_area",5000.0f);
    params.min_light_bar_ratio=readValue<float>(config,"min_light_bar_ratio",1.5f);
    params.max_light_bar_ratio=readValue<float>(config,"max_light_bar_ratio",20.0f);
    params.max_light_bar_angle=readValue<float>(config,"max_light_bar_angle",45.0f);
    params.max_bar_height_diff_ratio=readValue<float>(config,"max_bar_height_diff_ratio",0.5f);
    params.max_bar_angle_diff=readValue<float>(config,"max_bar_angle_diff",15.0f);
    params.min_armor_aspect_ratio=readValue<float>(config,"min_armor_aspect_ratio",0.3f);
    params.max_armor_aspect_ratio=readValue<float>(config,"max_armor_aspect_ratio",4.0f);
    params.max_armor_vertical_angle=readValue<float>(config,"max_armor_vertical_angle",20.0f);
    params.save_images=readValue<bool>(config,"save_images",true);
    params.output_dir=readValue<std::string>(config,"output_dir","data/output");
    

    //验证参数范围是否合法
    params.validate();

    return params;
}

void DetectorParams::saveToYaml(const std::string& yaml_path) const {
    YAML::Node config;

    config["red_hue_low_min"] = red_hue_low_min;
    config["red_hue_low_max"] = red_hue_low_max;
    config["red_hue_high_min"] = red_hue_high_min;
    config["red_hue_high_max"] = red_hue_high_max;
    config["blue_hue_min"] = blue_hue_min;
    config["blue_hue_max"] = blue_hue_max;
    config["saturation_min"] = saturation_min;
    config["value_min"] = value_min;

    config["morph_kernel_size"] = morph_kernel_size;
    config["morph_iterations"] = morph_iterations;
    config["fill_holes"] = fill_holes;

    config["min_light_bar_area"] = min_light_bar_area;
    config["max_light_bar_area"] = max_light_bar_area;
    config["min_light_bar_ratio"] = min_light_bar_ratio;
    config["max_light_bar_ratio"] = max_light_bar_ratio;
    config["max_light_bar_angle"] = max_light_bar_angle;

    config["max_bar_height_diff_ratio"] = max_bar_height_diff_ratio;
    config["max_bar_angle_diff"] = max_bar_angle_diff;
    config["min_armor_aspect_ratio"] = min_armor_aspect_ratio;
    config["max_armor_aspect_ratio"] = max_armor_aspect_ratio;
    config["max_armor_vertical_angle"] = max_armor_vertical_angle;

    config["save_images"] = save_images;
    config["output_dir"] = output_dir;

    std::ofstream fout(yaml_path);
    if (!fout.is_open()) {
        throw std::runtime_error("Failed to open file for writing: " + yaml_path);
    }
    fout << config;
}

} // namespace armor_detector
