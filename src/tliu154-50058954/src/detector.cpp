#include "armor_detector/detector.hpp"

#include "armor_detector/armor.hpp"
#include "armor_detector/image_utils.hpp"
#include "armor_detector/light_bar.hpp"

namespace armor_detector {

ArmorDetector::ArmorDetector(const DetectorParams& params) : params_(params) {}

void ArmorDetector::setTargetColor(ColorType color) {
    target_color_ = color;
}

void ArmorDetector::setParams(const DetectorParams& params) {
    params_ = params;
}

std::vector<Armor> ArmorDetector::detect(const cv::Mat& raw_image) {

    std::vector<Armor> armors;
    cv::Mat mask;
    //预处理
    mask=   preprocess(raw_image,this->target_color_,this->params_);
    
    //检测灯条
    std::vector<LightBar> bars;
    bars=   detectLightBars(mask,this->params_,this->target_color_);

    //配对装甲板
    armors= matchArmors(bars,this->params_);

    //保存调试信息
    this->debug_info_.binary_image=mask;
    this->debug_info_.light_bar_image=raw_image.clone();
    this->debug_info_.armor_image=raw_image.clone();
    drawLightBars(this->debug_info_.light_bar_image,bars);
    drawArmors(this->debug_info_.armor_image,armors);
    
    return armors;
}

const DebugInfo& ArmorDetector::getDebugInfo() const {
    return debug_info_;
}

} // namespace armor_detector
