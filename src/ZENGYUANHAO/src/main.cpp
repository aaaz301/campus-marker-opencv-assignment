#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<algorithm>
#include<opencv2/videoio.hpp>
#include<opencv2/core.hpp>
#include<filesystem>
#include"frame_proceed.h"
#define fs std::filesystem
using std::cin;
using std::cout;
using std::string;
void frame_write(fs::path opath,cv::Mat x,int fps,cv::VideoWriter& writer,bool isgrey){
    if(!writer.isOpened())
        writer.open(opath.string(),cv::VideoWriter::fourcc('m','p','4','v')
                    ,fps,x.size(),!isgrey);
    if(!writer.isOpened()) throw std::runtime_error("无法生成视频");
    writer.write(x);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(const int arg,const char** argv){
    int debug=0;
    cout<<"无调试输入0"<<'\n'<<"Debug Number: ";
    cin>>debug;
    if(arg!=3){
        std::cerr<<"输入错误：输入量应为输入地址与输出地址"<<'\n';
        return 1;
    }
    try{
        fs::path input_path(argv[1]);
        if(!fs::is_regular_file(input_path)) throw std::runtime_error("未找到文件");
        fs::path output_path(argv[2]);
        fs::create_directory(output_path.stem());
        output_path=output_path.stem()/output_path;
//--------------------------------------------------------------------------文件处理
        cv::VideoCapture capture(input_path);
        if(!capture.isOpened()) throw std::runtime_error("未找到视频文件");
        int fps=capture.get(cv::CAP_PROP_FPS);
        if(!fps) fps=30;
        int size_=capture.get(cv::CAP_PROP_FRAME_COUNT);
        int count=0;
        int armor_count=0;
        cv::VideoWriter writer;
        cv::Mat frame;
        while(capture.read(frame)){
            if(frame.empty()) throw std::runtime_error("空帧");
            img res=frame_proceed(frame,debug);
            armor_count+=res.armor_number;
            cout<<++count<<"/"<<size_<<'\n';
            frame_write(output_path,res.frame,fps,writer,res.is_grey);
        }
        if(!count) throw std::runtime_error("无可识别帧");
        cout<<"共识别："<<armor_count<<"块装甲板"<<'\n';
    }
    catch(const std::exception& error){std::cerr<<"错误："<<error.what()<<'\n';return 1;}
}