#include"frame_proceed.h"

#include<algorithm>
#include<math.h>
#include<cstring>

std::vector<BarPair> proceed_pair(const std::vector<LightBar>& bars,
                                  const cv::Mat& frame,float max_distance){
    cv::Mat grey;
    cv::Mat bright_pixel;
    cv::cvtColor(frame,grey,cv::COLOR_BGR2GRAY);
    cv::threshold(grey,bright_pixel,80,255,cv::THRESH_BINARY);
    std::vector<BarPair> pairs;
    std::vector<BarPair> proceeded_pair;
    for(int l=0;l<bars.size();l++){
        for(int r=l+1;r<bars.size();r++){
            float mean_length=(bars[l].length+bars[r].length)/2.0F;
            float length_difference=std::max(bars[l].length,bars[r].length)
                                    /std::min(bars[l].length,bars[r].length);
            float distance=abs(bars[l].rect.center.x-bars[r].rect.center.x)/mean_length;
            float hight_difference=abs(bars[l].rect.center.y-bars[r].rect.center.y)/mean_length;
            float angle_difference=abs(bars[l].angle-bars[r].angle);
            if(length_difference>1.5F||distance>max_distance||hight_difference>0.7F
                ||distance<1.0F||angle_difference>45.0F) continue;
            bool skip_armor=false;
            float pair_center=(bars[l].rect.center.x+bars[r].rect.center.x)/2.0F;
            for(int mid=l+1;mid<r;mid++)
                if(abs(pair_center-bars[mid].rect.center.x)<mean_length*0.5F
                    && bars[mid].length*2.0F>=std::min(bars[l].length,bars[r].length)
                    && !(abs(bars[l].rect.center.x-bars[mid].rect.center.x)/mean_length<1.0F)
                    && !(abs(bars[r].rect.center.x-bars[mid].rect.center.x)/mean_length<1.0F)){
                    skip_armor=true;
                    break;
                }
            if(skip_armor) continue;
            float score=std::abs(distance-2.0F)+0.8F*hight_difference
                        +0.6F*(length_difference-1.0F)+0.03*angle_difference;
            pairs.push_back({bars[l],bars[r],l,r,score,{bars[l].top,bars[l].bottom,bars[r].bottom,bars[r].top}});
        }
    }
    std::sort(pairs.begin(),pairs.end(),[](BarPair& x,BarPair& y){
        return x.score<y.score;
    });
    int vis[bars.size()+10];
    memset(vis,0,sizeof(vis));
    for(int i=0;i<pairs.size();i++){
        if(vis[pairs[i].numleft]||vis[pairs[i].numright])continue;
        vis[pairs[i].numleft]=1;
        vis[pairs[i].numright]=1;
        proceeded_pair.push_back(pairs[i]);
    }
    return proceeded_pair;
}

img frame_proceed(const cv::Mat& frame,int x){
    img res[10];
    cv::Mat temp=frame.clone();
    cv::Mat blurred;
    cv::medianBlur(temp,blurred,3);
//-----------------------------------------------------------------
    std::vector<cv::Mat> color_channels;
    cv::Mat color_difference;
    cv::split(blurred,color_channels);
    cv::absdiff(color_channels[0],color_channels[2],color_difference);
//------------------------------------------------------------------
    cv::Mat binary;
    cv::threshold(color_difference,binary,60,255,cv::THRESH_BINARY); 
//-------------------------------------------------------------------
    cv::Mat close_temp;
    cv::Mat close_final;
    cv::Mat close_kernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,5));
    cv::erode(binary,close_temp,close_kernel);
    cv::dilate(close_temp,close_final,close_kernel);
    cv::Mat open_temp;
    cv::Mat open_final;
    cv::Mat open_kernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::dilate(close_final,open_temp,open_kernel);
    cv::erode(open_temp,open_final,open_kernel);
    close_final.release();
    open_temp.release();close_temp.release();
    close_kernel.release();open_kernel.release();
//---------------------------------------------------------------------
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat frame_contours=open_final.clone();
    cv::findContours(frame_contours,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    cv::Mat frame_rected=frame.clone();
    cv::RotatedRect rect;
    std::vector<LightBar> bars;
    for(int i=0;i<contours.size();i++){
        if(cv::contourArea(contours[i])>=15){
            rect=minAreaRect(contours[i]);
            cv::Point2f apex[4];
            rect.points(apex);
            float length=rect.size.height;
            float width=rect.size.width;
            float angle=rect.angle;
            if(length<width){
                std::swap(length,width);
                cv::Point2f tmp=apex[0];
                apex[0]=apex[3];apex[3]=apex[2];
                apex[2]=apex[1];apex[1]=tmp;
                angle=90-angle;
            }
            if(length<width*4||angle>45) continue;
            cv::Point2f axis=apex[0]-apex[1];
            bars.push_back({rect,rect.center-(axis)/2,rect.center+(axis)/2,length,angle});
            for(int i=0;i<4;i++)
                cv::line(frame_rected,apex[i],apex[(i+1)%4],cv::Scalar(0,0,255));
        }
    }
//----------------------------------------------------------------------
    std::sort(bars.begin(),bars.end(),[](const LightBar &a,const LightBar &b){
        return a.rect.center.x<b.rect.center.x;
    });
    std::vector<BarPair> barpairs=proceed_pair(bars,frame,2.5F);
    if(barpairs.empty()) barpairs=proceed_pair(bars,frame,2.7F);
    cv::Mat frame_paired=frame_rected.clone();
    int armor_num=barpairs.size();
    for(int i=0;i<barpairs.size();i++){
        for(int j=0;j<4;j++){
            cv::circle(frame_paired,barpairs[i].apex[j],4,cv::Scalar(0,0,255),4);
            cv::line(frame_paired,barpairs[i].apex[j],barpairs[i].apex[(j+1)%4],cv::Scalar(255,255,0),2);
        }        
    //    cv::line(frame_paired,barpairs[i].former.rect.center,barpairs[i].latter.rect.center,cv::Scalar(0,255,255),5);
    }
    cv::Mat frame_armored=frame.clone();
    for(int i=0;i<barpairs.size();i++)
    {
        float average_length=(barpairs[i].former.length+barpairs[i].latter.length)/2.0F;
        cv::Point2f dots[4],lable;
        dots[0]={barpairs[i].apex[0].x,barpairs[i].apex[0].y-average_length/2.8F};
        dots[1]={barpairs[i].apex[1].x,barpairs[i].apex[1].y+average_length/2.8F};
        dots[2]={barpairs[i].apex[2].x,barpairs[i].apex[2].y+average_length/2.8F};
        dots[3]={barpairs[i].apex[3].x,barpairs[i].apex[3].y-average_length/2.8F};
        lable={dots[0].x,std::min(dots[0].y,dots[3].y)-(barpairs[i].former.length+barpairs[i].latter.length)/20.0F};
        for(int j=0;j<4;j++){
            cv::circle(frame_armored,dots[j],3,cv::Scalar(0,0,255),2);
            cv::line(frame_armored,dots[j],dots[(j+1)%4],cv::Scalar(0,255,255),2);
        }
        cv::putText(frame_armored,"Detected Armor "+std::to_string(i+1),lable,cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,150,150),2);
    }
    res[0]={frame_armored,false,armor_num};
    res[1]={blurred,false,armor_num};
    res[2]={color_difference,true,armor_num};
    res[3]={binary,true,armor_num};
    res[4]={open_final,true,armor_num};
    res[5]={frame_rected,false,armor_num};
    res[6]={frame_paired,false,armor_num};
    return res[x];
}