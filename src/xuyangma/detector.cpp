#include "detector.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
struct BrightPart{
    std::vector<cv::Point> contour;
    cv::Rect box;
    cv::Point2f center;
    double area;
    double lScore;
};
struct FitLine{
    cv::Point2f point;
    cv::Point2f direction;
    bool valid=false;
};
// point distance
static double pointDistance(const cv::Point2f& a,const cv::Point2f& b){
    double dx=a.x-b.x;
    double dy=a.y-b.y;
    return std::sqrt(dx*dx+dy*dy);
}
// 2D cross product
static double cross2D(const cv::Point2f& a,const cv::Point2f& b){
    return a.x*b.y-a.y*b.x;
}
// marker area
static double quadArea(const Quad& q){
    double area=0.0;
    for(int i=0;i<4;i++){
        const cv::Point2f& a=q[i];
        const cv::Point2f& b=q[(i+1)%4];
        area+=a.x*b.y-a.y*b.x;
    }
    return std::abs(area)*0.5;
}
// marker center
static cv::Point2f quadCenter(const Quad& q){
    cv::Point2f center(0,0);
    for(const auto& p:q){
        center+=p;
    }
    return center*0.25f;
}
// average side length
static double averageSide(const Quad& q){
    double total=0.0;
    for(int i=0;i<4;i++){
        total+=pointDistance(q[i],q[(i+1)%4]);
    }
    return total/4.0;
}
// check convex quad
static bool isConvexQuad(const Quad& q){
    double first=0.0;
    for(int i=0;i<4;i++){
        cv::Point2f a=q[(i+1)%4]-q[i];
        cv::Point2f b=q[(i+2)%4]-q[(i+1)%4];
        double value=cross2D(a,b);
        if(std::abs(value)<1e-3){
            return false;
        }
        if(i==0){
            first=value;
        }else if(value*first<0){
            return false;
        }
    }
    return true;
}
// check candidate geometry
static bool validQuad(const Quad& q,const cv::Size& frameSize){
    if(!isConvexQuad(q)){
        return false;
    }
    double area=quadArea(q);
    double frameArea=static_cast<double>(frameSize.width)*frameSize.height;
    if(area/frameArea<0.0004||area/frameArea>0.55){
        return false;
    }
    double minSide=1e9;
    double maxSide=0.0;
    for(int i=0;i<4;i++){
        double side=pointDistance(q[i],q[(i+1)%4]);
        minSide=std::min(minSide,side);
        maxSide=std::max(maxSide,side);
    }
    if(minSide<15||maxSide/minSide>4.0){
        return false;
    }
    double margin=0.12*std::max(frameSize.width,frameSize.height);
    for(const auto& p:q){
        if(p.x<-margin||p.y<-margin||p.x>frameSize.width+margin||p.y>frameSize.height+margin){
            return false;
        }
    }
    return true;
}
// L shape model from 30 22 8 dimensions
static std::vector<cv::Point> makeLModel(){
    return{
        cv::Point(0,0),
        cv::Point(30,0),
        cv::Point(30,8),
        cv::Point(8,8),
        cv::Point(8,30),
        cv::Point(0,30)
    };
}
// get white bright mask
static cv::Mat getBrightMask(const cv::Mat& frame){
    cv::Mat hsv;
    cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv,channels);
    cv::Mat saturation=channels[1];
    cv::Mat value=channels[2];
    cv::Mat temp;
    double otsu=cv::threshold(value,temp,0,255,cv::THRESH_BINARY|cv::THRESH_OTSU);
    double valueThreshold=std::max(130.0,std::min(220.0,otsu));
    cv::Mat bright;
    cv::Mat lowSaturation;
    cv::threshold(value,bright,valueThreshold,255,cv::THRESH_BINARY);
    cv::threshold(saturation,lowSaturation,145,255,cv::THRESH_BINARY_INV);
    cv::Mat mask;
    cv::bitwise_and(bright,lowSaturation,mask);
    cv::Mat kernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::morphologyEx(mask,mask,cv::MORPH_OPEN,kernel);
    cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,kernel);
    return mask;
}
// score one L contour
static double scoreLContour(const std::vector<cv::Point>& contour){
    double area=cv::contourArea(contour);
    if(area<=0){
        return 0.0;
    }
    cv::Rect box=cv::boundingRect(contour);
    if(box.width<4||box.height<4){
        return 0.0;
    }
    double aspect=static_cast<double>(std::max(box.width,box.height))/std::min(box.width,box.height);
    if(aspect>2.2){
        return 0.0;
    }
    double fill=area/static_cast<double>(box.area());
    if(fill<0.18||fill>0.82){
        return 0.0;
    }
    std::vector<cv::Point> model=makeLModel();
    double shapeDistance=cv::matchShapes(contour,model,cv::CONTOURS_MATCH_I1,0.0);
    double shapeScore=1.0/(1.0+2.0*shapeDistance);
    double expectedFill=(30.0*8.0+22.0*8.0)/(30.0*30.0);
    double fillScore=1.0-std::min(std::abs(fill-expectedFill)/0.35,1.0);
    return 0.75*shapeScore+0.25*fillScore;
}
// extract possible L lights
static std::vector<BrightPart> extractBrightParts(const cv::Mat& mask){
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(),contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    std::vector<BrightPart> parts;
    double imageArea=static_cast<double>(mask.rows)*mask.cols;
    for(const auto& contour:contours){
        double area=cv::contourArea(contour);
        double ratio=area/imageArea;
        if(ratio<0.000005||ratio>0.04){
            continue;
        }
        cv::Rect box=cv::boundingRect(contour);
        if(box.width<4||box.height<4){
            continue;
        }
        double aspect=static_cast<double>(std::max(box.width,box.height))/std::min(box.width,box.height);
        if(aspect>2.5){
            continue;
        }
        double lScore=scoreLContour(contour);
        if(lScore<0.30){
            continue;
        }
        BrightPart part;
        part.contour=contour;
        part.box=box;
        part.center=cv::Point2f(box.x+box.width/2.0f,box.y+box.height/2.0f);
        part.area=area;
        part.lScore=lScore;
        parts.push_back(part);
    }
    return parts;
}
// transform model points with affine transform
static std::vector<cv::Point2f> transformPoints(const cv::Mat& transform,const std::vector<cv::Point2f>& points){
    std::vector<cv::Point2f> result;
    cv::transform(points,result,transform);
    return result;
}
// warp any quadrilateral
static cv::Mat warpQuad(const cv::Mat& image,const Quad& quad,int size){
    std::vector<cv::Point2f> src(quad.begin(),quad.end());
    std::vector<cv::Point2f> dst={
        cv::Point2f(0,0),
        cv::Point2f(size-1,0),
        cv::Point2f(size-1,size-1),
        cv::Point2f(0,size-1)
    };
    cv::Mat transform=cv::getPerspectiveTransform(src,dst);
    cv::Mat result;
    cv::warpPerspective(image,result,transform,cv::Size(size,size),cv::INTER_NEAREST);
    return result;
}
// score A pattern
static double scoreAPatch(const cv::Mat& patch){
    if(patch.empty()){
        return 0.0;
    }
    double brightRatio=static_cast<double>(cv::countNonZero(patch))/patch.total();
    if(brightRatio<0.04||brightRatio>0.60){
        return 0.0;
    }
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(patch.clone(),contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    int count=0;
    double totalArea=0.0;
    double largestArea=0.0;
    for(const auto& contour:contours){
        double area=cv::contourArea(contour);
        if(area>patch.total()*0.004){
            count++;
            totalArea+=area;
            largestArea=std::max(largestArea,area);
        }
    }
    if(count<2||count>5||totalArea<=0){
        return 0.0;
    }
    double countScore=1.0-std::min(std::abs(count-3)/3.0,1.0);
    double ratioScore=1.0-std::min(std::abs(brightRatio-0.22)/0.30,1.0);
    double largestRatio=largestArea/totalArea;
    double separationScore=1.0-std::min(largestRatio,1.0);
    return 0.45*countScore+0.35*ratioScore+0.20*separationScore;
}
// verify predicted A region
static double verifyARegion(const cv::Mat& mask,const cv::Mat& affine){
    std::vector<cv::Point2f> model={
        cv::Point2f(42,2),
        cv::Point2f(78,2),
        cv::Point2f(78,38),
        cv::Point2f(42,38)
    };
    std::vector<cv::Point2f> imagePoints=transformPoints(affine,model);
    Quad quad={imagePoints[0],imagePoints[1],imagePoints[2],imagePoints[3]};
    cv::Mat patch=warpQuad(mask,quad,96);
    return scoreAPatch(patch);
}
// largest L score in one ROI
static double largestLScore(const cv::Mat& roi){
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roi.clone(),contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    double best=0.0;
    for(const auto& contour:contours){
        if(cv::contourArea(contour)<roi.total()*0.01){
            continue;
        }
        best=std::max(best,scoreLContour(contour));
    }
    return best;
}
// get region using 80x80 model coordinates
static cv::Rect modelRect(int size,double x0,double y0,double x1,double y1){
    int left=static_cast<int>(x0/80.0*size);
    int top=static_cast<int>(y0/80.0*size);
    int right=static_cast<int>(x1/80.0*size);
    int bottom=static_cast<int>(y1/80.0*size);
    left=std::max(0,std::min(size-1,left));
    top=std::max(0,std::min(size-1,top));
    right=std::max(left+1,std::min(size,right));
    bottom=std::max(top+1,std::min(size,bottom));
    return cv::Rect(left,top,right-left,bottom-top);
}
// verify 3B+A after perspective correction
static double verifyMarkerModel(const cv::Mat& mask,const Quad& quad){
    const int size=320;
    cv::Mat normalized=warpQuad(mask,quad,size);
    if(normalized.empty()){
        return 0.0;
    }
    cv::Rect tlRect=modelRect(size,4,4,36,36);
    cv::Rect blRect=modelRect(size,4,44,36,76);
    cv::Rect brRect=modelRect(size,44,44,76,76);
    cv::Rect aRect=modelRect(size,42,2,78,38);
    double tlScore=largestLScore(normalized(tlRect));
    double blScore=largestLScore(normalized(blRect));
    double brScore=largestLScore(normalized(brRect));
    double aScore=scoreAPatch(normalized(aRect));
    if(tlScore<0.25||blScore<0.25||brScore<0.25||aScore<0.20){
        return 0.0;
    }
    return (tlScore+blScore+brScore+aScore)/4.0;
}
// check layout of TL BL RB lights
static bool validThreeLLayout(const BrightPart& tl,const BrightPart& bl,const BrightPart& br,double& geometryScore){
    cv::Point2f up=tl.center-bl.center;
    cv::Point2f right=br.center-bl.center;
    double vertical=std::sqrt(up.x*up.x+up.y*up.y);
    double horizontal=std::sqrt(right.x*right.x+right.y*right.y);
    if(vertical<20||horizontal<20){
        return false;
    }
    double ratio=std::max(vertical,horizontal)/std::min(vertical,horizontal);
    if(ratio>2.5){
        return false;
    }
    double cosine=std::abs((up.x*right.x+up.y*right.y)/(vertical*horizontal));
    if(cosine>0.70){
        return false;
    }
    if(cross2D(up,right)<=0){
        return false;
    }
    double minArea=std::min(tl.area,std::min(bl.area,br.area));
    double maxArea=std::max(tl.area,std::max(bl.area,br.area));
    if(minArea<=0||maxArea/minArea>3.0){
        return false;
    }
    double angleScore=1.0-std::min(cosine/0.70,1.0);
    double ratioScore=1.0-std::min((ratio-1.0)/1.5,1.0);
    geometryScore=(angleScore+ratioScore)/2.0;
    return true;
}
// build affine transform using known lamp centers
static cv::Mat makeMarkerAffine(const BrightPart& tl,const BrightPart& bl,const BrightPart& br){
    cv::Point2f model[3]={
        cv::Point2f(20,20),
        cv::Point2f(20,60),
        cv::Point2f(60,60)
    };
    cv::Point2f image[3]={
        tl.center,
        bl.center,
        br.center
    };
    return cv::getAffineTransform(model,image);
}
// estimate outer 80x80 board
static Quad estimateOuterQuad(const cv::Mat& affine){
    std::vector<cv::Point2f> model={
        cv::Point2f(0,0),
        cv::Point2f(80,0),
        cv::Point2f(80,80),
        cv::Point2f(0,80)
    };
    std::vector<cv::Point2f> image=transformPoints(affine,model);
    return{image[0],image[1],image[2],image[3]};
}
// fit one real outer edge
static FitLine fitOuterEdge(const cv::Mat& gx,const cv::Mat& gy,const cv::Point2f& start,const cv::Point2f& end){
    FitLine result;
    cv::Point2f tangent=end-start;
    double length=std::sqrt(tangent.x*tangent.x+tangent.y*tangent.y);
    if(length<20){
        return result;
    }
    tangent.x/=static_cast<float>(length);
    tangent.y/=static_cast<float>(length);
    cv::Point2f normal(-tangent.y,tangent.x);
    int searchRadius=std::max(4,std::min(22,static_cast<int>(0.055*length)));
    const int samples=28;
    std::vector<cv::Point2f> edgePoints;
    for(int i=0;i<samples;i++){
        double u=(i+1.0)/(samples+1.0);
        cv::Point2f base=start+(end-start)*static_cast<float>(u);
        double bestScore=0.0;
        cv::Point2f bestPoint;
        for(int k=-searchRadius;k<=searchRadius;k++){
            cv::Point2f p=base+normal*static_cast<float>(k);
            int x=cvRound(p.x);
            int y=cvRound(p.y);
            if(x<1||y<1||x>=gx.cols-1||y>=gx.rows-1){
                continue;
            }
            float dx=gx.at<float>(y,x);
            float dy=gy.at<float>(y,x);
            double directional=std::abs(dx*normal.x+dy*normal.y);
            double score=directional/(1.0+0.08*std::abs(k));
            if(score>bestScore){
                bestScore=score;
                bestPoint=p;
            }
        }
        if(bestScore>25.0){
            edgePoints.push_back(bestPoint);
        }
    }
    if(edgePoints.size()<10){
        return result;
    }
    cv::Vec4f line;
    cv::fitLine(edgePoints,line,cv::DIST_L2,0,0.01,0.01);
    cv::Point2f direction(line[0],line[1]);
    double directionLength=std::sqrt(direction.x*direction.x+direction.y*direction.y);
    if(directionLength<=0){
        return result;
    }
    direction.x/=static_cast<float>(directionLength);
    direction.y/=static_cast<float>(directionLength);
    double alignment=std::abs(direction.x*tangent.x+direction.y*tangent.y);
    if(alignment<0.85){
        return result;
    }
    result.point=cv::Point2f(line[2],line[3]);
    result.direction=direction;
    result.valid=true;
    return result;
}
// line intersection
static bool lineIntersection(const FitLine& a,const FitLine& b,cv::Point2f& result){
    if(!a.valid||!b.valid){
        return false;
    }
    double denominator=cross2D(a.direction,b.direction);
    if(std::abs(denominator)<1e-5){
        return false;
    }
    cv::Point2f diff=b.point-a.point;
    double t=cross2D(diff,b.direction)/denominator;
    result=a.point+a.direction*static_cast<float>(t);
    return true;
}
// refine board edges
static Quad refineOuterQuad(const cv::Mat& frame,const Quad& rough){
    cv::Mat gray;
    cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray,gray,cv::Size(5,5),0);
    cv::Mat gx;
    cv::Mat gy;
    cv::Sobel(gray,gx,CV_32F,1,0,3);
    cv::Sobel(gray,gy,CV_32F,0,1,3);
    FitLine top=fitOuterEdge(gx,gy,rough[0],rough[1]);
    FitLine right=fitOuterEdge(gx,gy,rough[1],rough[2]);
    FitLine bottom=fitOuterEdge(gx,gy,rough[3],rough[2]);
    FitLine left=fitOuterEdge(gx,gy,rough[0],rough[3]);
    if(!top.valid||!right.valid||!bottom.valid||!left.valid){
        return rough;
    }
    Quad refined;
    if(!lineIntersection(top,left,refined[0])){
        return rough;
    }
    if(!lineIntersection(top,right,refined[1])){
        return rough;
    }
    if(!lineIntersection(bottom,right,refined[2])){
        return rough;
    }
    if(!lineIntersection(bottom,left,refined[3])){
        return rough;
    }
    if(!validQuad(refined,frame.size())){
        return rough;
    }
    double movement=0.0;
    for(int i=0;i<4;i++){
        movement+=pointDistance(refined[i],rough[i]);
    }
    movement/=4.0;
    if(movement>0.15*averageSide(rough)){
        return rough;
    }
    return refined;
}
// detect marker
bool detectMarker(const cv::Mat& frame,Quad& bestCorners){
    if(frame.empty()){
        return false;
    }
    cv::Mat mask=getBrightMask(frame);
    std::vector<BrightPart> parts=extractBrightParts(mask);
    if(parts.size()<3){
        return false;
    }
    double bestScore=-1.0;
    double bestModelScore=0.0;
    Quad bestRough;
    int n=static_cast<int>(parts.size());
    for(int i=0;i<n;i++){
        for(int j=0;j<n;j++){
            for(int k=0;k<n;k++){
                if(i==j||i==k||j==k){
                    continue;
                }
                double geometryScore=0.0;
                if(!validThreeLLayout(parts[i],parts[j],parts[k],geometryScore)){
                    continue;
                }
                cv::Mat affine=makeMarkerAffine(parts[i],parts[j],parts[k]);
                Quad rough=estimateOuterQuad(affine);
                if(!validQuad(rough,frame.size())){
                    continue;
                }
                double aScore=verifyARegion(mask,affine);
                if(aScore<0.20){
                    continue;
                }
                double modelScore=verifyMarkerModel(mask,rough);
                if(modelScore<0.20){
                    continue;
                }
                double lScore=(parts[i].lScore+parts[j].lScore+parts[k].lScore)/3.0;
                double totalScore=0.40*lScore+0.25*aScore+0.25*modelScore+0.10*geometryScore;
                if(totalScore>bestScore){
                    bestScore=totalScore;
                    bestModelScore=modelScore;
                    bestRough=rough;
                }
            }
        }
    }
    if(bestScore<0.40){
        return false;
    }
    Quad refined=refineOuterQuad(frame,bestRough);
    double refinedModelScore=verifyMarkerModel(mask,refined);
    if(refinedModelScore>=bestModelScore*0.65){
        bestCorners=refined;
    }else{
        bestCorners=bestRough;
    }
    return true;
}
// temporal consistency
bool isTemporallyConsistent(const Quad& current,const Quad& previous,const cv::Size& frameSize){
    double previousArea=quadArea(previous);
    double currentArea=quadArea(current);
    if(previousArea<=1||currentArea<=1){
        return false;
    }
    double areaRatio=currentArea/previousArea;
    if(areaRatio>3.0||areaRatio<1.0/3.0){
        return false;
    }
    cv::Point2f currentCenter=quadCenter(current);
    cv::Point2f previousCenter=quadCenter(previous);
    double movement=pointDistance(currentCenter,previousCenter);
    double targetScale=std::sqrt(previousArea);
    if(targetScale<1){
        targetScale=1;
    }
    double normalizedMovement=movement/targetScale;
    if(normalizedMovement>1.5){
        return false;
    }
    return true;
}
// smooth corners
void smoothCorners(Quad& current,const Quad& previous,const cv::Size& frameSize){
    double movement=0.0;
    for(int i=0;i<4;i++){
        movement+=pointDistance(current[i],previous[i]);
    }
    movement/=4.0;
    double threshold=0.12*std::max(frameSize.width,frameSize.height);
    if(movement>threshold){
        return;
    }
    const double alpha=0.60;
    for(int i=0;i<4;i++){
        current[i].x=static_cast<float>(alpha*previous[i].x+(1.0-alpha)*current[i].x);
        current[i].y=static_cast<float>(alpha*previous[i].y+(1.0-alpha)*current[i].y);
    }
}
// draw result
void drawDetection(cv::Mat& frame,const Quad& corners){
    const char* labels[4]={"LT","RT","RB","LB"};
    std::vector<cv::Point> polygon;
    for(const auto& point:corners){
        polygon.emplace_back(cvRound(point.x),cvRound(point.y));
    }
    for(int i=0;i<4;i++){
        cv::line(frame,polygon[i],polygon[(i+1)%4],cv::Scalar(0,255,0),3);
        cv::circle(frame,polygon[i],7,cv::Scalar(0,0,255),-1);
        cv::putText(frame,labels[i],polygon[i]+cv::Point(8,-8),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(0,255,255),2);
    }
    cv::Rect boundingBox=cv::boundingRect(polygon);
    cv::rectangle(frame,boundingBox,cv::Scalar(255,0,0),2);
    cv::putText(frame,"DETECTED",cv::Point(30,50),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0,255,0),2);
}