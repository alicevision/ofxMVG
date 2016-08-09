#include "CameraLocalizerInteract.hpp"
#include "stb_easy_font.h"
#include <cmath>

namespace openMVG_ofx {
namespace Localizer {

using namespace OFX;

template <class T> inline T
Minimum(T a, T b) {    return (a < b) ? a : b;}

template <class T> inline T
Absolute(T a) { return (a < 0) ? -a : a;}


void drawCircle(GLfloat x, GLfloat y, GLfloat radius)
{
	const int nbLine = 50; 
  
	GLfloat coefficient = (2.0f * M_PI) / nbLine;
	
	glBegin(GL_LINE_LOOP);
  for(int i = 0; i <= nbLine; ++i) 
  { 
    glVertex2f(
        x + (radius * cos(i *  coefficient)), 
        y + (radius * sin(i * coefficient))
    );
  }
	glEnd();
}

bool CameraLocalizerInteract::draw(const OFX::DrawArgs &args)
{
  //Check if current frame has cache
  if(!_plugin->hasFrameDataCache(args.time))
  {
    return false;
  }
  
  //Get all overlay parameters
  bool drawDetectedFeatures = _plugin->hasOverlayDetectedFeatures();
  bool drawMatchedFeatures = _plugin->hasOverlayMatchedFeatures();
  bool drawResectionFeatures = _plugin->hasOverlayResectionFeatures();
  bool drawReprojectionError = _plugin->hasOverlayReprojectionError();
  bool drawReconstructionVisibility = _plugin->hasOverlayReconstructionVisibility();
  bool drawFeaturesId = _plugin->hasOverlayFeaturesId();
  bool drawFeaturesScaleOrientation = _plugin->hasOverlayFeaturesScaleOrientation();
  bool drawTracks = _plugin->hasOverlayTracks();

  //Check if drawing is necessary
  if(!drawDetectedFeatures &&
      !drawMatchedFeatures &&
      !drawResectionFeatures &&
      !drawTracks)
  {
    return false;
  }
  
  //Get frame cache and check mutex
  const FrameData& frameCachedData = _plugin->getFrameDataCache(args.time);
  //std::lock_guard<std::mutex> guard(frameCachedData.mutex);
  
  openMVG::cameras::Pinhole_Intrinsic_Radial_K3 intrinsics;

  if(frameCachedData.localized)
  {
    intrinsics = frameCachedData.localizationResult.getIntrinsics();
  }
  else
  {
    // init intrinsics from input params
  }

  const std::vector<openMVG::features::SIOPointFeature>& detectedFeatures = frameCachedData.extractedFeatures;
      
  // Display all detected features
  if(drawDetectedFeatures)
  {
    glColor3f(1.f, 0.f, 0.f);
    glPointSize(2);
    glBegin(GL_POINTS);
    
    for(std::size_t i = 0; i < detectedFeatures.size(); ++i)
    {
      openMVG::Vec2 origPoint = detectedFeatures[i].coords().cast<double>();
      openMVG::Vec2 point = intrinsics.get_ud_pixel(origPoint);
      
      // Vertical flip
      glVertex2f(point(0), intrinsics.h() - point(1));
    }
    glEnd();
    
    if(drawFeaturesScaleOrientation)
    {
      for(std::size_t i = 0; i < detectedFeatures.size(); ++i)
      {
        double radius = _plugin->getOverlayScaleOrientationRadius();
        
        openMVG::Vec2 origPoint = detectedFeatures[i].coords().cast<double>();
        openMVG::Vec2 center = intrinsics.get_ud_pixel(origPoint);
        
        // Vertical flip
        center(1) = intrinsics.h() - center(1);
        
        openMVG::Vec2 origOrientation = detectedFeatures[i].getScaledOrientationVector().cast<double>();
        origOrientation(1) = - origOrientation(1);
        openMVG::Vec2 orientation = center + origOrientation * radius;
        
        drawCircle(center(0), center(1), detectedFeatures[i].scale() * radius);
        
        glBegin(GL_LINES);
        glVertex2f(center(0), center(1));
        glVertex2f(orientation(0), orientation(1));
        glEnd();
      }
    }
  }

  // Next options only for localized frame
  if(!frameCachedData.localized)
  {
    return true;
  }
  
  const openMVG::localization::LocalizationResult& localizationResult = frameCachedData.localizationResult;
  const std::size_t nbViews = _plugin->getLocalizerSfMData().views.size();

  openMVG::Mat pt2dDetected = frameCachedData.undistortedPt2D; 
  openMVG::Mat pt2dProjected;

  pt2dProjected.resize(2, pt2dDetected.size());
  
  const openMVG::Mat& pt3d = localizationResult.getPt3D();
  
  //  openMVG::Mat2X residuals = intrinsics.residuals(localizationResult.getPose(), pt3d, pt2d);
  //  openMVG::Mat2X residuals = localizationResult.computeResiduals();
  //  std::cout << "CameraLocalizerInteract::draw: nb points: " << pt2d.cols() << std::endl;

  for(std::size_t i = 0; i < pt2dDetected.cols(); ++i)
  {
    // Vertical flip: OpenFX is bottomUp and openMVG is topDown
    pt2dDetected(1, i) = intrinsics.h() - pt2dDetected(1, i);

    // Project 3D point in 2D without distortion
    openMVG::Vec2 projected = intrinsics.project(localizationResult.getPose(), pt3d.col(i), false);
    pt2dProjected(0, i) = projected(0);
    
    // Vertical flip: OpenFX is bottomUp and openMVG is topDown
    pt2dProjected(1, i) = intrinsics.h() - projected(1);
  }

  
  // Matched points
  if(drawMatchedFeatures)
  {
    glColor3f(1.f, 0.5f, 0.5f);
    glPointSize(4);

    for(std::size_t i = 0; i < pt2dDetected.cols(); ++i)
    {
      openMVG::IndexT pt3dIndex = localizationResult.getIndMatch3D2D()[i].first;
      openMVG::IndexT pt2dIndex = localizationResult.getIndMatch3D2D()[i].second;
      
      const openMVG::Vec2& pointDetected = pt2dDetected.col(i);
      const openMVG::Vec2& pointProjected = pt2dProjected.col(i);
      
      openMVG::Vec2 point = pt2dDetected.col(i);
      
      if(drawReconstructionVisibility)
      {
        float obs = _plugin->getLocalizerSfMData().structure.find(pt3dIndex)->second.obs.size() / (float) nbViews;
        
        glColor3f(0.f,obs, 1 - obs);
      }
      
      if(drawReprojectionError)
      {
        glLineWidth(1);
        glBegin(GL_LINES);
        glVertex2f(pointDetected(0), pointDetected(1));
        glVertex2f(pointProjected(0), pointProjected(1));
        glEnd();
      }
      
      if(drawFeaturesId)
      {
       std::string idStr = std::to_string(pt3dIndex);
       stb_print_string(pointDetected(0) + 2, pointDetected(1) + 2, idStr);
      }
      
      if(drawFeaturesScaleOrientation)
      {
        double radius = _plugin->getOverlayScaleOrientationRadius();

        openMVG::Vec2 origOrientation = detectedFeatures[pt2dIndex].getScaledOrientationVector().cast<double>();
        origOrientation(1) = - origOrientation(1);
        openMVG::Vec2 orientation = pointDetected + origOrientation * radius;

        drawCircle(pointDetected(0), pointDetected(1), detectedFeatures[pt2dIndex].scale() * radius);

        glBegin(GL_LINES);
        glVertex2f(pointDetected(0), pointDetected(1));
        glVertex2f(orientation(0), orientation(1));
        glEnd();
      }
      
      glBegin(GL_POINTS);
      glVertex2f(pointDetected(0), pointDetected(1));
      glEnd();
    }
  }

  // Resectioning points inliers    
  if(drawResectionFeatures)
  {
    glColor3f(.5f, 1.f, .5f);
    glPointSize(4);

    for(std::size_t i: localizationResult.getInliers())
    {
      openMVG::IndexT pt3dIndex = localizationResult.getIndMatch3D2D()[i].first;
      openMVG::IndexT pt2dIndex = localizationResult.getIndMatch3D2D()[i].second;
      
      const openMVG::Vec2& pointDetected = pt2dDetected.col(i);
      const openMVG::Vec2& pointProjected = pt2dProjected.col(i);
      
      if(drawReconstructionVisibility)
      {
        openMVG::IndexT pt2dIndex = localizationResult.getIndMatch3D2D()[i].second;
        float obs = _plugin->getLocalizerSfMData().structure.find(pt3dIndex)->second.obs.size() / (float) nbViews;
        
        glColor3f(0.f,obs, 1 - obs);
      }
      
      if(drawReprojectionError)
      {
        glLineWidth(1);
        glBegin(GL_LINES);
        glVertex2f(pointDetected(0), pointDetected(1));
        glVertex2f(pointProjected(0), pointProjected(1));
        glEnd();
      }
      
      if(drawFeaturesId)
      {
       std::string idStr = std::to_string(pt3dIndex);
       stb_print_string(pointDetected(0) + 2, pointDetected(1) + 2, idStr);
      }
      
      if(drawFeaturesScaleOrientation)
      {
        double radius = _plugin->getOverlayScaleOrientationRadius();

        openMVG::Vec2 origOrientation = detectedFeatures[pt2dIndex].getScaledOrientationVector().cast<double>();
        origOrientation(1) = - origOrientation(1);
        openMVG::Vec2 orientation = pointDetected + origOrientation * radius;

        drawCircle(pointDetected(0), pointDetected(1), detectedFeatures[pt2dIndex].scale() * radius);

        glBegin(GL_LINES);
        glVertex2f(pointDetected(0), pointDetected(1));
        glVertex2f(orientation(0), orientation(1));
        glEnd();
      }
      
      glBegin(GL_POINTS);
      glVertex2f(pointDetected(0), pointDetected(1));
      glEnd();
    }
  }


  if(drawTracks)
  {
    int nbTracksWindowSize = _plugin->getOverlayTracksWindowSize();
    int firstTime = args.time - nbTracksWindowSize;
    int lastTime = args.time + nbTracksWindowSize;

    std::map< openMVG::IndexT, std::map<OfxTime, openMVG::Vec2> > pt2dTracking;

    for(int time = firstTime; time < lastTime; ++time)
    {
      std::cout << "[Overlay] Tracking Time : " << time << std::endl;

      if(!_plugin->hasFrameDataCache(time))
        continue;
      
      const FrameData& frameAtTimeCachedData = _plugin->getFrameDataCache(time);

      // Do not lock the current time: args.time
      std::mutex doNotLock;
      std::lock_guard<std::mutex> guardAtTime(time != args.time ? frameAtTimeCachedData.mutex : doNotLock);  
      
      if(!frameAtTimeCachedData.localized)
        continue;
      
      const openMVG::localization::LocalizationResult& localizationResultAtTime = frameAtTimeCachedData.localizationResult;
      const openMVG::Mat& pt2d = frameAtTimeCachedData.undistortedPt2D; 

      const std::size_t minIndex = 0;
      const std::size_t maxIndex = std::min(minIndex + 10000, localizationResultAtTime.getInliers().size());

      for(std::size_t index = minIndex; index < maxIndex; ++index)
      {
        const std::size_t i = localizationResultAtTime.getInliers()[index];
        
        // Vertical flip: OpenFX is bottomUp and openMVG is topDown
        openMVG::Vec2 point = pt2d.col(i);//
        point(1) = intrinsics.h() - point(1);

        openMVG::IndexT id = localizationResultAtTime.getIndMatch3D2D()[i].first;

        auto it = pt2dTracking.find(id);

        if (it != pt2dTracking.end())
        {
          it->second.emplace(time, point);
        }
        else
        {
          std::map<OfxTime, openMVG::Vec2> points;
          points[time] = point;
          pt2dTracking.emplace(id, points);
        }
      }
      
    }

    //draw tracks
    glLineWidth(1);
    for(auto &pt2dIt : pt2dTracking)
    {
      OfxTime firstTime = pt2dIt.second.begin()->first;
      OfxTime lastTime = pt2dIt.second.rbegin()->first;
      if(firstTime > args.time || lastTime < args.time)
        continue;
      std::array<float, 3> lineColor;
      std::array<float, 3> pointColor;
      if(pt2dIt.second.find(args.time) != pt2dIt.second.end())
      {
        // If the track is visible in current frame
        lineColor = {.6f, .3f, 0.f};
        pointColor = {1.f, .6f, .2f};
      }
      else
      {
        // If the track is visible before and after the current frame but not
        // visible on the current frame
        lineColor = {.35f, .2f, 0.f};
        pointColor = {.6f, .3f, 0.f};
      }

      //draw lines
      if(pt2dIt.second.size() > 1)
      {
        glBegin(GL_LINES);
        auto pointAtTimeA = pt2dIt.second.cbegin();
        auto pointAtTimeB = pt2dIt.second.cbegin();
        for(++pointAtTimeB;
            pointAtTimeB != pt2dIt.second.cend();
            ++pointAtTimeA, ++pointAtTimeB)
        {
          if(pointAtTimeB->first - pointAtTimeA->first == 1)
            glColor3f(lineColor[0], lineColor[1], lineColor[2]);
          else
            glColor3f(1.f, .2f, .2f);
          glVertex2f(pointAtTimeA->second(0), pointAtTimeA->second(1));
          glVertex2f(pointAtTimeB->second(0), pointAtTimeB->second(1));
        }
        glEnd();
      }

      //draw points
      glColor3f(pointColor[0], pointColor[1], pointColor[2]);
      glPointSize(2);
      glBegin(GL_POINTS);
      for(auto &point : pt2dIt.second)
        glVertex2f(point.second(0), point.second(1));
      glEnd();
    }


    glEnd();
  }
  
  return true;
}

// overridden functions from OFX::Interact to do things
bool CameraLocalizerInteract::penMotion(const OFX::PenArgs &args)
{
//  // figure the size of the box in cannonical coords
//  float dx = (float)(kBoxSize.x * args.pixelScale.x);
//  float dy = (float)(kBoxSize.y * args.pixelScale.y);
//
//  // pen position is in cannonical coords
//  OfxPointD penPos = args.penPosition;
//
//  switch(_state) {
//    case eInActive: 
//    case ePoised: 
//    {
//      // are we in the box, become 'poised'
//      StateEnum newState;
//      penPos.x -= _position.x;
//      penPos.y -= _position.y;
//      if(Absolute(penPos.x) < dx &&
//        Absolute(penPos.y) < dy) {
//          newState = ePoised;
//      }
//      else {
//        newState = eInActive;
//      }
//
//      if(_state != newState) {
//        // we have a new state
//        _state = newState;
//
//        // and force an overlay redraw
//        _effect->redrawOverlays();
//      }
//    }
//    break;
//    case ePicked:
//    {
//      // move our position
//      _position = penPos;
//
//      // and force an overlay redraw
//      _effect->redrawOverlays();
//    }
//    break;
//  }
//
//  // we have trapped it only if the mouse ain't over it or we are actively dragging
//  return _state != eInActive;
  return false;
}

bool CameraLocalizerInteract::penDown(const OFX::PenArgs &args)
{
//  // this will refigure the state
//  penMotion(args);
//
//  // if poised means we were over it when the pen went down, so pick it
//  if(_state == ePoised) {
//    // we are now picked
//    _state = ePicked;
//
//    // move our position
//    _position = args.penPosition;
//
//    // and request a redraw just incase
//    _effect->redrawOverlays();
//  }
//
//  return _state == ePicked;
  return false;
}

bool CameraLocalizerInteract::penUp(const OFX::PenArgs &args)
{
//  if(_state == ePicked) {
//    // reset to poised for a moment
//    _state = ePoised;
//
//    // this will refigure the state
//    penMotion(args);
//
//    // and redraw for good measure
//    _effect->redrawOverlays();
//
//    // we did trap it
//    return true;
//  }

  // we didn't trap it
  return false;
}

}
}
