#include "CameraLocalizerInteract.hpp"

#include "CameraLocalizerPlugin.hpp"

#include "../common/stb_easy_font.h"

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

  std::array<float, 3> colorDetected = {.6f, .5f, .5f};
  std::array<float, 3> colorMatched = {1.f, 0.5f, 0.5f};
  std::array<float, 3> colorTrackMatchHole = {.7f, .0f, .7f};
  std::array<float, 3> colorResection = {.5f, 1.f, .5f};

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
    glColor3f(colorDetected[0], colorDetected[1], colorDetected[2]);
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


  if(drawTracks && (drawMatchedFeatures || drawResectionFeatures))
  {
    int nbTracksWindowSize = _plugin->getOverlayTracksWindowSize();
    int firstTime = args.time - nbTracksWindowSize;
    int lastTime = args.time + nbTracksWindowSize;

    std::map< openMVG::IndexT, std::map<OfxTime, std::pair<openMVG::Vec2, bool> > > pt2dTracking;

    for(int time = firstTime; time < lastTime; ++time)
    {
      // Check if the frame at time has data in cache
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
      
      std::vector<bool> isInlier(pt2d.cols(), false);
      for(std::size_t i: localizationResultAtTime.getInliers())
      {
        isInlier[i] = true;
      }

      for(std::size_t i = 0; i < pt2d.cols(); ++i)
      {
        // Vertical flip: OpenFX is bottomUp and openMVG is topDown
        openMVG::Vec2 point = pt2d.col(i);
        point(1) = intrinsics.h() - point(1);

        openMVG::IndexT id = localizationResultAtTime.getIndMatch3D2D()[i].first;

        auto it = pt2dTracking.find(id);

        if (it != pt2dTracking.end())
        {
          // add the current point to the it's id map
          it->second.emplace(time, std::pair<openMVG::Vec2, bool>(point, isInlier[i]) );
        }
        else
        {
          // create point id map and add it
          std::map<OfxTime, std::pair<openMVG::Vec2, bool> > points;
          points[time] = std::pair<openMVG::Vec2, bool>(point, isInlier[i]);
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
      
      if(!drawMatchedFeatures)
      {
        // determine the first/last times without outliers
        firstTime = kOfxFlagInfiniteMax;
        lastTime = -kOfxFlagInfiniteMax;
        for(auto it = pt2dIt.second.begin(); it != pt2dIt.second.end(); ++it)
        {
          // if it's an inlier
          if(it->second.second)
          {
            firstTime = it->first;
            break;
          }
        }
        for(auto it = pt2dIt.second.rbegin(); it != pt2dIt.second.rend(); ++it)
        {
          // if it's an inlier
          if(it->second.second)
          {
            lastTime = it->first;
            break;
          }
        }
      }
      
      if(firstTime > args.time || lastTime < args.time)
        continue;
      
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
          bool segmentWithValidResection =
              (pointAtTimeA->second.second == true) &&
              (pointAtTimeB->second.second == true);
          
          if(!drawMatchedFeatures && !segmentWithValidResection)
            // Skip the segment if we only draw the inliers and the sement is not a resectioning inlier
            continue;
          
          if(pointAtTimeB->first - pointAtTimeA->first != 1)
            // Non-contiguous features in the track, which mean that this track is not matched at all between these frames.
            glColor3f(colorTrackMatchHole[0], colorTrackMatchHole[1], colorTrackMatchHole[2]);
          else if(!drawResectionFeatures || !segmentWithValidResection)
            // The 2 features of the track are contiguous but at least one of them is not a resectioning inlier
            glColor3f(colorMatched[0], colorMatched[1], colorMatched[2]);
          else
            // Display resection inlier with color if segmentWithValidResection and drawResectionFeatures
            glColor3f(colorResection[0], colorResection[1], colorResection[2]);
          
          glVertex2f(pointAtTimeA->second.first(0), pointAtTimeA->second.first(1));
          glVertex2f(pointAtTimeB->second.first(0), pointAtTimeB->second.first(1));
        }
        glEnd();
      }

      //draw points
      glPointSize(2);
      glBegin(GL_POINTS);
      for(auto &point : pt2dIt.second)
      {
        if(drawResectionFeatures && (point.second.second == true) ) //is ResectionFeatures
        {
          glColor3f(colorResection[0], colorResection[1], colorResection[2]);
          glVertex2f(point.second.first(0), point.second.first(1));
        }
        else if(drawMatchedFeatures)
        {
          glColor3f(colorMatched[0], colorMatched[1], colorMatched[2]);
          glVertex2f(point.second.first(0), point.second.first(1));
        }
      }
      glEnd();
    }
  }
  
  
  // Matched points
  if(drawMatchedFeatures)
  {
    glColor3f(colorMatched[0], colorMatched[1], colorMatched[2]);
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
    glColor3f(colorResection[0], colorResection[1], colorResection[2]);
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
  return true;
}

}
}
