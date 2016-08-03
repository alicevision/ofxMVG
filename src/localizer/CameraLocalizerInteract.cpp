#include "CameraLocalizerInteract.hpp"

#include "CameraLocalizerPlugin.hpp"

#include "../common/stb_easy_font.h"

namespace openMVG_ofx {
namespace Localizer {

using namespace OFX;

template <class T> inline T
Minimum(T a, T b) {    return (a < b) ? a : b;}

template <class T> inline T
Absolute(T a) { return (a < 0) ? -a : a;}


bool CameraLocalizerInteract::draw(const OFX::DrawArgs &args)
{
  if(!_plugin->displayOverlay())
    return false;

  openMVG::cameras::Pinhole_Intrinsic_Radial_K3 intrinsics;
  

  if(_plugin->hasCachedLocalizationResults(args.time))
  {
    const openMVG::localization::LocalizationResult& localizationResult = _plugin->getCachedLocalizationResults(args.time);
    intrinsics = localizationResult.getIntrinsics();
  }
  else
  {
    // init intrinsics from input params
  }

  // Display all detected features
  if(_plugin->hasCachedFeatures(args.time))
  {
    // Detected points
    glColor3f(1.f, 0.5f, 0.5f);
    glPointSize(2);
    glBegin(GL_POINTS);
    const std::vector<openMVG::features::SIOPointFeature>& detectedFeatures = _plugin->getCachedFeatures(args.time);
    for(std::size_t i = 0; i < detectedFeatures.size(); ++i)
    {
      openMVG::Vec2 origPoint = detectedFeatures[i].coords().cast<double>();
      openMVG::Vec2 point = intrinsics.get_ud_pixel(origPoint);
      // Vertical flip
      glVertex2f(point(0), intrinsics.h() - point(1));
    }
    glEnd();
  }

  // Display localization results
  if(_plugin->hasCachedLocalizationResults(args.time))
  {
    const openMVG::localization::LocalizationResult& localizationResult = _plugin->getCachedLocalizationResults(args.time);

    openMVG::Mat pt2dDetected = localizationResult.retrieveUndistortedPt2D();
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
    glColor3f(1.f, 0.5f, 0.5f);
    glPointSize(4);
    glBegin(GL_POINTS);
    for(std::size_t i = 0; i < pt2dDetected.cols(); ++i)
    {
      openMVG::Vec2 point = pt2dDetected.col(i);
      glVertex2f(point(0), point(1));
    }
    glEnd();
    glLineWidth(1);
    glBegin(GL_LINES);
    for(std::size_t i = 0; i < pt2dDetected.cols(); ++i)
    {
      openMVG::Vec2 pointDetected = pt2dDetected.col(i);
      openMVG::Vec2 pointProjected = pt2dProjected.col(i);
      glVertex2f(pointDetected(0), pointDetected(1));
      glVertex2f(pointProjected(0), pointProjected(1));
    }
    glEnd();

    // Resectioning points inliers
    glColor3f(.5f, 1.f, .5f);

    glPointSize(4);
    glBegin(GL_POINTS);
    for(std::size_t i: localizationResult.getInliers())
    {
      openMVG::Vec2 point = pt2dDetected.col(i);
      glVertex2f(point(0), point(1));
    }
    glEnd();
    glLineWidth(1);
    glBegin(GL_LINES);
    for(std::size_t i: localizationResult.getInliers())
    {
      openMVG::Vec2 pointDetected = pt2dDetected.col(i);
      openMVG::Vec2 pointProjected = pt2dProjected.col(i);
      glVertex2f(pointDetected(0), pointDetected(1));
      glVertex2f(pointProjected(0), pointProjected(1));
    }
    glEnd();

    glColor3f(0.f, .5f, 0.f);
    for(std::size_t i: localizationResult.getInliers())
    {
      openMVG::Vec2 point = pt2dProjected.col(i);
      openMVG::IndexT id = localizationResult.getIndMatch3D2D()[i].first;
      std::string idStr = std::to_string(id);

      stb_print_string(point(0) + 2, point(1) + 2, idStr);
    }

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
