#include "LensCalibrationInteract.hpp"

#ifdef HAVE_CCTAG
#include <cctag/CCTag.hpp>
#endif

#include "../common/stb_easy_font.h"


namespace openMVG_ofx {
namespace LensCalibration {

inline void drawEllipse(float cx, float cy, float rx, float ry, float angle, int num_segments = 50)
{
  float theta = 2 * 3.1415926 / float(num_segments); 
  float c = std::cos(theta); // precalculate the sine and cosine
  float s = std::sin(theta);
  float t;

  float x = angle;
  float y = 0; 

  glBegin(GL_LINE_LOOP); 
  for(int ii = 0; ii < num_segments; ii++) 
  {
    //apply radius and offset
    glVertex2f(x * rx + cx, y * ry + cy);//output vertex 

    //apply the rotation matrix
    t = x;
    x = c * x - s * y;
    y = s * t + c * y;
  } 
  glEnd(); 
}

bool LensCalibrationInteract::draw(const OFX::DrawArgs &args)
{
  if(_plugin->_checkerPerFrame.count(args.time) == 0)
    return false;
  
  const LensCalibrationPlugin::CheckerPoints& checker = _plugin->_checkerPerFrame.at(args.time);
  const std::size_t height = _plugin->_inputImageSize->getValue().y;
  
  glLineWidth(1);
  glBegin(GL_LINE_STRIP);
  for(std::size_t i = 0; i < checker._detectedPoints.size(); ++i)
  {
    float ratio = i / float(checker._detectedPoints.size());
    glColor3f(0.0, ratio, 1.0 - ratio);
    glVertex2f(checker._detectedPoints[i].x, height - checker._detectedPoints[i].y);
  }
  glEnd();
  
#ifdef HAVE_CCTAG
  if(_plugin->_cctagsPerFrame.count(args.time) != 0)
  {
    boost::ptr_list<cctag::ICCTag>::const_iterator cctagIt = _plugin->_cctagsPerFrame.at(args.time).begin();
    for(std::size_t i = 0; i < checker._detectedPoints.size(); ++i, ++cctagIt)
    {
      float ratio = i / float(checker._detectedPoints.size());
      glColor3f(0.0, ratio, 1.0 - ratio);
      const auto& ellipse = dynamic_cast<const cctag::CCTag&>(*cctagIt).rescaledOuterEllipse();
      
      drawEllipse(checker._detectedPoints[i].x, height - checker._detectedPoints[i].y,
                  ellipse.a() * 0.5, ellipse.b() * 0.5,
                  ellipse.angle());
    }
  }
#endif

  glPointSize(5.0);
  glBegin(GL_POINTS);
  for(std::size_t i = 0; i < checker._detectedPoints.size(); ++i)
  {
    float ratio = i / float(checker._detectedPoints.size());
    glColor3f(0.0, ratio, 1.0 - ratio);
    
    glVertex2f(checker._detectedPoints[i].x, height - checker._detectedPoints[i].y);
  }
  glEnd();

  for(std::size_t i = 0; i < checker._detectedPoints.size(); ++i)
  {
    float ratio = i / float(checker._detectedPoints.size());
    glColor3f(0.0, ratio, 1.0 - ratio);
    const std::string idStr = std::to_string(checker._pointsId[i]);
    stb_print_string(checker._detectedPoints[i].x + 2, height - checker._detectedPoints[i].y + 2, idStr);
  }
  return true;
}

bool LensCalibrationInteract::penMotion(const OFX::PenArgs &args)
{
  return false;
}

bool LensCalibrationInteract::penDown(const OFX::PenArgs &args)
{
  return false;
}

bool LensCalibrationInteract::penUp(const OFX::PenArgs &args)
{
  return false;
}

} //namespace LensCalibration
} //namespace openMVG_ofx
