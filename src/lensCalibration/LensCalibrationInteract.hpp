#pragma once

#ifdef _WINDOWS
#include <windows.h>
#endif
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include "ofxsImageEffect.h"
#include "ofxsInteract.h"
#include "LensCalibrationPlugin.hpp"

namespace openMVG_ofx {
namespace LensCalibration {

class LensCalibrationInteract : public OFX::OverlayInteract
{
  LensCalibrationPlugin* _plugin;

public:
  LensCalibrationInteract(OfxInteractHandle handle, OFX::ImageEffect* effect)
    : OFX::OverlayInteract(handle)
    , _plugin(reinterpret_cast<LensCalibrationPlugin*>(effect))
  {
  }

  // overridden functions from OFX::Interact to do things
  bool draw(const OFX::DrawArgs &args);
  bool penMotion(const OFX::PenArgs &args);
  bool penDown(const OFX::PenArgs &args);
  bool penUp(const OFX::PenArgs &args);
};

class LensCalibrationOverlayDescriptor : public OFX::DefaultEffectOverlayDescriptor<LensCalibrationOverlayDescriptor, LensCalibrationInteract>
{};

} //namespace LensCalibration
} //namespace openMVG_ofx