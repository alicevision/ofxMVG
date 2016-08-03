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

////////////////////////////////////////////////////////////////////////////////
// a dumb interact that just draw's a square you can drag
static const OfxPointD kBoxSize = {20, 20};

class LensCalibrationInteract : public OFX::OverlayInteract
{
  LensCalibrationPlugin* _plugin;
  
protected:
  enum StateEnum {
    eInActive,
    ePoised,
    ePicked
  };

  OfxPointD _position;
  StateEnum _state;

public:
  LensCalibrationInteract(OfxInteractHandle handle, OFX::ImageEffect* effect)
    : OFX::OverlayInteract(handle)
    , _plugin(reinterpret_cast<LensCalibrationPlugin*>(effect))
    , _state(eInActive)
  {
    _position.x = 0;
    _position.y = 0;
  }

  // overridden functions from OFX::Interact to do things
  virtual bool draw(const OFX::DrawArgs &args);
  virtual bool penMotion(const OFX::PenArgs &args);
  virtual bool penDown(const OFX::PenArgs &args);
  virtual bool penUp(const OFX::PenArgs &args);
};

class LensCalibrationOverlayDescriptor : public OFX::DefaultEffectOverlayDescriptor<LensCalibrationOverlayDescriptor, LensCalibrationInteract>
{};

} //namespace LensCalibration
} //namespace openMVG_ofx