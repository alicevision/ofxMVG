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
#include "CameraLocalizerPlugin.hpp"
namespace openMVG_ofx {
namespace Localizer {

class CameraLocalizerPlugin;

////////////////////////////////////////////////////////////////////////////////
// a dumb interact that just draw's a square you can drag
static const OfxPointD kBoxSize = {20, 20};

class CameraLocalizerInteract : public OFX::OverlayInteract
{
private:
  const CameraLocalizerPlugin* _plugin;
  
protected:
  enum StateEnum {
    eInActive,
    ePoised,
    ePicked
  };

  OfxPointD _position;
  StateEnum _state;

public:
  CameraLocalizerInteract(OfxInteractHandle handle, OFX::ImageEffect* effect)
    : OFX::OverlayInteract(handle)
    , _plugin(reinterpret_cast<CameraLocalizerPlugin*>(effect))
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

class CameraLocalizerOverlayDescriptor : public OFX::DefaultEffectOverlayDescriptor<CameraLocalizerOverlayDescriptor, CameraLocalizerInteract>
{};

}
}
