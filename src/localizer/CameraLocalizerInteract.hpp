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

class CameraLocalizerInteract : public OFX::OverlayInteract
{
private:
  const CameraLocalizerPlugin* _plugin;
  
public:
  CameraLocalizerInteract(OfxInteractHandle handle, OFX::ImageEffect* effect)
    : OFX::OverlayInteract(handle)
    , _plugin(reinterpret_cast<CameraLocalizerPlugin*>(effect))
  {}

  // overridden function from OFX::Interact to do things
  virtual bool draw(const OFX::DrawArgs &args);
};

class CameraLocalizerOverlayDescriptor : public OFX::DefaultEffectOverlayDescriptor<CameraLocalizerOverlayDescriptor, CameraLocalizerInteract>
{};

}
}
