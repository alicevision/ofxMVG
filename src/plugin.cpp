#include "ofxsImageEffect.h"
#include "localizer/CameraLocalizerPluginFactory.hpp"
#include "lensCalibration/LensCalibrationPluginFactory.hpp"

/**
 * OFX::Plugin::getPluginIDs
 * @param ids
 */
void OFX::Plugin::getPluginIDs(OFX::PluginFactoryArray &ids)
{
  static openMVG_ofx::Localizer::CameraLocalizerPluginFactory cameraLocalizer("openmvg.cameralocalizer", 1, 0);
  static openMVG_ofx::LensCalibration::LensCalibrationPluginFactory lensCalibration("openmvg.lenscalibration", 1, 0);
  
  ids.push_back(&cameraLocalizer);
  ids.push_back(&lensCalibration);
}
