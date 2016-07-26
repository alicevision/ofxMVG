#pragma once
#include "ofxsImageEffect.h"

namespace openMVG_ofx {
namespace Localizer {

/**
 * Plugin Factory declaration
 * @param id
 * @param versionMaj
 * @param versionMin
 */
mDeclarePluginFactory(CameraLocalizerPluginFactory, {}, {});


} //namespace Localizer
} //namespace openMVG_ofx