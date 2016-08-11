#pragma once
#include "ofxsImageEffect.h"
#include <climits>

namespace openMVG_ofx {
namespace LensCalibration {

/**
 * Plugin Factory declaration
 * @param id
 * @param versionMaj
 * @param versionMin
 */
mDeclarePluginFactory(LensCalibrationPluginFactory, {}, {});


} //namespace LensCalibration
} //namespace openMVG_ofx