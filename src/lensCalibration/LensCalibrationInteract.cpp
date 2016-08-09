#include "LensCalibrationInteract.hpp"

namespace openMVG_ofx {
namespace LensCalibration {

using namespace OFX;

bool LensCalibrationInteract::draw(const OFX::DrawArgs &args)
{
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