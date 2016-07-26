#include "CameraLocalizer.hpp"

namespace openMVG_ofx {
namespace Localizer {

bool LocalizerProcessData::localize(openMVG::image::Image<unsigned char> &imageGray, 
                                    bool hasIntrinsics,  
                                    openMVG::cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                                    openMVG::localization::LocalizationResult &localizationResult)
{
  return this->localizer->localize(imageGray, 
                this->param.get(),
                hasIntrinsics,
                queryIntrinsics,
                localizationResult,
                ""); //for image debug
}
                                    
bool LocalizerProcessData::localizeRig(const std::vector<openMVG::image::Image<unsigned char> > & vec_imageGray,
                                        std::vector<openMVG::cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
                                        const std::vector<openMVG::geometry::Pose3 > &vec_subPoses,
                                        openMVG::geometry::Pose3 rigPose)
{
  return this->localizer->localizeRig(vec_imageGray,
                                      this->param.get(),
                                      vec_queryIntrinsics,
                                      vec_subPoses,
                                      rigPose);
}

  
openMVG::features::EDESCRIBER_PRESET LocalizerProcessData::getDescriberPreset(EParamFeaturesPreset preset)
{
  switch(preset)
  {
    case eParamFeaturesPresetLow : return openMVG::features::EDESCRIBER_PRESET::LOW_PRESET; break;
    case eParamFeaturesPresetMedium : return openMVG::features::EDESCRIBER_PRESET::MEDIUM_PRESET; break;
    case eParamFeaturesPresetNormal : return openMVG::features::EDESCRIBER_PRESET::NORMAL_PRESET; break;
    case eParamFeaturesPresetHigh : return openMVG::features::EDESCRIBER_PRESET::HIGH_PRESET; break;
    case eParamFeaturesPresetUltra : return openMVG::features::EDESCRIBER_PRESET::ULTRA_PRESET; break;
    
    default : throw std::invalid_argument("Unrecognized Features Preset : " + std::to_string(preset));
  }
}

openMVG::localization::VoctreeLocalizer::Algorithm LocalizerProcessData::getAlgorithm(EParamAlgorithm algorithm)
{
  switch(algorithm)
  {
    case eParamAlgorithmFirstBest : return openMVG::localization::VoctreeLocalizer::Algorithm::FirstBest; break;
    case eParamAlgorithmBestResult : return openMVG::localization::VoctreeLocalizer::Algorithm::BestResult; break;
    case eParamAlgorithmAllResults : return openMVG::localization::VoctreeLocalizer::Algorithm::AllResults; break;
    case eParamAlgorithmCluster : return openMVG::localization::VoctreeLocalizer::Algorithm::Cluster; break;
    
    default : throw std::invalid_argument("Unrecognized Algorithm : " + std::to_string(algorithm));
  }
}


void setPoseToParamsAtTime(
    const openMVG::geometry::Pose3 & pose,
    const double time,
    OFX::Double3DParam *cameraOutputTranslate,
    OFX::Double3DParam *cameraOutputRotate,
    OFX::Double3DParam *cameraOutputScale)
{
  // Convert camera orientation
  openMVG::Mat3 fixOrientation;
  fixOrientation << 1.,  0., 0.,
                    0., -1., 0.,
                    0.,  0., -1.;

  openMVG::Vec3 translation = pose.center();
  cameraOutputTranslate->setValueAtTime(time, translation(0), translation(1), translation(2));

// ZYX: 0 1 2
// Decompose the rotation in ZXY angles
//  std::ifstream f("/tmp/fixRotation.txt");
  int a=0, b=1, c=2;
//  f >> a;
//  f >> b;
//  f >> c;
  openMVG::Vec3 rotationAngles = (fixOrientation * pose.rotation()).transpose().eulerAngles(a, b, c);
  cameraOutputRotate->setValueAtTime(time, rotationAngles(0), rotationAngles(1), rotationAngles(2));
  
  cameraOutputScale->setValueAtTime(time, 1, 1, 1);
}

void setIntrinsicsToParamsAtTime(
    const openMVG::cameras::Pinhole_Intrinsic& intrinsics,
    const double time,
    const double sensorWidth,
    OFX::DoubleParam *cameraOutputFocalLength,
    OFX::Double2DParam *cameraOutputOpticalCenter)
{
  const std::vector<double> params = intrinsics.getParams();
  cameraOutputFocalLength->setValueAtTime(time, params[0] * sensorWidth / double(intrinsics.w()) );
  cameraOutputOpticalCenter->setValueAtTime(time, params[1], params[2]);
//  for(std::size_t i = 3; params.size(); ++i)
//  {
//    
//  }
}


void convertRGB32ToGRAY8(const Image<float>& inputImage, openMVG::image::Image<unsigned char> &outputImage)
{
  for(unsigned int y = 0; y < outputImage.Height(); ++y)
  {
    for(unsigned int x = 0; x < outputImage.Width(); ++x)
    {
      const float* rgbPtr = inputImage.getPixel(x, y);
      const float gray = openMVG::image::Rgb2Gray(rgbPtr[0], rgbPtr[1], rgbPtr[2]);
      outputImage(y, x) = (unsigned char)(gray * 255.f);
    }
  }
}

void convertGGG32ToGRAY8(const Image<float>& inputImage, openMVG::image::Image<unsigned char> &outputImage)
{
  for(unsigned int y = 0; y < outputImage.Height(); ++y)
  {
    for(unsigned int x = 0; x < outputImage.Width(); ++x)
    {
      const float* rgbPtr = inputImage.getPixel(x, y);
      const float gray = rgbPtr[0];
      outputImage(y, x) = (unsigned char)(gray * 255.f);
    }
  }
}

void convertGRAY8ToRGB32(openMVG::image::Image<unsigned char> &inputImage, const Image<float>& outputImage)
{
  for(unsigned int y = 0; y < inputImage.Height(); ++y)
  {
    for(unsigned int x = 0; x < inputImage.Width(); ++x)
    {
      const float gray = float(inputImage(y, x)) / 255.f;
      float* rgbPtr = outputImage.getPixel(x, y);
      rgbPtr[0] = gray;
      rgbPtr[1] = gray;
      rgbPtr[2] = gray;
    }
  }
}
  

} //namespace Localizer
} //namespace openMVG_ofx