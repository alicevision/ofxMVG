#pragma once

#include "CameraLocalizerPluginDefinition.hpp"
#include "Image.hpp"

#include <openMVG/localization/ILocalizer.hpp>
#include <openMVG/localization/VoctreeLocalizer.hpp>
#if HAVE_CCTAG
#include <openMVG/localization/CCTagLocalizer.hpp>
#endif
#include <openMVG/rig/Rig.hpp>
#include <openMVG/image/image_io.hpp>
#include <openMVG/dataio/FeedProvider.hpp>

#include <memory>
#include <mutex>


namespace openMVG_ofx {
namespace Localizer {

//FrameData structure for cache result
struct FrameData
{
  bool localized;
  openMVG::localization::LocalizationResult localizationResult;
  std::vector<openMVG::features::SIOPointFeature> extractedFeatures;
  openMVG::Mat undistortedPt2D;
  mutable std::mutex mutex;
  
  /**
   * @brief Assignment copy operator
   * @param other
   * @return 
   */
  FrameData& operator=(FrameData const &other) 
  {
    localized = other.localized;
    localizationResult = other.localizationResult;
    extractedFeatures = other.extractedFeatures;
    undistortedPt2D = other.undistortedPt2D;
    //the mutex can't be copied
    
    return *this;
  }
};


//ProcessData structure for localization process
struct LocalizerProcessData
{
  std::vector<openMVG::cameras::Pinhole_Intrinsic_Radial_K3> queryIntrinsics;
  std::unique_ptr<openMVG::localization::LocalizerParameters> param;
  std::unique_ptr<openMVG::localization::ILocalizer> localizer;
  // rig::Rig rig;
  
  void extractFeatures(
      const openMVG::image::Image<unsigned char> &imageGray,
      std::unique_ptr<openMVG::features::Regions>& outQueryRegions) const;

  bool localize(std::unique_ptr<openMVG::features::Regions>& queryRegions,
                const std::pair<std::size_t, std::size_t>& queryImageSize,
                bool hasIntrinsics,  
                openMVG::cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                openMVG::localization::LocalizationResult &localizationResult);

  bool localizeRig(const std::vector<openMVG::image::Image<unsigned char> > & vec_imageGray,
                    std::vector<openMVG::cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
                    const std::vector<openMVG::geometry::Pose3 > &vec_subPoses,
                    openMVG::geometry::Pose3 rigPose);
  
  /**
   * @brief get openMVG features preset enum from Plugin display choice enum
   * @param preset
   * @return 
   */
  static openMVG::features::EDESCRIBER_PRESET getDescriberPreset(EParamFeaturesPreset preset);
  
  /**
   * @brief get openMVG algorithm enum from Plugin display choice enum 
   * @param algorithm
   * @return 
   */
  static openMVG::localization::VoctreeLocalizer::Algorithm getAlgorithm(EParamAlgorithm algorithm);
  
  /**
   * @brief get openMVG robust estimator enum from Plugin matching display choice enum 
   * @param estimator
   * @return 
   */
  static openMVG::robust::EROBUST_ESTIMATOR getMatchingEstimator(EParamEstimatorMatching estimator);
  
  /**
   *  @brief get openMVG robust estimator enum from Plugin resection display choice enum  
   * @param estimator
   * @return 
   */
  static openMVG::robust::EROBUST_ESTIMATOR getResectionEstimator(EParamEstimatorResection estimator);
  
  /**
   * 
   * @param model
   * @return 
   */
  static EParamLensDistortionMode getLensDistortionModelFromEnum(openMVG::cameras::EINTRINSIC model);
};

/**
 * @brief get the digit id of a param name given
 * @param paramName
 * @return 
 */
std::size_t getParamInputId(const std::string& paramName);

/**
 * @brief Set openMVG pose values into OFX parameters
 * @param pose
 * @param time
 * @param cameraOutputTranslate
 * @param cameraOutputRotate
 * @param cameraOutputScale
 */
void setPoseToParamsAtTime(
    const openMVG::geometry::Pose3 & pose,
    const double time,
    OFX::Double3DParam *cameraOutputTranslate,
    OFX::Double3DParam *cameraOutputRotate,
    OFX::Double3DParam *cameraOutputScale);

/**
 * @brief Set openMVG intrinsics values into OFX parameters
 * @param intrinsics
 * @param time
 * @param sensorWidth
 * @param cameraOutputFocalLength
 * @param cameraOutputOpticalCenter
 */
void setIntrinsicsToParamsAtTime(
    const openMVG::cameras::Pinhole_Intrinsic& intrinsics,
    const double time,
    const double sensorWidth,
    OFX::DoubleParam *cameraOutputFocalLength,
    OFX::Double2DParam *cameraOutputOpticalCenter);

/**
 * @brief Set openMVG errors values into OFX parameters
 * @param localizationResult
 * @param time
 * @param outputErrorMean
 * @param outputErrorMin
 * @param outputErrorMax
 */
void setStatToParamsAtTime(
    const openMVG::localization::LocalizationResult &localizationResult,
    const std::vector<openMVG::features::SIOPointFeature>& features,
    const double time,
    OFX::DoubleParam *outputStatErrorMean,
    OFX::DoubleParam *outputStatErrorMin,
    OFX::DoubleParam *outputStatErrorMax,
    OFX::DoubleParam *outputStatNbMatchedImages,
    OFX::DoubleParam *outputStatNbDetectedFeatures,
    OFX::DoubleParam *outputStatNbMatchedFeatures,
    OFX::DoubleParam *outputStatNbInlierFeatures);

/**
 * @brief convert a 32 bits float image to a gray (unsigned char) 8 bits image
 * @param inputImage
 * @param outputImage
 */
void convertRGB32ToGRAY8(const Image<float>& inputImage, openMVG::image::Image<unsigned char> &outputImage);

/**
 * @brief convert a 32 bits float grayscale image to an (unsigned char) 8 bits image
 * @param inputImage
 * @param outputImage
 */
void convertGGG32ToGRAY8(const Image<float>& inputImage, openMVG::image::Image<unsigned char> &outputImage);

/**
 * @brief convert a gray (unsigned char) 8 bits image to a 32 bits float image
 * @param inputImage
 * @param outputImage
 */
void convertGRAY8ToRGB32(openMVG::image::Image<unsigned char> &inputImage, const Image<float>& outputImage);


} //namespace Localizer
} //namespace openMVG_ofx






