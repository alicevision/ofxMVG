#pragma once

#include "CameraLocalizerPluginDefinition.hpp"
#include "../common/Image.hpp"

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
  openMVG::localization::LocalizationResult localizationResult;
  std::vector<openMVG::features::SIOPointFeature> extractedFeatures;
  openMVG::Mat undistortedPt2D;
  mutable std::mutex mutex;
  
  FrameData()
  {}
  
  /**
   * @brief Copy constructor
   * @param[in] other
   */
  FrameData(const FrameData &other)
  {
    localizationResult = other.localizationResult;
    extractedFeatures = other.extractedFeatures;
    undistortedPt2D = other.undistortedPt2D;
  }
  
  /**
   * @brief Assignment copy operator
   * @param[in] other
   * @return 
   */
  FrameData& operator=(const FrameData &other) 
  {
    localizationResult = other.localizationResult;
    extractedFeatures = other.extractedFeatures;
    undistortedPt2D = other.undistortedPt2D;
    //the mutex can't be copied
    
    return *this;
  }
  
  bool isLocalized() const
  {
    return localizationResult.isValid();
  }
  
};


//ProcessData structure for localization process
struct LocalizerProcessData
{
  std::vector<openMVG::cameras::Pinhole_Intrinsic_Radial_K3> queryIntrinsics;
  std::unique_ptr<openMVG::localization::LocalizerParameters> param;
  std::unique_ptr<openMVG::localization::ILocalizer> localizer;
  
  void extractFeatures(
      const std::map< std::size_t, openMVG::image::Image<unsigned char> > &mapImageGray,
      std::vector< std::unique_ptr<openMVG::features::Regions> > &vecQueryRegions) const;

  bool localize(std::unique_ptr<openMVG::features::Regions>& queryRegions,
                const std::pair<std::size_t, std::size_t>& queryImageSize,
                bool hasIntrinsics,  
                openMVG::cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                openMVG::localization::LocalizationResult &localizationResult);

  bool localizeRig(const std::vector<std::unique_ptr<openMVG::features::Regions> > &vecQueryRegions,
                                        const std::vector<std::pair<std::size_t, std::size_t> > &vecQueryImageSize,
                                        std::vector<openMVG::cameras::Pinhole_Intrinsic_Radial_K3 > &vecQueryIntrinsics,
                                        const std::vector<openMVG::geometry::Pose3 > &vecQuerySubPoses,
                                        openMVG::geometry::Pose3 &rigPose,
                                        std::vector<openMVG::localization::LocalizationResult> &vecLocResults);
  
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
void convertRGB32ToGRAY8(const Common::Image<float>& inputImage, openMVG::image::Image<unsigned char> &outputImage);

/**
 * @brief convert a 32 bits float grayscale image to an (unsigned char) 8 bits image
 * @param inputImage
 * @param outputImage
 */
void convertGGG32ToGRAY8(const Common::Image<float>& inputImage, openMVG::image::Image<unsigned char> &outputImage);

/**
 * @brief convert a gray (unsigned char) 8 bits image to a 32 bits float image
 * @param inputImage
 * @param outputImage
 */
void convertGRAY8ToRGB32(openMVG::image::Image<unsigned char> &inputImage, const Common::Image<float>& outputImage);


} //namespace Localizer
} //namespace openMVG_ofx






