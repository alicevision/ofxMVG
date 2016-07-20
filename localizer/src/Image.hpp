#pragma once
#include "ofxsImageEffect.h"
#include <cstddef>
#include <vector>

namespace openMVG_ofx
{

namespace Localizer
{

template<typename DataType>
class Image
{
public:

  /**
  * @brief Empty constructor
  */
  Image();

  /**
   * @brief Image with internal buffer constructor
   * @param[in] width
   * @param[in] height
   */
  Image(std::size_t width, std::size_t height, std::size_t nbChannels);

  /**
   * @brief Image with external buffer constructor
   * @param[in,out] imgData
   */
  Image(OFX::Image *imgData);

  /**
   * @brief Copy constructor 
   * @param[in] image
   */
  //Image(const Image &image) = delete;

  /**
   * @brief Destructor
   * Call clean method
   */
  ~Image();

  /**
   * @brief Create image internal buffer
   * @param[in] width
   * @param[in] height
   */
  void createInternalBuffer(std::size_t width, std::size_t height, std::size_t nbChannels);

  /**
   * @brief Create image external buffer
   * @param[in,out] data
   * @param[in] width
   * @param[in] height
   * @param[in] channels
   * @param[in] rowBufferSize
   */
  void setExternalBuffer(DataType *data, std::size_t width, std::size_t height, std::size_t channels, std::size_t rowBufferSize);

  /**
   * @brief Clean image memory
   * Reset all member variables
   * Delete image buffer if has ownership
   */
  void clear();

  /**
   * @brief Set image to zero
   */
  void setZero();
  void setRed();

  /**
   * @brief Multiply each image values by its corresponding entry in another image of the same size
   * @param[in] other - Image
   */
  void multiply(const Image &other);

  /**
   * @brief Multiply images values by a scalar
   * @param[in] other - Scalar
   */
  void multiply(float coefficient);
  
  /**
   * @brief Divide images values by its corresponding entry in another image of the same size
   * @param other
   */
  void divide(const Image &other);
  
  /**
   * @brief copy the image in parameter
   * @param other
   */
  void copyFrom(const Image &other);


  bool hasOwnership() const
  {
    return _hasOwnership;
  }

  DataType* getData()
  {
    return _data;
  }

  DataType* getData() const
  {
    return _data;
  }

  DataType* getPixel(std::size_t x, std::size_t y) const
  {
    return _data + (y * _rowBufferSize) + x * _nbChannels;
  }

  std::size_t getWidth() const
  {
    return _width;
  }

  std::size_t getHeight() const
  {
    return _height;
  }

  std::size_t getNbChannels() const
  {
    return _nbChannels;
  }

  std::size_t getSize() const
  {
    return _size;
  }

  std::size_t getNbPixels() const
  {
    return _nbPixels;
  }

  std::size_t getRowBufferSize() const
  {
    return _rowBufferSize;
  }

  std::size_t getChannelQuantization() const
  {
    return _channelQuantization;
  }

  /**
   * @brief Check if a group of images have the same dimensions
   * @param[in] images
   */
  static void checkSameDimensions(const std::vector<Image> &images);

private:
  OFX::Image *_imgPtr = nullptr;
  DataType *_data = nullptr;
  bool _hasOwnership = 0;
  std::size_t _width = 0;
  std::size_t _height = 0;
  std::size_t _nbChannels = 3;
  std::size_t _size = 0;
  std::size_t _nbPixels = 0;
  std::size_t _rowBufferSize = 0;
  std::size_t _channelQuantization = 1 << 12;
};


} //namespace Localizer

} //namespace openMVG_ofx
