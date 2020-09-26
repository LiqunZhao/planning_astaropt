#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>

#include <stdexcept>

namespace lanelet {

/**
 * @brief Thrown when matching is not possible.
 */
class MatchingError : public LaneletError {
  using LaneletError::LaneletError;
};

}  // namespace lanelet
