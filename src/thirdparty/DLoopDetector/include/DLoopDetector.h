#ifndef __D_T_LOOP_DETECTOR__
#define __D_T_LOOP_DETECTOR__

/// Loop detector for sequences of monocular images
namespace DLoopDetector
{
}

#include "DBoW2/include/DBoW2.h"
#include "TemplatedLoopDetector.h"
#include "DBoW2/include/FORB.h"
#include "DBoW2/include/FBrief.h"

/// SURF64 Loop Detector
typedef DLoopDetector::TemplatedLoopDetector
  <FORB::TDescriptor, FORB> OrbLoopDetector;

/// BRIEF Loop Detector
typedef DLoopDetector::TemplatedLoopDetector
  <FBrief::TDescriptor, FBrief> BriefLoopDetector;

#endif

