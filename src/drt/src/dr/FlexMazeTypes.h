/* Authors: Lutong Wang and Bangqi Xu */
/*
 * Copyright (c) 2019, The Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "frBaseTypes.h"

namespace drt {
class FlexMazeIdx
{
 public:
  FlexMazeIdx() = default;
  FlexMazeIdx(frMIdx xIn, frMIdx yIn, frMIdx zIn)
      : xIdx_(xIn), yIdx_(yIn), zIdx_(zIn)
  {
  }
  // getters
  frMIdx x() const { return xIdx_; }
  frMIdx y() const { return yIdx_; }
  frMIdx z() const { return zIdx_; }
  bool empty() const { return (xIdx_ == -1 && yIdx_ == -1 && zIdx_ == -1); }
  // setters
  void set(frMIdx xIn, frMIdx yIn, frMIdx zIn)
  {
    xIdx_ = xIn;
    yIdx_ = yIn;
    zIdx_ = zIn;
  }
  void setX(frMIdx xIn) { xIdx_ = xIn; }
  void setY(frMIdx yIn) { yIdx_ = yIn; }
  void setZ(frMIdx zIn) { zIdx_ = zIn; }
  void set(const FlexMazeIdx& in)
  {
    xIdx_ = in.x();
    yIdx_ = in.y();
    zIdx_ = in.z();
  }
  // others
  bool operator<(const FlexMazeIdx& rhs) const
  {
    if (xIdx_ != rhs.x()) {
      return xIdx_ < rhs.x();
    }
    if (yIdx_ != rhs.y()) {
      return yIdx_ < rhs.y();
    }
    return zIdx_ < rhs.z();
  }
  bool operator==(const FlexMazeIdx& rhs) const
  {
    return (xIdx_ == rhs.xIdx_ && yIdx_ == rhs.yIdx_ && zIdx_ == rhs.zIdx_);
  }

  friend std::ostream& operator<<(std::ostream& os, const FlexMazeIdx& mIdx)
  {
    os << "(" << mIdx.x() << ", " << mIdx.y() << ", " << mIdx.z() << ")";
    return os;
  }

 private:
  frMIdx xIdx_ = -1;
  frMIdx yIdx_ = -1;
  frMIdx zIdx_ = -1;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    (ar) & xIdx_;
    (ar) & yIdx_;
    (ar) & zIdx_;
  }
  friend class boost::serialization::access;
};
}  // namespace drt
