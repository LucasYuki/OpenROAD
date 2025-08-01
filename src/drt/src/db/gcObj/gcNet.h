// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "db/gcObj/gcBlockObject.h"
#include "db/gcObj/gcPin.h"
#include "db/obj/frBlockage.h"
#include "db/obj/frInstBlockage.h"
#include "db/obj/frNet.h"

namespace drt {
class frNet;
class gcNet : public gcBlockObject
{
 public:
  // constructors
  gcNet(const int numLayers)
      : fixedPolygons_(numLayers),
        routePolygons_(numLayers),
        fixedRectangles_(numLayers),
        routeRectangles_(numLayers),
        pins_(numLayers),
        taperedRects_(numLayers),
        nonTaperedRects_(numLayers)
  {
  }
  // setters
  void addPolygon(const Rect& box, frLayerNum layerNum, bool isFixed = false)
  {
    gtl::rectangle_data<frCoord> rect(
        box.xMin(), box.yMin(), box.xMax(), box.yMax());
    using gtl::operators::operator+=;
    if (isFixed) {
      fixedPolygons_[layerNum] += rect;
    } else {
      routePolygons_[layerNum] += rect;
    }
  }
  void addRectangle(const Rect& box, frLayerNum layerNum, bool isFixed = false)
  {
    gtl::rectangle_data<frCoord> rect(
        box.xMin(), box.yMin(), box.xMax(), box.yMax());
    if (isFixed) {
      fixedRectangles_[layerNum].push_back(rect);
    } else {
      routeRectangles_[layerNum].push_back(rect);
    }
  }
  void addPin(const gtl::polygon_90_with_holes_data<frCoord>& shape,
              frLayerNum layerNum)
  {
    auto pin = std::make_unique<gcPin>(shape, layerNum, this);
    pin->setId(pins_[layerNum].size());
    pins_[layerNum].push_back(std::move(pin));
  }
  void addPin(const gtl::rectangle_data<frCoord>& rect, frLayerNum layerNum)
  {
    gtl::polygon_90_with_holes_data<frCoord> shape;
    std::vector<frCoord> coords
        = {gtl::xl(rect), gtl::yl(rect), gtl::xh(rect), gtl::yh(rect)};
    shape.set_compact(coords.begin(), coords.end());
    auto pin = std::make_unique<gcPin>(shape, layerNum, this);
    pin->setId(pins_[layerNum].size());
    pins_[layerNum].push_back(std::move(pin));
  }
  void setOwner(frBlockObject* in) { owner_ = in; }
  void clear()
  {
    auto size = routePolygons_.size();
    routePolygons_.clear();
    routePolygons_.resize(size);
    routeRectangles_.clear();
    routeRectangles_.resize(size);
    taperedRects_.clear();
    taperedRects_.resize(size);
    nonTaperedRects_.clear();
    nonTaperedRects_.resize(size);
    specialSpacingRects_.clear();
    for (auto& layerPins : pins_) {
      layerPins.clear();
    }
  }
  // getters
  const std::vector<gtl::polygon_90_set_data<frCoord>>& getPolygons(
      bool isFixed = false) const
  {
    if (isFixed) {
      return fixedPolygons_;
    }
    return routePolygons_;
  }
  const gtl::polygon_90_set_data<frCoord>& getPolygons(frLayerNum layerNum,
                                                       bool isFixed
                                                       = false) const
  {
    if (isFixed) {
      return fixedPolygons_[layerNum];
    }
    return routePolygons_[layerNum];
  }
  const std::vector<std::vector<gtl::rectangle_data<frCoord>>>& getRectangles(
      bool isFixed = false) const
  {
    if (isFixed) {
      return fixedRectangles_;
    }
    return routeRectangles_;
  }
  const std::vector<gtl::rectangle_data<frCoord>>& getRectangles(
      frLayerNum layerNum,
      bool isFixed = false) const
  {
    if (isFixed) {
      return fixedRectangles_[layerNum];
    }
    return routeRectangles_[layerNum];
  }
  const std::vector<std::vector<std::unique_ptr<gcPin>>>& getPins() const
  {
    return pins_;
  }
  const std::vector<std::unique_ptr<gcPin>>& getPins(frLayerNum layerNum) const
  {
    return pins_[layerNum];
  }
  bool hasOwner() const { return owner_; }
  frBlockObject* getOwner() const { return owner_; }
  bool isBlockage() const
  {
    return hasOwner()
           && (owner_->typeId() == frcInstBlockage
               || owner_->typeId() == frcInst
               || owner_->typeId() == frcBlockage);
  }
  frCoord getDesignRuleWidth() const
  {
    if (hasOwner()) {
      switch (owner_->typeId()) {
        case frcInstBlockage:
          return static_cast<frInstBlockage*>(owner_)
              ->getBlockage()
              ->getDesignRuleWidth();
        case frcBlockage:
          return static_cast<frBlockage*>(owner_)->getDesignRuleWidth();
        default:
          return -1;
      }
    } else {
      return -1;
    }
  }
  // others
  frBlockObjectEnum typeId() const override { return gccNet; }

  frNet* getFrNet() const
  {
    if (owner_->typeId() == frcNet) {
      return static_cast<frNet*>(owner_);
    }
    return nullptr;
  }
  bool isNondefault() const
  {
    return getFrNet() && getFrNet()->getNondefaultRule();
  }
  void addTaperedRect(const Rect& bx, int zIdx)
  {
    taperedRects_[zIdx].push_back(bx);
  }
  const std::vector<Rect>& getTaperedRects(int z) const
  {
    return taperedRects_[z];
  }
  void addNonTaperedRect(const Rect& bx, int zIdx)
  {
    nonTaperedRects_[zIdx].push_back(bx);
  }
  const std::vector<Rect>& getNonTaperedRects(int z) const
  {
    return nonTaperedRects_[z];
  }
  void addSpecialSpcRect(const Rect& bx,
                         frLayerNum lNum,
                         gcPin* pin,
                         gcNet* net)
  {
    std::unique_ptr<gcRect> sp = std::make_unique<gcRect>();
    sp->setLayerNum(lNum);
    sp->addToNet(net);
    sp->addToPin(pin);
    sp->setRect(bx);
    specialSpacingRects_.push_back(std::move(sp));
  }
  const std::vector<std::unique_ptr<gcRect>>& getSpecialSpcRects() const
  {
    return specialSpacingRects_;
  }
  bool hasPolyCornerAt(frCoord x, frCoord y, frLayerNum ln) const
  {
    for (auto& pin : pins_[ln]) {
      for (auto& corners : pin->getPolygonCorners()) {
        for (auto& corner : corners) {
          if (corner->x() == x && corner->y() == y) {
            return true;
          }
        }
      }
    }
    return false;
  }
  gcCorner* getPolyCornerAt(frCoord x, frCoord y, frLayerNum ln) const
  {
    for (auto& pin : pins_[ln]) {
      for (auto& corners : pin->getPolygonCorners()) {
        for (auto& corner : corners) {
          if (corner->x() == x && corner->y() == y) {
            return corner.get();
          }
        }
      }
    }
    return nullptr;
  }

 private:
  std::vector<gtl::polygon_90_set_data<frCoord>>
      fixedPolygons_;  // only routing layer
  std::vector<gtl::polygon_90_set_data<frCoord>>
      routePolygons_;  // only routing layer
  std::vector<std::vector<gtl::rectangle_data<frCoord>>>
      fixedRectangles_;  // only cut layer
  std::vector<std::vector<gtl::rectangle_data<frCoord>>>
      routeRectangles_;  // only cut layer
  std::vector<std::vector<std::unique_ptr<gcPin>>> pins_;
  frBlockObject* owner_{nullptr};
  std::vector<std::vector<Rect>> taperedRects_;     //(only routing layer)
  std::vector<std::vector<Rect>> nonTaperedRects_;  //(only routing layer)
  // A non-tapered rect within a tapered max rectangle still require nondefault
  // spacing. This list hold these rectangles
  std::vector<std::unique_ptr<gcRect>> specialSpacingRects_;

  void init();
};
}  // namespace drt
