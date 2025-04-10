// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "dbTechLayerSpacingRule.h"

#include <spdlog/fmt/ostr.h>

#include "dbDatabase.h"
#include "dbTable.h"
#include "dbTable.hpp"
#include "dbTech.h"
#include "dbTechLayer.h"
#include "odb/db.h"
#include "odb/lefout.h"

namespace odb {

template class dbTable<_dbTechLayerSpacingRule>;
template class dbTable<_dbTechV55InfluenceEntry>;

void _dbTechLayerSpacingRule::collectMemInfo(MemInfo& info)
{
  info.cnt++;
  info.size += sizeof(*this);
}

void _dbTechV55InfluenceEntry::collectMemInfo(MemInfo& info)
{
  info.cnt++;
  info.size += sizeof(*this);
}

bool _dbTechLayerSpacingRule::operator==(
    const _dbTechLayerSpacingRule& rhs) const
{
  if (_flags._rule != rhs._flags._rule) {
    return false;
  }

  if (_flags._except_same_pgnet != rhs._flags._except_same_pgnet) {
    return false;
  }

  if (_flags._cut_stacking != rhs._flags._cut_stacking) {
    return false;
  }

  if (_flags._cut_center_to_center != rhs._flags._cut_center_to_center) {
    return false;
  }

  if (_flags._cut_same_net != rhs._flags._cut_same_net) {
    return false;
  }

  if (_flags._cut_parallel_overlap != rhs._flags._cut_parallel_overlap) {
    return false;
  }

  if (_spacing != rhs._spacing) {
    return false;
  }

  if (_length_or_influence != rhs._length_or_influence) {
    return false;
  }

  if (_r1min != rhs._r1min) {
    return false;
  }

  if (_r1max != rhs._r1max) {
    return false;
  }

  if (_r2min != rhs._r2min) {
    return false;
  }

  if (_r2max != rhs._r2max) {
    return false;
  }

  if (_cut_area != rhs._cut_area) {
    return false;
  }

  if (_layer != rhs._layer) {
    return false;
  }

  if (_cut_layer_below != rhs._cut_layer_below) {
    return false;
  }

  return true;
}

bool _dbTechV55InfluenceEntry::operator==(
    const _dbTechV55InfluenceEntry& rhs) const
{
  if (_width != rhs._width) {
    return false;
  }

  if (_within != rhs._within) {
    return false;
  }

  if (_spacing != rhs._spacing) {
    return false;
  }

  return true;
}

////////////////////////////////////////////////////////////////////
//
// _dbTechLayerSpacingRule - Methods
// NOTE: Streaming methods work with pointers.
//
////////////////////////////////////////////////////////////////////

dbOStream& operator<<(dbOStream& stream, const _dbTechLayerSpacingRule& rule)
{
  uint* bit_field = (uint*) &rule._flags;
  stream << *bit_field;
  stream << rule._spacing;
  stream << rule._length_or_influence;
  stream << rule._r1min;
  stream << rule._r1max;
  stream << rule._r2min;
  stream << rule._r2max;
  stream << rule._cut_area;
  stream << rule._layer;
  stream << rule._cut_layer_below;
  return stream;
}

dbIStream& operator>>(dbIStream& stream, _dbTechLayerSpacingRule& rule)
{
  uint* bit_field = (uint*) &rule._flags;
  stream >> *bit_field;
  stream >> rule._spacing;
  stream >> rule._length_or_influence;
  stream >> rule._r1min;
  stream >> rule._r1max;
  stream >> rule._r2min;
  stream >> rule._r2max;
  stream >> rule._cut_area;
  stream >> rule._layer;
  stream >> rule._cut_layer_below;

  return stream;
}

////////////////////////////////////////////////////////////////////
//
// _dbTechV55InfluenceEntry - Methods
//
////////////////////////////////////////////////////////////////////

dbOStream& operator<<(dbOStream& stream,
                      const _dbTechV55InfluenceEntry& infitem)
{
  stream << infitem._width;
  stream << infitem._within;
  stream << infitem._spacing;
  return stream;
}

dbIStream& operator>>(dbIStream& stream, _dbTechV55InfluenceEntry& infitem)
{
  stream >> infitem._width;
  stream >> infitem._within;
  stream >> infitem._spacing;
  return stream;
}

////////////////////////////////////////////////////////////////////
//
// dbTechLayerSpacingRule - Methods
//
////////////////////////////////////////////////////////////////////

uint dbTechLayerSpacingRule::getSpacing() const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  return _lsp->_spacing;
}

void dbTechLayerSpacingRule::setSpacing(uint spacing)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  _lsp->_spacing = spacing;
}

bool dbTechLayerSpacingRule::getCutStacking() const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  return _lsp->_flags._cut_stacking;
}

void dbTechLayerSpacingRule::setCutStacking(bool stacking)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  _lsp->_flags._cut_stacking = stacking;
}

bool dbTechLayerSpacingRule::getCutCenterToCenter() const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  return _lsp->_flags._cut_center_to_center;
}

void dbTechLayerSpacingRule::setCutCenterToCenter(bool c2c)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  _lsp->_flags._cut_center_to_center = c2c;
}

bool dbTechLayerSpacingRule::getCutSameNet() const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  return _lsp->_flags._cut_same_net;
}

void dbTechLayerSpacingRule::setCutSameNet(bool same_net)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  _lsp->_flags._cut_same_net = same_net;
}

bool dbTechLayerSpacingRule::getCutParallelOverlap() const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  return _lsp->_flags._cut_parallel_overlap;
}

void dbTechLayerSpacingRule::setCutParallelOverlap(bool overlap)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  _lsp->_flags._cut_parallel_overlap = overlap;
}

uint dbTechLayerSpacingRule::getCutArea() const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  return _lsp->_cut_area;
}

void dbTechLayerSpacingRule::setCutArea(uint area)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  _lsp->_cut_area = area;
}

bool dbTechLayerSpacingRule::isUnconditional() const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  return (_lsp->_flags._rule == _dbTechLayerSpacingRule::DEFAULT);
}

bool dbTechLayerSpacingRule::getLengthThreshold(uint& threshold) const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  if ((_lsp->_flags._rule != _dbTechLayerSpacingRule::LENGTHTHRESHOLD)
      && (_lsp->_flags._rule
          != _dbTechLayerSpacingRule::LENGTHTHRESHOLD_RANGE)) {
    return false;
  }

  threshold = _lsp->_length_or_influence;
  return true;
}

bool dbTechLayerSpacingRule::getLengthThresholdRange(uint& rmin,
                                                     uint& rmax) const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  if (_lsp->_flags._rule != _dbTechLayerSpacingRule::LENGTHTHRESHOLD_RANGE) {
    return false;
  }

  rmin = _lsp->_r2min;
  rmax = _lsp->_r2max;
  return true;
}
bool dbTechLayerSpacingRule::hasRange() const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  if ((_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_ONLY)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_RANGE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_USELENGTH)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_INFLUENCE)
      && (_lsp->_flags._rule
          != _dbTechLayerSpacingRule::RANGE_INFLUENCE_RANGE)) {
    return false;
  }
  return true;
}

bool dbTechLayerSpacingRule::hasLengthThreshold() const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  if ((_lsp->_flags._rule != _dbTechLayerSpacingRule::LENGTHTHRESHOLD)
      && (_lsp->_flags._rule
          != _dbTechLayerSpacingRule::LENGTHTHRESHOLD_RANGE)) {
    return false;
  }
  return true;
}

void dbTechLayerSpacingRule::setSpacingNotchLengthValid(bool val)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  _lsp->_flags._notch_length = val;
}

void dbTechLayerSpacingRule::setSpacingEndOfNotchWidthValid(bool val)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  _lsp->_flags._end_of_notch_width = val;
}

bool dbTechLayerSpacingRule::hasSpacingNotchLength() const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  return _lsp->_flags._notch_length;
}

bool dbTechLayerSpacingRule::hasSpacingEndOfNotchWidth() const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  return _lsp->_flags._end_of_notch_width;
}

bool dbTechLayerSpacingRule::getRange(uint& rmin, uint& rmax) const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  if ((_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_ONLY)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_RANGE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_USELENGTH)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_INFLUENCE)
      && (_lsp->_flags._rule
          != _dbTechLayerSpacingRule::RANGE_INFLUENCE_RANGE)) {
    return false;
  }

  rmin = _lsp->_r1min;
  rmax = _lsp->_r1max;
  return true;
}

bool dbTechLayerSpacingRule::hasUseLengthThreshold() const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  return (_lsp->_flags._rule == _dbTechLayerSpacingRule::RANGE_USELENGTH);
}

bool dbTechLayerSpacingRule::getInfluence(uint& influence) const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  if ((_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_INFLUENCE)
      && (_lsp->_flags._rule
          != _dbTechLayerSpacingRule::RANGE_INFLUENCE_RANGE)) {
    return false;
  }

  influence = _lsp->_length_or_influence;
  return true;
}

bool dbTechLayerSpacingRule::getInfluenceRange(uint& rmin, uint& rmax) const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  if (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_INFLUENCE_RANGE) {
    return false;
  }

  rmin = _lsp->_r2min;
  rmax = _lsp->_r2max;
  return true;
}

bool dbTechLayerSpacingRule::getRangeRange(uint& rmin, uint& rmax) const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  if (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_RANGE) {
    return false;
  }

  rmin = _lsp->_r2min;
  rmax = _lsp->_r2max;
  return true;
}

bool dbTechLayerSpacingRule::getAdjacentCuts(uint& numcuts,
                                             uint& within,
                                             uint& spacing,
                                             bool& except_same_pgnet) const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;

  if (_lsp->_flags._rule != _dbTechLayerSpacingRule::ADJACENT_CUTS_INFLUENCE) {
    return false;
  }

  spacing = _lsp->_spacing;
  within = _lsp->_length_or_influence;
  numcuts = _lsp->_r1min;
  except_same_pgnet = _lsp->_flags._except_same_pgnet;
  return true;
}

bool dbTechLayerSpacingRule::getCutLayer4Spacing(dbTechLayer*& outly) const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  if (_lsp->_flags._rule != _dbTechLayerSpacingRule::CUT_LAYER_BELOW) {
    return false;
  }

  dbTechLayer* tmply = (dbTechLayer*) _lsp->getOwner();
  dbTech* tmptech = (dbTech*) tmply->getImpl()->getOwner();
  outly = odb::dbTechLayer::getTechLayer(tmptech, _lsp->_cut_layer_below);
  return true;
}

void dbTechLayerSpacingRule::setLengthThreshold(uint threshold)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  assert(
      (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_ONLY)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_RANGE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_USELENGTH)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_INFLUENCE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_INFLUENCE_RANGE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::CUT_LAYER_BELOW)
      && (_lsp->_flags._rule
          != _dbTechLayerSpacingRule::ADJACENT_CUTS_INFLUENCE));

  // Not already LENGTHTHRESHOLD or LENGTHTHRESHOLD_RANGE
  if (_lsp->_flags._rule == _dbTechLayerSpacingRule::DEFAULT) {
    _lsp->_flags._rule = _dbTechLayerSpacingRule::LENGTHTHRESHOLD;
  }

  _lsp->_length_or_influence = threshold;
}

void dbTechLayerSpacingRule::setLengthThresholdRange(uint rmin, uint rmax)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  assert((_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_ONLY)
         && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_RANGE)
         && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_USELENGTH)
         && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_INFLUENCE)
         && (_lsp->_flags._rule
             != _dbTechLayerSpacingRule::RANGE_INFLUENCE_RANGE));

  _lsp->_flags._rule = _dbTechLayerSpacingRule::LENGTHTHRESHOLD_RANGE;
  _lsp->_r2min = rmin;
  _lsp->_r2max = rmax;
}

void dbTechLayerSpacingRule::setRange(uint rmin, uint rmax)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  assert((_lsp->_flags._rule != _dbTechLayerSpacingRule::LENGTHTHRESHOLD)
         && (_lsp->_flags._rule
             != _dbTechLayerSpacingRule::LENGTHTHRESHOLD_RANGE));

  if (_lsp->_flags._rule == _dbTechLayerSpacingRule::DEFAULT) {
    _lsp->_flags._rule = _dbTechLayerSpacingRule::RANGE_ONLY;
  }

  _lsp->_r1min = rmin;
  _lsp->_r1max = rmax;
}

void dbTechLayerSpacingRule::setUseLengthThreshold()
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  assert(
      (_lsp->_flags._rule != _dbTechLayerSpacingRule::LENGTHTHRESHOLD)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::LENGTHTHRESHOLD_RANGE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_RANGE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_INFLUENCE_RANGE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_INFLUENCE));

  _lsp->_flags._rule = _dbTechLayerSpacingRule::RANGE_USELENGTH;
}

void dbTechLayerSpacingRule::setInfluence(uint influence)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  assert(
      (_lsp->_flags._rule != _dbTechLayerSpacingRule::LENGTHTHRESHOLD)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::LENGTHTHRESHOLD_RANGE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_USELENGTH)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::CUT_LAYER_BELOW)
      && (_lsp->_flags._rule
          != _dbTechLayerSpacingRule::ADJACENT_CUTS_INFLUENCE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_RANGE));

  // Not already RANGE_INFLUENCE or RANGE_INFLUENCE_RANGE
  if ((_lsp->_flags._rule == _dbTechLayerSpacingRule::DEFAULT)
      || (_lsp->_flags._rule == _dbTechLayerSpacingRule::RANGE_ONLY)) {
    _lsp->_flags._rule = _dbTechLayerSpacingRule::RANGE_INFLUENCE;
  }

  _lsp->_length_or_influence = influence;
}

void dbTechLayerSpacingRule::setInfluenceRange(uint rmin, uint rmax)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  assert(
      (_lsp->_flags._rule != _dbTechLayerSpacingRule::LENGTHTHRESHOLD)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::LENGTHTHRESHOLD_RANGE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_RANGE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_USELENGTH));

  _lsp->_flags._rule = _dbTechLayerSpacingRule::RANGE_INFLUENCE_RANGE;
  _lsp->_r2min = rmin;
  _lsp->_r2max = rmax;
}

void dbTechLayerSpacingRule::setRangeRange(uint rmin, uint rmax)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  assert(
      (_lsp->_flags._rule != _dbTechLayerSpacingRule::LENGTHTHRESHOLD)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::LENGTHTHRESHOLD_RANGE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_INFLUENCE_RANGE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_INFLUENCE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_USELENGTH));

  _lsp->_flags._rule = _dbTechLayerSpacingRule::RANGE_RANGE;
  _lsp->_r2min = rmin;
  _lsp->_r2max = rmax;
}

void dbTechLayerSpacingRule::setEol(uint width,
                                    uint within,
                                    bool parallelEdge,
                                    uint parallelSpace,
                                    uint parallelWithin,
                                    bool twoEdges)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  assert(
      (_lsp->_flags._rule != _dbTechLayerSpacingRule::LENGTHTHRESHOLD)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::LENGTHTHRESHOLD_RANGE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_RANGE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_INFLUENCE_RANGE)
      && (_lsp->_flags._rule != _dbTechLayerSpacingRule::RANGE_INFLUENCE));

  _lsp->_r1min = width;
  _lsp->_r1max = within;
  if (!parallelEdge) {
    _lsp->_flags._rule = _dbTechLayerSpacingRule::ENDOFLINE;
  } else {
    _lsp->_r2min = parallelSpace;
    _lsp->_r2max = parallelWithin;
    if (!twoEdges) {
      _lsp->_flags._rule = _dbTechLayerSpacingRule::ENDOFLINE_PARALLEL;
    } else {
      _lsp->_flags._rule = _dbTechLayerSpacingRule::ENDOFLINE_PARALLEL_TWOEDGES;
    }
  }
}

bool dbTechLayerSpacingRule::getEol(uint& width,
                                    uint& within,
                                    bool& parallelEdge,
                                    uint& parallelSpace,
                                    uint& parallelWithin,
                                    bool& twoEdges) const
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;

  if (_lsp->_flags._rule != _dbTechLayerSpacingRule::ENDOFLINE
      && _lsp->_flags._rule != _dbTechLayerSpacingRule::ENDOFLINE_PARALLEL
      && _lsp->_flags._rule
             != _dbTechLayerSpacingRule::ENDOFLINE_PARALLEL_TWOEDGES) {
    return false;
  }

  parallelSpace = 0;
  parallelWithin = 0;
  twoEdges = false;
  width = _lsp->_r1min;
  within = _lsp->_r1max;
  if (_lsp->_flags._rule == _dbTechLayerSpacingRule::ENDOFLINE) {
    parallelSpace = false;
    return true;
  }
  parallelEdge = true;
  parallelSpace = _lsp->_r2min;
  parallelWithin = _lsp->_r2max;
  twoEdges = _lsp->_flags._rule
             == _dbTechLayerSpacingRule::ENDOFLINE_PARALLEL_TWOEDGES;
  return true;
}

void dbTechLayerSpacingRule::setSameNetPgOnly(bool pgonly)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  _lsp->_flags._except_same_pgnet = pgonly;
}

bool dbTechLayerSpacingRule::getSameNetPgOnly()
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  return _lsp->_flags._except_same_pgnet;
}

void dbTechLayerSpacingRule::setAdjacentCuts(uint numcuts,
                                             uint within,
                                             uint spacing,
                                             bool except_same_pgnet)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;

  assert((_lsp->_flags._rule == _dbTechLayerSpacingRule::DEFAULT)
         || (_lsp->_flags._rule
             == _dbTechLayerSpacingRule::ADJACENT_CUTS_INFLUENCE));

  _lsp->_flags._rule = _dbTechLayerSpacingRule::ADJACENT_CUTS_INFLUENCE;
  _lsp->_flags._except_same_pgnet = except_same_pgnet;
  _lsp->_spacing = spacing;
  _lsp->_length_or_influence = within;
  _lsp->_r1min = numcuts;
}

void dbTechLayerSpacingRule::setCutLayer4Spacing(dbTechLayer* cutly)
{
  _dbTechLayerSpacingRule* _lsp = (_dbTechLayerSpacingRule*) this;
  assert((_lsp->_flags._rule == _dbTechLayerSpacingRule::DEFAULT)
         || (_lsp->_flags._rule == _dbTechLayerSpacingRule::CUT_LAYER_BELOW));

  dbTechLayer* tmply = (dbTechLayer*) _lsp->getOwner();
  ZASSERT(cutly->getNumber() < tmply->getNumber());
  _dbTechLayer* ct_ly = (_dbTechLayer*) cutly;

  _lsp->_flags._rule = _dbTechLayerSpacingRule::CUT_LAYER_BELOW;
  _lsp->_cut_layer_below = ct_ly->getOID();
}

dbTechLayerSpacingRule* dbTechLayerSpacingRule::create(dbTechLayer* inly)
{
  _dbTechLayer* layer = (_dbTechLayer*) inly;
  _dbTechLayerSpacingRule* newrule = layer->_spacing_rules_tbl->create();
  newrule->_layer = inly->getImpl()->getOID();

  return ((dbTechLayerSpacingRule*) newrule);
}

dbTechLayerSpacingRule* dbTechLayerSpacingRule::getTechLayerSpacingRule(
    dbTechLayer* inly,
    uint dbid)
{
  _dbTechLayer* layer = (_dbTechLayer*) inly;
  return (dbTechLayerSpacingRule*) layer->_spacing_rules_tbl->getPtr(dbid);
}

void dbTechLayerSpacingRule::writeLef(lefout& writer) const
{
  uint rmin, rmax, length_or_influence, cut_spacing, numcuts;
  bool except_same_pgnet;
  dbTechLayer* rulely;

  fmt::print(writer.out(), "    SPACING {:g} ", writer.lefdist(getSpacing()));

  if (getCutCenterToCenter()) {
    fmt::print(writer.out(), "    CENTERTOCENTER ");
  }

  if (getCutSameNet()) {
    fmt::print(writer.out(), "    SAMENET ");
  }

  if (getCutParallelOverlap()) {
    fmt::print(writer.out(), "    PARALLELOVERLAP ");
  }

  if (getCutArea() > 0) {
    fmt::print(writer.out(), "    AREA {:g} ", writer.lefdist(getCutArea()));
  }

  if (getRange(rmin, rmax)) {
    fmt::print(writer.out(),
               "RANGE {:g} {:g} ",
               writer.lefdist(rmin),
               writer.lefdist(rmax));
    if (hasUseLengthThreshold()) {
      fmt::print(writer.out(), "USELENGTHTHRESHOLD ");
    } else if (getInfluence(length_or_influence)) {
      fmt::print(
          writer.out(), "INFLUENCE {:g} ", writer.lefdist(length_or_influence));
      if (getInfluenceRange(rmin, rmax)) {
        fmt::print(writer.out(),
                   "RANGE {:g} {:g} ",
                   writer.lefdist(rmin),
                   writer.lefdist(rmax));
      }
    } else if (getRangeRange(rmin, rmax)) {
      fmt::print(writer.out(),
                 "RANGE {:g} {:g} ",
                 writer.lefdist(rmin),
                 writer.lefdist(rmax));
    }
  } else if (getLengthThreshold(length_or_influence)) {
    fmt::print(writer.out(),
               "_dbTechLayerSpacingRule::LENGTHTHRESHOLD {:g} ",
               writer.lefdist(length_or_influence));
    if (getLengthThresholdRange(rmin, rmax)) {
      fmt::print(writer.out(),
                 "RANGE {:g} {:g} ",
                 writer.lefdist(rmin),
                 writer.lefdist(rmax));
    }
  } else if (getCutLayer4Spacing(rulely)) {
    fmt::print(writer.out(), "LAYER {} ", rulely->getName().c_str());
  } else if (getAdjacentCuts(numcuts,
                             length_or_influence,
                             cut_spacing,
                             except_same_pgnet)) {
    fmt::print(writer.out(),
               "ADJACENTCUTS {} WITHIN {:g} ",
               numcuts,
               writer.lefdist(length_or_influence));
    if (except_same_pgnet) {
      fmt::print(writer.out(), "EXCEPTSAMEPGNET ");
    }
  } else {
    uint width, within, parallelSpace, parallelWithin;
    bool parallelEdge, twoEdges;
    if (getEol(width,
               within,
               parallelEdge,
               parallelSpace,
               parallelWithin,
               twoEdges)) {
      fmt::print(writer.out(),
                 "ENDOFLINE {:g} WITHIN {:g} ",
                 writer.lefdist(width),
                 writer.lefdist(within));
      if (parallelEdge) {
        fmt::print(writer.out(),
                   "PARALLELEDGE {:g} WITHIN {:g} ",
                   writer.lefdist(parallelSpace),
                   writer.lefdist(parallelWithin));
        if (twoEdges) {
          fmt::print(writer.out(), " TWOEDGES ");
        }
      }
    }
  }
  fmt::print(writer.out(), " ;\n");
}

////////////////////////////////////////////////////////////////////
//
// dbTechV55InfluenceEntry - Methods
//
////////////////////////////////////////////////////////////////////

bool dbTechV55InfluenceEntry::getV55InfluenceEntry(uint& width,
                                                   uint& within,
                                                   uint& spacing) const
{
  _dbTechV55InfluenceEntry* _v55ie = (_dbTechV55InfluenceEntry*) this;
  width = _v55ie->_width;
  within = _v55ie->_within;
  spacing = _v55ie->_spacing;
  return true;
}

void dbTechV55InfluenceEntry::setV55InfluenceEntry(const uint& width,
                                                   const uint& within,
                                                   const uint& spacing)
{
  _dbTechV55InfluenceEntry* _v55ie = (_dbTechV55InfluenceEntry*) this;
  _v55ie->_width = width;
  _v55ie->_within = within;
  _v55ie->_spacing = spacing;
}

void dbTechV55InfluenceEntry::writeLef(lefout& writer) const
{
  uint inf_width, inf_within, inf_spacing;

  getV55InfluenceEntry(inf_width, inf_within, inf_spacing);
  fmt::print(writer.out(),
             "\n   WIDTH {:g} WITHIN {:g} SPACING {:g}",
             writer.lefdist(inf_width),
             writer.lefdist(inf_within),
             writer.lefdist(inf_spacing));
}

dbTechV55InfluenceEntry* dbTechV55InfluenceEntry::create(dbTechLayer* inly)
{
  _dbTechLayer* layer = (_dbTechLayer*) inly;
  _dbTechV55InfluenceEntry* newitem = layer->_v55inf_tbl->create();
  return ((dbTechV55InfluenceEntry*) newitem);
}

dbTechV55InfluenceEntry* dbTechV55InfluenceEntry::getV55InfluenceEntry(
    dbTechLayer* inly,
    uint oid)
{
  _dbTechLayer* layer = (_dbTechLayer*) inly;
  return (dbTechV55InfluenceEntry*) layer->_v55inf_tbl->getPtr(oid);
}

}  // namespace odb
