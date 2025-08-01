// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

// Generator Code Begin Cpp
#include "dbTechLayer.h"

#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

#include "dbDatabase.h"
#include "dbTable.h"
#include "dbTable.hpp"
#include "dbTechLayerAreaRule.h"
#include "dbTechLayerArraySpacingRule.h"
#include "dbTechLayerCornerSpacingRule.h"
#include "dbTechLayerCutClassRule.h"
#include "dbTechLayerCutEnclosureRule.h"
#include "dbTechLayerCutSpacingRule.h"
#include "dbTechLayerCutSpacingTableDefRule.h"
#include "dbTechLayerCutSpacingTableOrthRule.h"
#include "dbTechLayerEolExtensionRule.h"
#include "dbTechLayerEolKeepOutRule.h"
#include "dbTechLayerForbiddenSpacingRule.h"
#include "dbTechLayerKeepOutZoneRule.h"
#include "dbTechLayerMaxSpacingRule.h"
#include "dbTechLayerMinCutRule.h"
#include "dbTechLayerMinStepRule.h"
#include "dbTechLayerSpacingEolRule.h"
#include "dbTechLayerSpacingTablePrlRule.h"
#include "dbTechLayerTwoWiresForbiddenSpcRule.h"
#include "dbTechLayerWidthTableRule.h"
#include "dbTechLayerWrongDirSpacingRule.h"
#include "odb/db.h"
#include "odb/dbSet.h"
// User Code Begin Includes
#include <spdlog/fmt/ostr.h>

#include "dbHashTable.hpp"
#include "dbTech.h"
#include "dbTechLayerAntennaRule.h"
#include "dbTechLayerSpacingRule.h"
#include "dbTechMinCutOrAreaRule.h"
#include "odb/lefout.h"
#include "utl/Logger.h"
// User Code End Includes
namespace odb {
template class dbTable<_dbTechLayer>;

bool _dbTechLayer::operator==(const _dbTechLayer& rhs) const
{
  if (flags_.num_masks_ != rhs.flags_.num_masks_) {
    return false;
  }
  if (flags_.has_max_width_ != rhs.flags_.has_max_width_) {
    return false;
  }
  if (flags_.has_thickness_ != rhs.flags_.has_thickness_) {
    return false;
  }
  if (flags_.has_area_ != rhs.flags_.has_area_) {
    return false;
  }
  if (flags_.has_protrusion_ != rhs.flags_.has_protrusion_) {
    return false;
  }
  if (flags_.has_alias_ != rhs.flags_.has_alias_) {
    return false;
  }
  if (flags_.has_xy_pitch_ != rhs.flags_.has_xy_pitch_) {
    return false;
  }
  if (flags_.has_xy_offset_ != rhs.flags_.has_xy_offset_) {
    return false;
  }
  if (flags_.rect_only_ != rhs.flags_.rect_only_) {
    return false;
  }
  if (flags_.right_way_on_grid_only_ != rhs.flags_.right_way_on_grid_only_) {
    return false;
  }
  if (flags_.right_way_on_grid_only_check_mask_
      != rhs.flags_.right_way_on_grid_only_check_mask_) {
    return false;
  }
  if (flags_.rect_only_except_non_core_pins_
      != rhs.flags_.rect_only_except_non_core_pins_) {
    return false;
  }
  if (flags_.lef58_type_ != rhs.flags_.lef58_type_) {
    return false;
  }
  if (wrong_way_width_ != rhs.wrong_way_width_) {
    return false;
  }
  if (layer_adjustment_ != rhs.layer_adjustment_) {
    return false;
  }
  if (*cut_class_rules_tbl_ != *rhs.cut_class_rules_tbl_) {
    return false;
  }
  if (cut_class_rules_hash_ != rhs.cut_class_rules_hash_) {
    return false;
  }
  if (*spacing_eol_rules_tbl_ != *rhs.spacing_eol_rules_tbl_) {
    return false;
  }
  if (*cut_spacing_rules_tbl_ != *rhs.cut_spacing_rules_tbl_) {
    return false;
  }
  if (*minstep_rules_tbl_ != *rhs.minstep_rules_tbl_) {
    return false;
  }
  if (*corner_spacing_rules_tbl_ != *rhs.corner_spacing_rules_tbl_) {
    return false;
  }
  if (*spacing_table_prl_rules_tbl_ != *rhs.spacing_table_prl_rules_tbl_) {
    return false;
  }
  if (*cut_spacing_table_orth_tbl_ != *rhs.cut_spacing_table_orth_tbl_) {
    return false;
  }
  if (*cut_spacing_table_def_tbl_ != *rhs.cut_spacing_table_def_tbl_) {
    return false;
  }
  if (*cut_enc_rules_tbl_ != *rhs.cut_enc_rules_tbl_) {
    return false;
  }
  if (*eol_ext_rules_tbl_ != *rhs.eol_ext_rules_tbl_) {
    return false;
  }
  if (*array_spacing_rules_tbl_ != *rhs.array_spacing_rules_tbl_) {
    return false;
  }
  if (*eol_keep_out_rules_tbl_ != *rhs.eol_keep_out_rules_tbl_) {
    return false;
  }
  if (*max_spacing_rules_tbl_ != *rhs.max_spacing_rules_tbl_) {
    return false;
  }
  if (*width_table_rules_tbl_ != *rhs.width_table_rules_tbl_) {
    return false;
  }
  if (*min_cuts_rules_tbl_ != *rhs.min_cuts_rules_tbl_) {
    return false;
  }
  if (*area_rules_tbl_ != *rhs.area_rules_tbl_) {
    return false;
  }
  if (*forbidden_spacing_rules_tbl_ != *rhs.forbidden_spacing_rules_tbl_) {
    return false;
  }
  if (*keepout_zone_rules_tbl_ != *rhs.keepout_zone_rules_tbl_) {
    return false;
  }
  if (*wrongdir_spacing_rules_tbl_ != *rhs.wrongdir_spacing_rules_tbl_) {
    return false;
  }
  if (*two_wires_forbidden_spc_rules_tbl_
      != *rhs.two_wires_forbidden_spc_rules_tbl_) {
    return false;
  }

  // User Code Begin ==
  if (flags_.type_ != rhs.flags_.type_) {
    return false;
  }

  if (flags_.direction_ != rhs.flags_.direction_) {
    return false;
  }

  if (flags_.minstep_type_ != rhs.flags_.minstep_type_) {
    return false;
  }

  if (_pitch_x != rhs._pitch_x) {
    return false;
  }

  if (_pitch_y != rhs._pitch_y) {
    return false;
  }

  if (_offset_x != rhs._offset_x) {
    return false;
  }

  if (_offset_y != rhs._offset_y) {
    return false;
  }

  if (_width != rhs._width) {
    return false;
  }

  if (_spacing != rhs._spacing) {
    return false;
  }

  if (_resistance != rhs._resistance) {
    return false;
  }

  if (_capacitance != rhs._capacitance) {
    return false;
  }

  if (_edge_capacitance != rhs._edge_capacitance) {
    return false;
  }

  if (_wire_extension != rhs._wire_extension) {
    return false;
  }

  if (_number != rhs._number) {
    return false;
  }

  if (_rlevel != rhs._rlevel) {
    return false;
  }

  if (_area != rhs._area) {
    return false;
  }

  if (_thickness != rhs._thickness) {
    return false;
  }

  if (_min_step != rhs._min_step) {
    return false;
  }

  if (_max_width != rhs._max_width) {
    return false;
  }

  if (_min_width != rhs._min_width) {
    return false;
  }

  if (_min_step_max_length != rhs._min_step_max_length) {
    return false;
  }

  if (_min_step_max_edges != rhs._min_step_max_edges) {
    return false;
  }

  if (_first_last_pitch != rhs._first_last_pitch) {
    return false;
  }

  if (_pt._width != rhs._pt._width) {
    return false;
  }

  if (_pt._length != rhs._pt._length) {
    return false;
  }

  if (_pt._from_width != rhs._pt._from_width) {
    return false;
  }

  if (_name && rhs._name) {
    if (strcmp(_name, rhs._name) != 0) {
      return false;
    }
  } else if (_name || rhs._name) {
    return false;
  }

  if (_alias && rhs._alias) {
    if (strcmp(_alias, rhs._alias) != 0) {
      return false;
    }
  } else if (_alias || rhs._alias) {
    return false;
  }

  if (_upper != rhs._upper) {
    return false;
  }

  if (_lower != rhs._lower) {
    return false;
  }

  if (*_spacing_rules_tbl != *rhs._spacing_rules_tbl) {
    return false;
  }

  if (*_min_cut_rules_tbl != *rhs._min_cut_rules_tbl) {
    return false;
  }

  if (*_min_enc_rules_tbl != *rhs._min_enc_rules_tbl) {
    return false;
  }

  if (*_v55inf_tbl != *rhs._v55inf_tbl) {
    return false;
  }

  if (_v55sp_length_idx != rhs._v55sp_length_idx) {
    return false;
  }

  if (_v55sp_width_idx != rhs._v55sp_width_idx) {
    return false;
  }

  if (_v55sp_spacing != rhs._v55sp_spacing) {
    return false;
  }

  if (_two_widths_sp_idx != rhs._two_widths_sp_idx) {
    return false;
  }

  if (_two_widths_sp_prl != rhs._two_widths_sp_prl) {
    return false;
  }

  if (_two_widths_sp_spacing != rhs._two_widths_sp_spacing) {
    return false;
  }

  if (_oxide1 != rhs._oxide1) {
    return false;
  }

  if (_oxide2 != rhs._oxide2) {
    return false;
  }
  // User Code End ==
  return true;
}

bool _dbTechLayer::operator<(const _dbTechLayer& rhs) const
{
  // User Code Begin <
  if (_number >= rhs._number) {
    return false;
  }
  // User Code End <
  return true;
}

_dbTechLayer::_dbTechLayer(_dbDatabase* db)
{
  flags_ = {};
  wrong_way_width_ = 0;
  layer_adjustment_ = 0;
  cut_class_rules_tbl_ = new dbTable<_dbTechLayerCutClassRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerCutClassRuleObj);
  cut_class_rules_hash_.setTable(cut_class_rules_tbl_);
  spacing_eol_rules_tbl_ = new dbTable<_dbTechLayerSpacingEolRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerSpacingEolRuleObj);
  cut_spacing_rules_tbl_ = new dbTable<_dbTechLayerCutSpacingRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerCutSpacingRuleObj);
  minstep_rules_tbl_ = new dbTable<_dbTechLayerMinStepRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerMinStepRuleObj);
  corner_spacing_rules_tbl_ = new dbTable<_dbTechLayerCornerSpacingRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerCornerSpacingRuleObj);
  spacing_table_prl_rules_tbl_ = new dbTable<_dbTechLayerSpacingTablePrlRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerSpacingTablePrlRuleObj);
  cut_spacing_table_orth_tbl_
      = new dbTable<_dbTechLayerCutSpacingTableOrthRule>(
          db,
          this,
          (GetObjTbl_t) &_dbTechLayer::getObjectTable,
          dbTechLayerCutSpacingTableOrthRuleObj);
  cut_spacing_table_def_tbl_ = new dbTable<_dbTechLayerCutSpacingTableDefRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerCutSpacingTableDefRuleObj);
  cut_enc_rules_tbl_ = new dbTable<_dbTechLayerCutEnclosureRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerCutEnclosureRuleObj);
  eol_ext_rules_tbl_ = new dbTable<_dbTechLayerEolExtensionRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerEolExtensionRuleObj);
  array_spacing_rules_tbl_ = new dbTable<_dbTechLayerArraySpacingRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerArraySpacingRuleObj);
  eol_keep_out_rules_tbl_ = new dbTable<_dbTechLayerEolKeepOutRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerEolKeepOutRuleObj);
  max_spacing_rules_tbl_ = new dbTable<_dbTechLayerMaxSpacingRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerMaxSpacingRuleObj);
  width_table_rules_tbl_ = new dbTable<_dbTechLayerWidthTableRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerWidthTableRuleObj);
  min_cuts_rules_tbl_ = new dbTable<_dbTechLayerMinCutRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerMinCutRuleObj);
  area_rules_tbl_ = new dbTable<_dbTechLayerAreaRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerAreaRuleObj);
  forbidden_spacing_rules_tbl_ = new dbTable<_dbTechLayerForbiddenSpacingRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerForbiddenSpacingRuleObj);
  keepout_zone_rules_tbl_ = new dbTable<_dbTechLayerKeepOutZoneRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerKeepOutZoneRuleObj);
  wrongdir_spacing_rules_tbl_ = new dbTable<_dbTechLayerWrongDirSpacingRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerWrongDirSpacingRuleObj);
  two_wires_forbidden_spc_rules_tbl_
      = new dbTable<_dbTechLayerTwoWiresForbiddenSpcRule>(
          db,
          this,
          (GetObjTbl_t) &_dbTechLayer::getObjectTable,
          dbTechLayerTwoWiresForbiddenSpcRuleObj);
  // User Code Begin Constructor
  flags_.type_ = dbTechLayerType::ROUTING;
  flags_.direction_ = dbTechLayerDir::NONE;
  flags_.minstep_type_ = dbTechLayerMinStepType();
  flags_.num_masks_ = 1;
  _pitch_x = 0;
  _pitch_y = 0;
  _offset_x = 0;
  _offset_y = 0;
  _width = 0;
  _spacing = 0;
  _resistance = 0.0;
  _capacitance = 0.0;
  _edge_capacitance = 0.0;
  _wire_extension = 0;
  _number = 0;
  _rlevel = 0;
  _area = 0.0;
  _thickness = 0;
  _min_step = -1;
  _pt._width = 0;
  _pt._length = 0;
  _pt._from_width = 0;
  _max_width = MAX_INT;
  _min_width = 0;
  _min_step_max_length = -1;
  _min_step_max_edges = -1;
  _first_last_pitch = -1;
  _v55sp_length_idx.clear();
  _v55sp_width_idx.clear();
  _v55sp_spacing.clear();
  _name = nullptr;
  _alias = nullptr;

  _spacing_rules_tbl = new dbTable<_dbTechLayerSpacingRule>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechLayerSpacingRuleObj);

  _min_cut_rules_tbl = new dbTable<_dbTechMinCutRule, 8>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechMinCutRuleObj);

  _min_enc_rules_tbl = new dbTable<_dbTechMinEncRule, 8>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechMinEncRuleObj);

  _v55inf_tbl = new dbTable<_dbTechV55InfluenceEntry, 8>(
      db,
      this,
      (GetObjTbl_t) &_dbTechLayer::getObjectTable,
      dbTechV55InfluenceEntryObj);
  // User Code End Constructor
}

dbIStream& operator>>(dbIStream& stream, _dbTechLayer& obj)
{
  uint32_t flags_bit_field;
  stream >> flags_bit_field;
  static_assert(sizeof(obj.flags_) == sizeof(flags_bit_field));
  std::memcpy(&obj.flags_, &flags_bit_field, sizeof(flags_bit_field));
  if (obj.getDatabase()->isSchema(db_schema_orth_spc_tbl)) {
    stream >> obj.orth_spacing_tbl_;
  }
  stream >> *obj.cut_class_rules_tbl_;
  stream >> obj.cut_class_rules_hash_;
  stream >> *obj.spacing_eol_rules_tbl_;
  stream >> *obj.cut_spacing_rules_tbl_;
  stream >> *obj.minstep_rules_tbl_;
  stream >> *obj.corner_spacing_rules_tbl_;
  stream >> *obj.spacing_table_prl_rules_tbl_;
  stream >> *obj.cut_spacing_table_orth_tbl_;
  stream >> *obj.cut_spacing_table_def_tbl_;
  stream >> *obj.cut_enc_rules_tbl_;
  stream >> *obj.eol_ext_rules_tbl_;
  stream >> *obj.array_spacing_rules_tbl_;
  stream >> *obj.eol_keep_out_rules_tbl_;
  if (obj.getDatabase()->isSchema(db_schema_max_spacing)) {
    stream >> *obj.max_spacing_rules_tbl_;
  }
  stream >> *obj.width_table_rules_tbl_;
  stream >> *obj.min_cuts_rules_tbl_;
  stream >> *obj.area_rules_tbl_;
  if (obj.getDatabase()->isSchema(db_schema_lef58_forbidden_spacing)) {
    stream >> *obj.forbidden_spacing_rules_tbl_;
  }
  if (obj.getDatabase()->isSchema(db_schema_keepout_zone)) {
    stream >> *obj.keepout_zone_rules_tbl_;
  }
  if (obj.getDatabase()->isSchema(db_schema_wrongdir_spacing)) {
    stream >> *obj.wrongdir_spacing_rules_tbl_;
  }
  if (obj.getDatabase()->isSchema(
          db_schema_lef58_two_wires_forbidden_spacing)) {
    stream >> *obj.two_wires_forbidden_spc_rules_tbl_;
  }
  // User Code Begin >>
  if (obj.getDatabase()->isSchema(db_schema_layer_adjustment)) {
    stream >> obj.layer_adjustment_;
  } else {
    obj.layer_adjustment_ = 0.0;
  }
  stream >> obj._pitch_x;
  stream >> obj._pitch_y;
  stream >> obj._offset_x;
  stream >> obj._offset_y;
  stream >> obj._width;
  stream >> obj._spacing;
  stream >> obj._resistance;
  stream >> obj._capacitance;
  stream >> obj._edge_capacitance;
  stream >> obj._wire_extension;
  stream >> obj._number;
  stream >> obj._rlevel;
  stream >> obj._area;
  stream >> obj._thickness;
  stream >> obj._min_step;
  stream >> obj._min_step_max_length;
  stream >> obj._min_step_max_edges;
  stream >> obj._max_width;
  stream >> obj._min_width;
  stream >> obj._pt._width;
  stream >> obj._pt._length;
  stream >> obj._pt._from_width;
  stream >> obj._name;
  stream >> obj._alias;
  stream >> obj._lower;
  stream >> obj._upper;
  stream >> *obj._spacing_rules_tbl;
  stream >> *obj._min_cut_rules_tbl;
  stream >> *obj._min_enc_rules_tbl;
  stream >> *obj._v55inf_tbl;
  stream >> obj._v55sp_length_idx;
  stream >> obj._v55sp_width_idx;
  stream >> obj._v55sp_spacing;
  stream >> obj._two_widths_sp_idx;
  stream >> obj._two_widths_sp_prl;
  stream >> obj._two_widths_sp_spacing;
  stream >> obj._oxide1;
  stream >> obj._oxide2;
  if (obj.getDatabase()->isSchema(db_schema_wrongway_width)) {
    stream >> obj.wrong_way_width_;
  } else {
    obj.wrong_way_width_ = obj._width;
    for (auto rule : ((dbTechLayer*) &obj)->getTechLayerWidthTableRules()) {
      if (rule->isWrongDirection()) {
        obj.wrong_way_width_ = *rule->getWidthTable().begin();
        break;
      }
    }
  }
  if (obj.getDatabase()->isSchema(db_schema_lef58_pitch)) {
    stream >> obj._first_last_pitch;
  }
  // User Code End >>
  return stream;
}

dbOStream& operator<<(dbOStream& stream, const _dbTechLayer& obj)
{
  uint32_t flags_bit_field;
  static_assert(sizeof(obj.flags_) == sizeof(flags_bit_field));
  std::memcpy(&flags_bit_field, &obj.flags_, sizeof(obj.flags_));
  stream << flags_bit_field;
  stream << obj.orth_spacing_tbl_;
  stream << *obj.cut_class_rules_tbl_;
  stream << obj.cut_class_rules_hash_;
  stream << *obj.spacing_eol_rules_tbl_;
  stream << *obj.cut_spacing_rules_tbl_;
  stream << *obj.minstep_rules_tbl_;
  stream << *obj.corner_spacing_rules_tbl_;
  stream << *obj.spacing_table_prl_rules_tbl_;
  stream << *obj.cut_spacing_table_orth_tbl_;
  stream << *obj.cut_spacing_table_def_tbl_;
  stream << *obj.cut_enc_rules_tbl_;
  stream << *obj.eol_ext_rules_tbl_;
  stream << *obj.array_spacing_rules_tbl_;
  stream << *obj.eol_keep_out_rules_tbl_;
  stream << *obj.max_spacing_rules_tbl_;
  stream << *obj.width_table_rules_tbl_;
  stream << *obj.min_cuts_rules_tbl_;
  stream << *obj.area_rules_tbl_;
  stream << *obj.forbidden_spacing_rules_tbl_;
  stream << *obj.keepout_zone_rules_tbl_;
  stream << *obj.wrongdir_spacing_rules_tbl_;
  stream << *obj.two_wires_forbidden_spc_rules_tbl_;
  // User Code Begin <<
  stream << obj.layer_adjustment_;
  stream << obj._pitch_x;
  stream << obj._pitch_y;
  stream << obj._offset_x;
  stream << obj._offset_y;
  stream << obj._width;
  stream << obj._spacing;
  stream << obj._resistance;
  stream << obj._capacitance;
  stream << obj._edge_capacitance;
  stream << obj._wire_extension;
  stream << obj._number;
  stream << obj._rlevel;
  stream << obj._area;
  stream << obj._thickness;
  stream << obj._min_step;
  stream << obj._min_step_max_length;
  stream << obj._min_step_max_edges;
  stream << obj._max_width;
  stream << obj._min_width;
  stream << obj._pt._width;
  stream << obj._pt._length;
  stream << obj._pt._from_width;
  stream << obj._name;
  stream << obj._alias;
  stream << obj._lower;
  stream << obj._upper;
  stream << *obj._spacing_rules_tbl;
  stream << *obj._min_cut_rules_tbl;
  stream << *obj._min_enc_rules_tbl;
  stream << *obj._v55inf_tbl;
  stream << obj._v55sp_length_idx;
  stream << obj._v55sp_width_idx;
  stream << obj._v55sp_spacing;
  stream << obj._two_widths_sp_idx;
  stream << obj._two_widths_sp_prl;
  stream << obj._two_widths_sp_spacing;
  stream << obj._oxide1;
  stream << obj._oxide2;
  stream << obj.wrong_way_width_;
  stream << obj._first_last_pitch;
  // User Code End <<
  return stream;
}

dbObjectTable* _dbTechLayer::getObjectTable(dbObjectType type)
{
  switch (type) {
    case dbTechLayerCutClassRuleObj:
      return cut_class_rules_tbl_;
    case dbTechLayerSpacingEolRuleObj:
      return spacing_eol_rules_tbl_;
    case dbTechLayerCutSpacingRuleObj:
      return cut_spacing_rules_tbl_;
    case dbTechLayerMinStepRuleObj:
      return minstep_rules_tbl_;
    case dbTechLayerCornerSpacingRuleObj:
      return corner_spacing_rules_tbl_;
    case dbTechLayerSpacingTablePrlRuleObj:
      return spacing_table_prl_rules_tbl_;
    case dbTechLayerCutSpacingTableOrthRuleObj:
      return cut_spacing_table_orth_tbl_;
    case dbTechLayerCutSpacingTableDefRuleObj:
      return cut_spacing_table_def_tbl_;
    case dbTechLayerCutEnclosureRuleObj:
      return cut_enc_rules_tbl_;
    case dbTechLayerEolExtensionRuleObj:
      return eol_ext_rules_tbl_;
    case dbTechLayerArraySpacingRuleObj:
      return array_spacing_rules_tbl_;
    case dbTechLayerEolKeepOutRuleObj:
      return eol_keep_out_rules_tbl_;
    case dbTechLayerMaxSpacingRuleObj:
      return max_spacing_rules_tbl_;
    case dbTechLayerWidthTableRuleObj:
      return width_table_rules_tbl_;
    case dbTechLayerMinCutRuleObj:
      return min_cuts_rules_tbl_;
    case dbTechLayerAreaRuleObj:
      return area_rules_tbl_;
    case dbTechLayerForbiddenSpacingRuleObj:
      return forbidden_spacing_rules_tbl_;
    case dbTechLayerKeepOutZoneRuleObj:
      return keepout_zone_rules_tbl_;
    case dbTechLayerWrongDirSpacingRuleObj:
      return wrongdir_spacing_rules_tbl_;
    case dbTechLayerTwoWiresForbiddenSpcRuleObj:
      return two_wires_forbidden_spc_rules_tbl_;
      // User Code Begin getObjectTable
    case dbTechLayerSpacingRuleObj:
      return _spacing_rules_tbl;

    case dbTechMinCutRuleObj:
      return _min_cut_rules_tbl;

    case dbTechMinEncRuleObj:
      return _min_enc_rules_tbl;

    case dbTechV55InfluenceEntryObj:
      return _v55inf_tbl;
    // User Code End getObjectTable
    default:
      break;
  }
  return getTable()->getObjectTable(type);
}
void _dbTechLayer::collectMemInfo(MemInfo& info)
{
  info.cnt++;
  info.size += sizeof(*this);

  cut_class_rules_tbl_->collectMemInfo(info.children_["cut_class_rules_tbl_"]);

  spacing_eol_rules_tbl_->collectMemInfo(
      info.children_["spacing_eol_rules_tbl_"]);

  cut_spacing_rules_tbl_->collectMemInfo(
      info.children_["cut_spacing_rules_tbl_"]);

  minstep_rules_tbl_->collectMemInfo(info.children_["minstep_rules_tbl_"]);

  corner_spacing_rules_tbl_->collectMemInfo(
      info.children_["corner_spacing_rules_tbl_"]);

  spacing_table_prl_rules_tbl_->collectMemInfo(
      info.children_["spacing_table_prl_rules_tbl_"]);

  cut_spacing_table_orth_tbl_->collectMemInfo(
      info.children_["cut_spacing_table_orth_tbl_"]);

  cut_spacing_table_def_tbl_->collectMemInfo(
      info.children_["cut_spacing_table_def_tbl_"]);

  cut_enc_rules_tbl_->collectMemInfo(info.children_["cut_enc_rules_tbl_"]);

  eol_ext_rules_tbl_->collectMemInfo(info.children_["eol_ext_rules_tbl_"]);

  array_spacing_rules_tbl_->collectMemInfo(
      info.children_["array_spacing_rules_tbl_"]);

  eol_keep_out_rules_tbl_->collectMemInfo(
      info.children_["eol_keep_out_rules_tbl_"]);

  max_spacing_rules_tbl_->collectMemInfo(
      info.children_["max_spacing_rules_tbl_"]);

  width_table_rules_tbl_->collectMemInfo(
      info.children_["width_table_rules_tbl_"]);

  min_cuts_rules_tbl_->collectMemInfo(info.children_["min_cuts_rules_tbl_"]);

  area_rules_tbl_->collectMemInfo(info.children_["area_rules_tbl_"]);

  forbidden_spacing_rules_tbl_->collectMemInfo(
      info.children_["forbidden_spacing_rules_tbl_"]);

  keepout_zone_rules_tbl_->collectMemInfo(
      info.children_["keepout_zone_rules_tbl_"]);

  wrongdir_spacing_rules_tbl_->collectMemInfo(
      info.children_["wrongdir_spacing_rules_tbl_"]);

  two_wires_forbidden_spc_rules_tbl_->collectMemInfo(
      info.children_["two_wires_forbidden_spc_rules_tbl_"]);

  // User Code Begin collectMemInfo
  info.children_["orth_spacing"].add(orth_spacing_tbl_);
  info.children_["cut_class_rules_hash"].add(cut_class_rules_hash_);
  info.children_["name"].add(_name);
  info.children_["alias"].add(_alias);
  _spacing_rules_tbl->collectMemInfo(info.children_["spacing_rules_tbl"]);
  _min_cut_rules_tbl->collectMemInfo(info.children_["min_cut_rules_tbl"]);
  _min_enc_rules_tbl->collectMemInfo(info.children_["min_enc_rules_tbl"]);
  _v55inf_tbl->collectMemInfo(info.children_["v55inf_tbl"]);
  info.children_["v55sp_length_idx"].add(_v55sp_length_idx);
  info.children_["v55sp_width_idx"].add(_v55sp_width_idx);
  info.children_["v55sp_spacing"].add(_v55sp_spacing);
  info.children_["two_widths_sp_idx"].add(_two_widths_sp_idx);
  info.children_["two_widths_sp_prl"].add(_two_widths_sp_prl);
  info.children_["two_widths_sp_spacing"].add(_two_widths_sp_spacing);
  // User Code End collectMemInfo
}

_dbTechLayer::~_dbTechLayer()
{
  delete cut_class_rules_tbl_;
  delete spacing_eol_rules_tbl_;
  delete cut_spacing_rules_tbl_;
  delete minstep_rules_tbl_;
  delete corner_spacing_rules_tbl_;
  delete spacing_table_prl_rules_tbl_;
  delete cut_spacing_table_orth_tbl_;
  delete cut_spacing_table_def_tbl_;
  delete cut_enc_rules_tbl_;
  delete eol_ext_rules_tbl_;
  delete array_spacing_rules_tbl_;
  delete eol_keep_out_rules_tbl_;
  delete max_spacing_rules_tbl_;
  delete width_table_rules_tbl_;
  delete min_cuts_rules_tbl_;
  delete area_rules_tbl_;
  delete forbidden_spacing_rules_tbl_;
  delete keepout_zone_rules_tbl_;
  delete wrongdir_spacing_rules_tbl_;
  delete two_wires_forbidden_spc_rules_tbl_;
  // User Code Begin Destructor
  if (_name) {
    free((void*) _name);
  }

  {
    delete _spacing_rules_tbl;
  }

  {
    delete _min_cut_rules_tbl;
  }

  {
    delete _min_enc_rules_tbl;
  }

  {
    delete _v55inf_tbl;
  }
  // User Code End Destructor
}

// User Code Begin PrivateMethods
uint _dbTechLayer::getV55RowIdx(const int& rowVal) const
{
  auto pos = --(std::lower_bound(
      _v55sp_width_idx.begin(), _v55sp_width_idx.end(), rowVal));
  return std::max(0, (int) std::distance(_v55sp_width_idx.begin(), pos));
}
uint _dbTechLayer::getV55ColIdx(const int& colVal) const
{
  auto pos = --(std::lower_bound(
      _v55sp_length_idx.begin(), _v55sp_length_idx.end(), colVal));
  return std::max(0, (int) std::distance(_v55sp_length_idx.begin(), pos));
}
uint _dbTechLayer::getTwIdx(const int width, const int prl) const
{
  auto pos = std::lower_bound(
      _two_widths_sp_idx.begin(), _two_widths_sp_idx.end(), width);
  if (pos != _two_widths_sp_idx.begin()) {
    --pos;
  }
  int idx = std::max(0, (int) std::distance(_two_widths_sp_idx.begin(), pos));
  for (; idx >= 0; idx--) {
    if (prl >= _two_widths_sp_prl[idx]) {
      return idx;
    }
  }
  return 0;
}
// User Code End PrivateMethods

////////////////////////////////////////////////////////////////////
//
// dbTechLayer - Methods
//
////////////////////////////////////////////////////////////////////

void dbTechLayer::setWrongWayWidth(uint wrong_way_width)
{
  _dbTechLayer* obj = (_dbTechLayer*) this;

  obj->wrong_way_width_ = wrong_way_width;
}

uint dbTechLayer::getWrongWayWidth() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return obj->wrong_way_width_;
}

void dbTechLayer::setLayerAdjustment(float layer_adjustment)
{
  _dbTechLayer* obj = (_dbTechLayer*) this;

  obj->layer_adjustment_ = layer_adjustment;
}

float dbTechLayer::getLayerAdjustment() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return obj->layer_adjustment_;
}

void dbTechLayer::getOrthSpacingTable(
    std::vector<std::pair<int, int>>& tbl) const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  tbl = obj->orth_spacing_tbl_;
}

dbSet<dbTechLayerCutClassRule> dbTechLayer::getTechLayerCutClassRules() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerCutClassRule>(obj, obj->cut_class_rules_tbl_);
}

dbTechLayerCutClassRule* dbTechLayer::findTechLayerCutClassRule(
    const char* name) const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return (dbTechLayerCutClassRule*) obj->cut_class_rules_hash_.find(name);
}

dbSet<dbTechLayerSpacingEolRule> dbTechLayer::getTechLayerSpacingEolRules()
    const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerSpacingEolRule>(obj, obj->spacing_eol_rules_tbl_);
}

dbSet<dbTechLayerCutSpacingRule> dbTechLayer::getTechLayerCutSpacingRules()
    const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerCutSpacingRule>(obj, obj->cut_spacing_rules_tbl_);
}

dbSet<dbTechLayerMinStepRule> dbTechLayer::getTechLayerMinStepRules() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerMinStepRule>(obj, obj->minstep_rules_tbl_);
}

dbSet<dbTechLayerCornerSpacingRule>
dbTechLayer::getTechLayerCornerSpacingRules() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerCornerSpacingRule>(obj,
                                             obj->corner_spacing_rules_tbl_);
}

dbSet<dbTechLayerSpacingTablePrlRule>
dbTechLayer::getTechLayerSpacingTablePrlRules() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerSpacingTablePrlRule>(
      obj, obj->spacing_table_prl_rules_tbl_);
}

dbSet<dbTechLayerCutSpacingTableOrthRule>
dbTechLayer::getTechLayerCutSpacingTableOrthRules() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerCutSpacingTableOrthRule>(
      obj, obj->cut_spacing_table_orth_tbl_);
}

dbSet<dbTechLayerCutSpacingTableDefRule>
dbTechLayer::getTechLayerCutSpacingTableDefRules() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerCutSpacingTableDefRule>(
      obj, obj->cut_spacing_table_def_tbl_);
}

dbSet<dbTechLayerCutEnclosureRule> dbTechLayer::getTechLayerCutEnclosureRules()
    const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerCutEnclosureRule>(obj, obj->cut_enc_rules_tbl_);
}

dbSet<dbTechLayerEolExtensionRule> dbTechLayer::getTechLayerEolExtensionRules()
    const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerEolExtensionRule>(obj, obj->eol_ext_rules_tbl_);
}

dbSet<dbTechLayerArraySpacingRule> dbTechLayer::getTechLayerArraySpacingRules()
    const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerArraySpacingRule>(obj, obj->array_spacing_rules_tbl_);
}

dbSet<dbTechLayerEolKeepOutRule> dbTechLayer::getTechLayerEolKeepOutRules()
    const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerEolKeepOutRule>(obj, obj->eol_keep_out_rules_tbl_);
}

dbSet<dbTechLayerMaxSpacingRule> dbTechLayer::getTechLayerMaxSpacingRules()
    const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerMaxSpacingRule>(obj, obj->max_spacing_rules_tbl_);
}

dbSet<dbTechLayerWidthTableRule> dbTechLayer::getTechLayerWidthTableRules()
    const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerWidthTableRule>(obj, obj->width_table_rules_tbl_);
}

dbSet<dbTechLayerMinCutRule> dbTechLayer::getTechLayerMinCutRules() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerMinCutRule>(obj, obj->min_cuts_rules_tbl_);
}

dbSet<dbTechLayerAreaRule> dbTechLayer::getTechLayerAreaRules() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerAreaRule>(obj, obj->area_rules_tbl_);
}

dbSet<dbTechLayerForbiddenSpacingRule>
dbTechLayer::getTechLayerForbiddenSpacingRules() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerForbiddenSpacingRule>(
      obj, obj->forbidden_spacing_rules_tbl_);
}

dbSet<dbTechLayerKeepOutZoneRule> dbTechLayer::getTechLayerKeepOutZoneRules()
    const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerKeepOutZoneRule>(obj, obj->keepout_zone_rules_tbl_);
}

dbSet<dbTechLayerWrongDirSpacingRule>
dbTechLayer::getTechLayerWrongDirSpacingRules() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerWrongDirSpacingRule>(
      obj, obj->wrongdir_spacing_rules_tbl_);
}

dbSet<dbTechLayerTwoWiresForbiddenSpcRule>
dbTechLayer::getTechLayerTwoWiresForbiddenSpcRules() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;
  return dbSet<dbTechLayerTwoWiresForbiddenSpcRule>(
      obj, obj->two_wires_forbidden_spc_rules_tbl_);
}

void dbTechLayer::setRectOnly(bool rect_only)
{
  _dbTechLayer* obj = (_dbTechLayer*) this;

  obj->flags_.rect_only_ = rect_only;
}

bool dbTechLayer::isRectOnly() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;

  return obj->flags_.rect_only_;
}

void dbTechLayer::setRightWayOnGridOnly(bool right_way_on_grid_only)
{
  _dbTechLayer* obj = (_dbTechLayer*) this;

  obj->flags_.right_way_on_grid_only_ = right_way_on_grid_only;
}

bool dbTechLayer::isRightWayOnGridOnly() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;

  return obj->flags_.right_way_on_grid_only_;
}

void dbTechLayer::setRightWayOnGridOnlyCheckMask(
    bool right_way_on_grid_only_check_mask)
{
  _dbTechLayer* obj = (_dbTechLayer*) this;

  obj->flags_.right_way_on_grid_only_check_mask_
      = right_way_on_grid_only_check_mask;
}

bool dbTechLayer::isRightWayOnGridOnlyCheckMask() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;

  return obj->flags_.right_way_on_grid_only_check_mask_;
}

void dbTechLayer::setRectOnlyExceptNonCorePins(
    bool rect_only_except_non_core_pins)
{
  _dbTechLayer* obj = (_dbTechLayer*) this;

  obj->flags_.rect_only_except_non_core_pins_ = rect_only_except_non_core_pins;
}

bool dbTechLayer::isRectOnlyExceptNonCorePins() const
{
  _dbTechLayer* obj = (_dbTechLayer*) this;

  return obj->flags_.rect_only_except_non_core_pins_;
}

// User Code Begin dbTechLayerPublicMethods

void dbTechLayer::setLef58Type(LEF58_TYPE type)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->flags_.lef58_type_ = (uint) type;
  if ((type == odb::dbTechLayer::MIMCAP
       || type == odb::dbTechLayer::STACKEDMIMCAP)
      && getType() == dbTechLayerType::ROUTING) {
    _dbTech* tech = (_dbTech*) layer->getOwner();
    layer->_rlevel = 0;
    --tech->_rlayer_cnt;
  }
}

dbTechLayer::LEF58_TYPE dbTechLayer::getLef58Type() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return (dbTechLayer::LEF58_TYPE) layer->flags_.lef58_type_;
}

std::string dbTechLayer::getLef58TypeString() const
{
  switch (getLef58Type()) {
    case NONE:
      return "NONE";
    case NWELL:
      return "NWELL";
    case PWELL:
      return "PWELL";
    case ABOVEDIEEDGE:
      return "ABOVEDIEEDGE";
    case BELOWDIEEDGE:
      return "BELOWDIEEDGE";
    case DIFFUSION:
      return "DIFFUSION";
    case TRIMPOLY:
      return "TRIMPOLY";
    case MIMCAP:
      return "MIMCAP";
    case STACKEDMIMCAP:
      return "STACKEDMIMCAP";
    case TSV:
      return "TSV";
    case PASSIVATION:
      return "PASSIVATION";
    case HIGHR:
      return "HIGHR";
    case TRIMMETAL:
      return "TRIMMETAL";
    case REGION:
      return "REGION";
    case MEOL:
      return "MEOL";
    case WELLDISTANCE:
      return "WELLDISTANCE";
    case CPODE:
      return "CPODE";
    case TSVMETAL:
      return "TSVMETAL";
    case PADMETAL:
      return "PADMETAL";
    case POLYROUTING:
      return "POLYROUTING";
  }

  return "Unknown";
}

std::string dbTechLayer::getName() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_name;
}

const char* dbTechLayer::getConstName() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_name;
}

bool dbTechLayer::hasAlias()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->flags_.has_alias_ == 1;
}

std::string dbTechLayer::getAlias()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;

  if (layer->_alias == nullptr) {
    return "";
  }

  return layer->_alias;
}

void dbTechLayer::setAlias(const char* alias)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;

  if (layer->_alias) {
    free((void*) layer->_alias);
  }

  layer->flags_.has_alias_ = true;
  layer->_alias = safe_strdup(alias);
}

uint dbTechLayer::getWidth() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_width;
}

void dbTechLayer::setWidth(int width)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_width = width;
  if (layer->wrong_way_width_ == 0) {
    layer->wrong_way_width_ = width;
  }
}

int dbTechLayer::getSpacing()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_spacing;
}

void dbTechLayer::setSpacing(int spacing)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_spacing = spacing;
}

double dbTechLayer::getEdgeCapacitance()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_edge_capacitance;
}

void dbTechLayer::setEdgeCapacitance(double cap)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_edge_capacitance = cap;
}

uint dbTechLayer::getWireExtension()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_wire_extension;
}

void dbTechLayer::setWireExtension(uint ext)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_wire_extension = ext;
}

int dbTechLayer::getSpacing(int w, int l)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;

  bool found_spacing = false;
  uint spacing = MAX_INT;

  bool found_over_spacing = false;
  uint over_spacing = MAX_INT;
  uint width = (uint) w;
  uint length = (uint) l;

  for (auto cur_rule : getV54SpacingRules()) {
    uint rmin, rmax;
    if (cur_rule->getRange(rmin, rmax)) {
      if ((width >= rmin) && (width <= rmax)) {
        spacing = std::min(spacing, cur_rule->getSpacing());
        found_spacing = true;
      }
      if (width > rmax) {
        found_over_spacing = true;
        over_spacing = std::min(over_spacing, cur_rule->getSpacing());
      }
    }
  }

  std::vector<std::vector<uint>> v55rules;
  uint i, j;
  if (getV55SpacingTable(v55rules)) {
    for (i = 1; (i < layer->_v55sp_width_idx.size())
                && (width > layer->_v55sp_width_idx[i]);
         i++) {
      ;
    }
    for (j = 1; (j < layer->_v55sp_length_idx.size())
                && (length > layer->_v55sp_length_idx[j]);
         j++) {
      ;
    }
    found_spacing = true;
    spacing = v55rules[i - 1][j - 1];
  }

  if ((!found_spacing) && (found_over_spacing)) {
    found_spacing = true;
    spacing = over_spacing;
  }

  return (found_spacing) ? spacing : layer->_spacing;
}

//
// Get the low end of the uppermost range for wide wire design rules.
//
void dbTechLayer::getMaxWideDRCRange(int& owidth, int& olength)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  dbSet<dbTechLayerSpacingRule> v54rules;

  owidth = getWidth();
  olength = owidth;

  for (auto rule : getV54SpacingRules()) {
    uint rmin, rmax;
    if (rule->getRange(rmin, rmax)) {
      if (rmin > (uint) owidth) {
        owidth = rmin;
        olength = rmin;
      }
    }
  }

  if (hasV55SpacingRules()) {
    owidth = layer->_v55sp_width_idx[layer->_v55sp_width_idx.size() - 1];
    olength = layer->_v55sp_length_idx[layer->_v55sp_length_idx.size() - 1];
  }
}

//
// Get the low end of the lowermost range for wide wire design rules.
//
void dbTechLayer::getMinWideDRCRange(int& owidth, int& olength)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  dbSet<dbTechLayerSpacingRule> v54rules;

  owidth = getWidth();
  olength = owidth;

  for (auto rule : getV54SpacingRules()) {
    uint rmin, rmax;
    if (rule->getRange(rmin, rmax)) {
      if (rmin < (uint) owidth) {
        owidth = rmin;
        olength = rmin;
      }
    }
  }

  if (hasV55SpacingRules()) {
    owidth = layer->_v55sp_width_idx[1];
    olength = layer->_v55sp_length_idx[1];
  }
}

dbSet<dbTechLayerSpacingRule> dbTechLayer::getV54SpacingRules() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return dbSet<dbTechLayerSpacingRule>(layer, layer->_spacing_rules_tbl);
}

bool dbTechLayer::hasV55SpacingRules() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return ((!layer->_v55sp_length_idx.empty())
          && (!layer->_v55sp_width_idx.empty())
          && (layer->_v55sp_spacing.numElems() > 0));
}

bool dbTechLayer::getV55SpacingWidthsAndLengths(
    std::vector<uint>& width_idx,
    std::vector<uint>& length_idx) const
{
  if (!hasV55SpacingRules()) {
    return false;
  }
  _dbTechLayer* layer = (_dbTechLayer*) this;
  width_idx = layer->_v55sp_width_idx;
  length_idx = layer->_v55sp_length_idx;
  return true;
}

void dbTechLayer::printV55SpacingRules(lefout& writer) const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;

  fmt::print(writer.out(), "SPACINGTABLE\n");
  fmt::print(writer.out(), "  PARALLELRUNLENGTH");
  dbVector<uint>::const_iterator v55_itr;
  uint wddx, lndx;

  for (v55_itr = layer->_v55sp_length_idx.begin();
       v55_itr != layer->_v55sp_length_idx.end();
       v55_itr++) {
    fmt::print(writer.out(), " {:.3f}", writer.lefdist(*v55_itr));
  }

  for (wddx = 0, v55_itr = layer->_v55sp_width_idx.begin();
       v55_itr != layer->_v55sp_width_idx.end();
       wddx++, v55_itr++) {
    fmt::print(writer.out(), "\n");
    fmt::print(writer.out(), "  WIDTH {:.3f}\t", writer.lefdist(*v55_itr));
    for (lndx = 0; lndx < layer->_v55sp_spacing.numCols(); lndx++) {
      fmt::print(writer.out(),
                 " {:.3f}",
                 writer.lefdist(layer->_v55sp_spacing(wddx, lndx)));
    }
  }

  fmt::print(writer.out(), " ;\n");
}

bool dbTechLayer::getV55SpacingTable(
    std::vector<std::vector<uint>>& sptbl) const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;

  if (layer->_v55sp_spacing.numElems() == 0) {
    return false;
  }

  uint i, j;
  sptbl.clear();
  sptbl.resize(layer->_v55sp_spacing.numRows());
  std::vector<uint> tmpvec;
  tmpvec.reserve(layer->_v55sp_spacing.numCols());
  for (i = 0; i < layer->_v55sp_spacing.numRows(); i++) {
    tmpvec.clear();
    for (j = 0; j < layer->_v55sp_spacing.numCols(); j++) {
      tmpvec.push_back(layer->_v55sp_spacing(i, j));
    }
    sptbl[i] = tmpvec;
  }

  return true;
}

int dbTechLayer::findV55Spacing(const int width, const int prl) const
{
  if (!hasV55SpacingRules()) {
    return 0;
  }
  _dbTechLayer* layer = (_dbTechLayer*) this;
  uint rowIdx = layer->getV55RowIdx(width);
  uint colIdx = layer->getV55ColIdx(prl);
  return layer->_v55sp_spacing(rowIdx, colIdx);
}

void dbTechLayer::initV55LengthIndex(uint numelems)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_v55sp_length_idx.reserve(numelems);
}

void dbTechLayer::addV55LengthEntry(uint length)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_v55sp_length_idx.push_back(length);
}

void dbTechLayer::initV55WidthIndex(uint numelems)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_v55sp_width_idx.reserve(numelems);
}

void dbTechLayer::addV55WidthEntry(uint width)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_v55sp_width_idx.push_back(width);
}

void dbTechLayer::initV55SpacingTable(uint numrows, uint numcols)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_v55sp_spacing.resize(numrows, numcols);
}

void dbTechLayer::addV55SpacingTableEntry(uint inrow, uint incol, uint spacing)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_v55sp_spacing(inrow, incol) = spacing;
}

bool dbTechLayer::hasTwoWidthsSpacingRules() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return ((!layer->_two_widths_sp_idx.empty())
          && (layer->_two_widths_sp_spacing.numElems() > 0));
}

void dbTechLayer::printTwoWidthsSpacingRules(lefout& writer) const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;

  fmt::print(writer.out(), "SPACINGTABLE TWOWIDTHS");
  dbVector<uint>::const_iterator itr;
  uint wddx, lndx;

  for (wddx = 0, itr = layer->_two_widths_sp_idx.begin();
       itr != layer->_two_widths_sp_idx.end();
       wddx++, itr++) {
    fmt::print(writer.out(), "\n  WIDTH {:.3f}\t", writer.lefdist(*itr));
    for (lndx = 0; lndx < layer->_two_widths_sp_spacing.numCols(); lndx++) {
      fmt::print(writer.out(),
                 " {:.3f}",
                 writer.lefdist(layer->_two_widths_sp_spacing(wddx, lndx)));
    }
  }

  fmt::print(writer.out(), " ;\n");
}

uint dbTechLayer::getTwoWidthsSpacingTableEntry(uint row, uint col) const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_two_widths_sp_spacing(row, col);
}

uint dbTechLayer::getTwoWidthsSpacingTableNumWidths() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_two_widths_sp_idx.size();
}

uint dbTechLayer::getTwoWidthsSpacingTableWidth(uint row) const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_two_widths_sp_idx.at(row);
}

bool dbTechLayer::getTwoWidthsSpacingTableHasPRL(uint row) const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_two_widths_sp_prl.at(row) >= 0;
}

uint dbTechLayer::getTwoWidthsSpacingTablePRL(uint row) const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_two_widths_sp_prl.at(row);
}

bool dbTechLayer::getTwoWidthsSpacingTable(
    std::vector<std::vector<uint>>& sptbl) const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;

  if (layer->_two_widths_sp_spacing.numElems() == 0) {
    return false;
  }

  uint i, j;
  sptbl.clear();
  sptbl.resize(layer->_two_widths_sp_spacing.numRows());
  std::vector<uint> tmpvec;
  tmpvec.reserve(layer->_two_widths_sp_spacing.numCols());
  for (i = 0; i < layer->_two_widths_sp_spacing.numRows(); i++) {
    tmpvec.clear();
    for (j = 0; j < layer->_two_widths_sp_spacing.numCols(); j++) {
      tmpvec.push_back(layer->_two_widths_sp_spacing(i, j));
    }
    sptbl[i] = tmpvec;
  }

  return true;
}

void dbTechLayer::initTwoWidths(uint num_widths)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_two_widths_sp_idx.reserve(num_widths);
  layer->_two_widths_sp_spacing.resize(num_widths, num_widths);
}

void dbTechLayer::addTwoWidthsIndexEntry(uint width, int parallel_run_length)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_two_widths_sp_idx.push_back(width);
  layer->_two_widths_sp_prl.push_back(parallel_run_length);
}

void dbTechLayer::addTwoWidthsSpacingTableEntry(uint inrow,
                                                uint incol,
                                                uint spacing)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_two_widths_sp_spacing(inrow, incol) = spacing;
}

int dbTechLayer::findTwSpacing(const int width1,
                               const int width2,
                               const int prl) const
{
  if (!hasTwoWidthsSpacingRules()) {
    return 0;
  }
  auto reqPrl = std::max(0, prl);
  _dbTechLayer* layer = (_dbTechLayer*) this;
  auto rowIdx = layer->getTwIdx(width1, reqPrl);
  auto colIdx = layer->getTwIdx(width2, reqPrl);
  return layer->_two_widths_sp_spacing(rowIdx, colIdx);
}

bool dbTechLayer::getMinimumCutRules(std::vector<dbTechMinCutRule*>& cut_rules)
{
  cut_rules.clear();

  for (dbTechMinCutRule* rule : getMinCutRules()) {
    cut_rules.push_back(rule);
  }

  return !cut_rules.empty();
}

dbSet<dbTechMinCutRule> dbTechLayer::getMinCutRules()
{
  dbSet<dbTechMinCutRule> rules;
  _dbTechLayer* layer = (_dbTechLayer*) this;
  rules = dbSet<dbTechMinCutRule>(layer, layer->_min_cut_rules_tbl);
  return rules;
}

dbSet<dbTechMinEncRule> dbTechLayer::getMinEncRules()
{
  dbSet<dbTechMinEncRule> rules;
  _dbTechLayer* layer = (_dbTechLayer*) this;
  rules = dbSet<dbTechMinEncRule>(layer, layer->_min_enc_rules_tbl);
  return rules;
}

dbSet<dbTechV55InfluenceEntry> dbTechLayer::getV55InfluenceRules()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return dbSet<dbTechV55InfluenceEntry>(layer, layer->_v55inf_tbl);
}

bool dbTechLayer::getMinEnclosureRules(
    std::vector<dbTechMinEncRule*>& enc_rules)
{
  enc_rules.clear();

  for (dbTechMinEncRule* rule : getMinEncRules()) {
    enc_rules.push_back(rule);
  }

  return !enc_rules.empty();
}

dbTechLayerAntennaRule* dbTechLayer::createDefaultAntennaRule()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  _dbTechLayerAntennaRule* r
      = (_dbTechLayerAntennaRule*) getDefaultAntennaRule();

  // Reinitialize the object to its default state...
  if (r != nullptr) {
    r->~_dbTechLayerAntennaRule();
    new (r) _dbTechLayerAntennaRule(layer->getDatabase());
    r->_layer = getImpl()->getOID();
  } else {
    _dbTech* tech = (_dbTech*) layer->getOwner();
    r = tech->_antenna_rule_tbl->create();
    layer->_oxide1 = r->getOID();
    r->_layer = getImpl()->getOID();
  }

  return (dbTechLayerAntennaRule*) r;
}

dbTechLayerAntennaRule* dbTechLayer::createOxide2AntennaRule()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  _dbTechLayerAntennaRule* r
      = (_dbTechLayerAntennaRule*) getOxide2AntennaRule();

  // Reinitialize the object to its default state...
  if (r != nullptr) {
    r->~_dbTechLayerAntennaRule();
    new (r) _dbTechLayerAntennaRule(layer->getDatabase());
    r->_layer = getImpl()->getOID();
  } else {
    _dbTech* tech = (_dbTech*) layer->getOwner();
    r = tech->_antenna_rule_tbl->create();
    layer->_oxide2 = r->getOID();
    r->_layer = getImpl()->getOID();
  }

  return (dbTechLayerAntennaRule*) r;
}

bool dbTechLayer::hasDefaultAntennaRule() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return (layer->_oxide1 != 0);
}

bool dbTechLayer::hasOxide2AntennaRule() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return (layer->_oxide2 != 0);
}

dbTechLayerAntennaRule* dbTechLayer::getDefaultAntennaRule() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  _dbTech* tech = (_dbTech*) layer->getOwner();

  if (layer->_oxide1 == 0) {
    return nullptr;
  }

  return (dbTechLayerAntennaRule*) tech->_antenna_rule_tbl->getPtr(
      layer->_oxide1);
}

dbTechLayerAntennaRule* dbTechLayer::getOxide2AntennaRule() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  _dbTech* tech = (_dbTech*) layer->getOwner();

  if (layer->_oxide2 == 0) {
    return nullptr;
  }

  return (dbTechLayerAntennaRule*) tech->_antenna_rule_tbl->getPtr(
      layer->_oxide2);
}

void dbTechLayer::writeAntennaRulesLef(lefout& writer) const
{
  bool prt_model = (hasDefaultAntennaRule() && hasOxide2AntennaRule());

  if (prt_model) {
    fmt::print(writer.out(), "    ANTENNAMODEL OXIDE1 ;\n");
  }
  if (hasDefaultAntennaRule()) {
    getDefaultAntennaRule()->writeLef(writer);
  }

  if (prt_model) {
    fmt::print(writer.out(), "    ANTENNAMODEL OXIDE2 ;\n");
  }
  if (hasOxide2AntennaRule()) {
    getOxide2AntennaRule()->writeLef(writer);
  }
}

uint dbTechLayer::getNumMasks() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->flags_.num_masks_;
}

void dbTechLayer::setNumMasks(uint number)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  if (number < 1 || number > 3) {
    getImpl()->getLogger()->error(
        utl::ODB, 282, "setNumMask {} not in range [1,3]", number);
  }
  layer->flags_.num_masks_ = number;
}

bool dbTechLayer::getThickness(uint& inthk) const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  if (layer->flags_.has_thickness_) {
    inthk = layer->_thickness;
    return true;
  }

  return false;
}

void dbTechLayer::setThickness(uint thickness)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->flags_.has_thickness_ = true;
  layer->_thickness = thickness;
}

bool dbTechLayer::hasArea() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return (layer->flags_.has_area_);
}

double  // Now denominated in squm
dbTechLayer::getArea() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  if (layer->flags_.has_area_) {
    return layer->_area;
  }

  return 0.0;  // Default
}

void dbTechLayer::setArea(double area)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->flags_.has_area_ = true;
  layer->_area = area;
}

bool dbTechLayer::hasMaxWidth() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return (layer->flags_.has_max_width_);
}

uint dbTechLayer::getMaxWidth() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  if (layer->flags_.has_max_width_) {
    return layer->_max_width;
  }

  return MAX_INT;  // Default
}

void dbTechLayer::setMaxWidth(uint max_width)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->flags_.has_max_width_ = true;
  layer->_max_width = max_width;
}

uint dbTechLayer::getMinWidth() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_min_width;
}

void dbTechLayer::setMinWidth(uint min_width)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_min_width = min_width;
}

bool dbTechLayer::hasMinStep() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return (layer->_min_step >= 0);
}

uint dbTechLayer::getMinStep() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  if (layer->_min_step >= 0) {
    return layer->_min_step;
  }

  return 0;  // Default
}

void dbTechLayer::setMinStep(uint min_step)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_min_step = min_step;
}

bool dbTechLayer::hasProtrusion() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return (layer->flags_.has_protrusion_);
}

uint dbTechLayer::getProtrusionWidth() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  if (layer->flags_.has_protrusion_) {
    return layer->_pt._width;
  }

  return 0;  // Default
}

uint dbTechLayer::getProtrusionLength() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  if (layer->flags_.has_protrusion_) {
    return layer->_pt._length;
  }

  return 0;  // Default
}

uint dbTechLayer::getProtrusionFromWidth() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  if (layer->flags_.has_protrusion_) {
    return layer->_pt._from_width;
  }

  return 0;  // Default
}

void dbTechLayer::setProtrusion(uint pt_width,
                                uint pt_length,
                                uint pt_from_width)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->flags_.has_protrusion_ = true;
  layer->_pt._width = pt_width;
  layer->_pt._length = pt_length;
  layer->_pt._from_width = pt_from_width;
}

int dbTechLayer::getPitch()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_pitch_x;
}

int dbTechLayer::getPitchX()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_pitch_x;
}

int dbTechLayer::getPitchY()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_pitch_y;
}

int dbTechLayer::getFirstLastPitch()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_first_last_pitch;
}

void dbTechLayer::setPitch(int pitch)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_pitch_x = pitch;
  layer->_pitch_y = pitch;
  layer->flags_.has_xy_pitch_ = false;
}

void dbTechLayer::setPitchXY(int pitch_x, int pitch_y)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_pitch_x = pitch_x;
  layer->_pitch_y = pitch_y;
  layer->flags_.has_xy_pitch_ = true;
}

void dbTechLayer::setFirstLastPitch(int first_last_pitch)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_first_last_pitch = first_last_pitch;
}

bool dbTechLayer::hasXYPitch()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->flags_.has_xy_pitch_;
}

int dbTechLayer::getOffset()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_offset_x;
}

int dbTechLayer::getOffsetX()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_offset_x;
}

int dbTechLayer::getOffsetY()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_offset_y;
}

void dbTechLayer::setOffset(int offset)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_offset_x = offset;
  layer->_offset_y = offset;
  layer->flags_.has_xy_offset_ = false;
}

void dbTechLayer::setOffsetXY(int offset_x, int offset_y)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_offset_x = offset_x;
  layer->_offset_y = offset_y;
  layer->flags_.has_xy_offset_ = true;
}

bool dbTechLayer::hasXYOffset()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->flags_.has_xy_offset_;
}

dbTechLayerDir dbTechLayer::getDirection()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return dbTechLayerDir(layer->flags_.direction_);
}

void dbTechLayer::setDirection(dbTechLayerDir direction)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->flags_.direction_ = direction.getValue();
}

dbTechLayerMinStepType dbTechLayer::getMinStepType() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return dbTechLayerMinStepType(layer->flags_.minstep_type_);
}

void dbTechLayer::setMinStepType(dbTechLayerMinStepType type)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->flags_.minstep_type_ = type.getValue();
}

bool dbTechLayer::hasMinStepMaxLength() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_min_step_max_length >= 0;
}

uint dbTechLayer::getMinStepMaxLength() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_min_step_max_length;
}

void dbTechLayer::setMinStepMaxLength(uint length)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_min_step_max_length = length;
}

bool dbTechLayer::hasMinStepMaxEdges() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_min_step_max_edges >= 0;
}

uint dbTechLayer::getMinStepMaxEdges() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_min_step_max_edges;
}

void dbTechLayer::setMinStepMaxEdges(uint edges)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_min_step_max_edges = edges;
}

dbTechLayerType dbTechLayer::getType()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return dbTechLayerType(layer->flags_.type_);
}

double dbTechLayer::getResistance()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_resistance;
}

void dbTechLayer::setResistance(double resistance)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_resistance = resistance;
}

double dbTechLayer::getCapacitance()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_capacitance;
}

void dbTechLayer::setCapacitance(double capacitance)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->_capacitance = capacitance;
}

int dbTechLayer::getNumber() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_number;
}

int dbTechLayer::getRoutingLevel()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return layer->_rlevel;
}

dbTechLayer* dbTechLayer::getLowerLayer()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  _dbTech* tech = (_dbTech*) layer->getOwner();

  if (layer->_lower == 0) {
    return nullptr;
  }

  return (dbTechLayer*) tech->_layer_tbl->getPtr(layer->_lower);
}

dbTechLayer* dbTechLayer::getUpperLayer()
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  _dbTech* tech = (_dbTech*) layer->getOwner();

  if (layer->_upper == 0) {
    return nullptr;
  }

  return (dbTechLayer*) tech->_layer_tbl->getPtr(layer->_upper);
}

dbTech* dbTechLayer::getTech() const
{
  return (dbTech*) getImpl()->getOwner();
}

bool dbTechLayer::hasOrthSpacingTable() const
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  return !layer->orth_spacing_tbl_.empty();
}

void dbTechLayer::addOrthSpacingTableEntry(const int within, const int spacing)
{
  _dbTechLayer* layer = (_dbTechLayer*) this;
  layer->orth_spacing_tbl_.emplace_back(within, spacing);
}

dbTechLayer* dbTechLayer::create(dbTech* tech_,
                                 const char* name_,
                                 dbTechLayerType type)
{
  if (type.getValue() == dbTechLayerType::NONE) {
    return nullptr;
  }

  if (tech_->findLayer(name_)) {
    return nullptr;
  }

  _dbTech* tech = (_dbTech*) tech_;
  _dbTechLayer* layer = tech->_layer_tbl->create();
  layer->_name = safe_strdup(name_);
  layer->_number = tech->_layer_cnt++;
  layer->flags_.type_ = type.getValue();

  if (type.getValue() == dbTechLayerType::ROUTING) {
    layer->_rlevel = ++tech->_rlayer_cnt;
  }

  if (tech->_bottom == 0) {
    tech->_bottom = layer->getOID();
    tech->_top = layer->getOID();
    return (dbTechLayer*) layer;
  }

  _dbTechLayer* top = tech->_layer_tbl->getPtr(tech->_top);
  top->_upper = layer->getOID();
  layer->_lower = top->getOID();
  tech->_top = layer->getOID();

  return (dbTechLayer*) layer;
}

dbTechLayer* dbTechLayer::getTechLayer(dbTech* tech_, uint dbid_)
{
  _dbTech* tech = (_dbTech*) tech_;
  return (dbTechLayer*) tech->_layer_tbl->getPtr(dbid_);
}

// User Code End dbTechLayerPublicMethods
}  // namespace odb
   // Generator Code End Cpp
