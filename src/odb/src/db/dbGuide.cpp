// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2020-2025, The OpenROAD Authors

// Generator Code Begin Cpp
#include "dbGuide.h"

#include "dbDatabase.h"
#include "dbNet.h"
#include "dbTable.h"
#include "dbTable.hpp"
#include "dbTechLayer.h"
#include "odb/db.h"
// User Code Begin Includes
#include "dbBlock.h"
#include "dbJournal.h"
// User Code End Includes
namespace odb {
template class dbTable<_dbGuide>;

bool _dbGuide::operator==(const _dbGuide& rhs) const
{
  if (net_ != rhs.net_) {
    return false;
  }
  if (box_ != rhs.box_) {
    return false;
  }
  if (layer_ != rhs.layer_) {
    return false;
  }
  if (via_layer_ != rhs.via_layer_) {
    return false;
  }
  if (guide_next_ != rhs.guide_next_) {
    return false;
  }
  if (is_congested_ != rhs.is_congested_) {
    return false;
  }
  if (is_jumper_ != rhs.is_jumper_) {
    return false;
  }
  if (is_connect_to_term_ != rhs.is_connect_to_term_) {
    return false;
  }

  return true;
}

bool _dbGuide::operator<(const _dbGuide& rhs) const
{
  return true;
}

_dbGuide::_dbGuide(_dbDatabase* db)
{
  is_congested_ = false;
  is_jumper_ = false;
  is_connect_to_term_ = false;
}

dbIStream& operator>>(dbIStream& stream, _dbGuide& obj)
{
  stream >> obj.net_;
  stream >> obj.box_;
  stream >> obj.layer_;
  if (obj.getDatabase()->isSchema(db_schema_db_guide_via_layer)) {
    stream >> obj.via_layer_;
  }
  stream >> obj.guide_next_;
  if (obj.getDatabase()->isSchema(db_schema_db_guide_congested)) {
    stream >> obj.is_congested_;
  }
  if (obj.getDatabase()->isSchema(db_schema_has_jumpers)) {
    stream >> obj.is_jumper_;
  }
  if (obj.getDatabase()->isSchema(db_schema_guide_connected_to_term)) {
    stream >> obj.is_connect_to_term_;
  }
  return stream;
}

dbOStream& operator<<(dbOStream& stream, const _dbGuide& obj)
{
  stream << obj.net_;
  stream << obj.box_;
  stream << obj.layer_;
  stream << obj.via_layer_;
  stream << obj.guide_next_;
  stream << obj.is_congested_;
  stream << obj.is_jumper_;
  stream << obj.is_connect_to_term_;
  return stream;
}

void _dbGuide::collectMemInfo(MemInfo& info)
{
  info.cnt++;
  info.size += sizeof(*this);
}

////////////////////////////////////////////////////////////////////
//
// dbGuide - Methods
//
////////////////////////////////////////////////////////////////////

Rect dbGuide::getBox() const
{
  _dbGuide* obj = (_dbGuide*) this;
  return obj->box_;
}

// User Code Begin dbGuidePublicMethods

dbTechLayer* dbGuide::getLayer() const
{
  _dbGuide* obj = (_dbGuide*) this;
  auto tech = getDb()->getTech();
  return odb::dbTechLayer::getTechLayer(tech, obj->layer_);
}

dbTechLayer* dbGuide::getViaLayer() const
{
  _dbGuide* obj = (_dbGuide*) this;
  auto tech = getDb()->getTech();
  return odb::dbTechLayer::getTechLayer(tech, obj->via_layer_);
}

bool dbGuide::isCongested() const
{
  _dbGuide* obj = (_dbGuide*) this;
  return obj->is_congested_;
}

dbNet* dbGuide::getNet() const
{
  _dbGuide* obj = (_dbGuide*) this;
  _dbBlock* block = (_dbBlock*) obj->getOwner();
  return (dbNet*) block->_net_tbl->getPtr(obj->net_);
}

dbGuide* dbGuide::create(dbNet* net,
                         dbTechLayer* layer,
                         dbTechLayer* via_layer,
                         Rect box,
                         bool is_congested)
{
  _dbNet* owner = (_dbNet*) net;
  _dbBlock* block = (_dbBlock*) owner->getOwner();
  _dbGuide* guide = block->_guide_tbl->create();

  if (block->_journal) {
    debugPrint(block->getImpl()->getLogger(),
               utl::ODB,
               "DB_ECO",
               1,
               "ECO: create guide, layer {} box {}",
               layer->getName(),
               box);
    block->_journal->beginAction(dbJournal::CREATE_OBJECT);
    block->_journal->pushParam(dbGuideObj);
    block->_journal->pushParam(guide->getOID());
    block->_journal->endAction();
  }

  guide->layer_ = layer->getImpl()->getOID();
  guide->via_layer_ = via_layer->getImpl()->getOID();
  guide->box_ = box;
  guide->net_ = owner->getId();
  guide->is_congested_ = is_congested;
  guide->guide_next_ = owner->guides_;
  guide->is_jumper_ = false;
  owner->guides_ = guide->getOID();
  return (dbGuide*) guide;
}

dbGuide* dbGuide::getGuide(dbBlock* block, uint dbid)
{
  _dbBlock* owner = (_dbBlock*) block;
  return (dbGuide*) owner->_guide_tbl->getPtr(dbid);
}

void dbGuide::destroy(dbGuide* guide)
{
  _dbBlock* block = (_dbBlock*) guide->getImpl()->getOwner();
  _dbNet* net = (_dbNet*) guide->getNet();
  _dbGuide* _guide = (_dbGuide*) guide;

  if (block->_journal) {
    debugPrint(block->getImpl()->getLogger(),
               utl::ODB,
               "DB_ECO",
               1,
               "ECO: destroy guide, id: {}",
               guide->getId());
    block->_journal->beginAction(dbJournal::DELETE_OBJECT);
    block->_journal->pushParam(dbGuideObj);
    block->_journal->pushParam(net->getOID());
    block->_journal->pushParam(_guide->box_.xMin());
    block->_journal->pushParam(_guide->box_.yMin());
    block->_journal->pushParam(_guide->box_.xMax());
    block->_journal->pushParam(_guide->box_.yMax());
    block->_journal->pushParam(_guide->layer_);
    block->_journal->pushParam(_guide->via_layer_);
    block->_journal->pushParam(_guide->is_congested_);
    block->_journal->endAction();
  }

  uint id = _guide->getOID();
  _dbGuide* prev = nullptr;
  uint cur = net->guides_;
  while (cur) {
    _dbGuide* c = block->_guide_tbl->getPtr(cur);
    if (cur == id) {
      if (prev == nullptr) {
        net->guides_ = _guide->guide_next_;
      } else {
        prev->guide_next_ = _guide->guide_next_;
      }
      break;
    }
    prev = c;
    cur = c->guide_next_;
  }

  dbProperty::destroyProperties(guide);
  block->_guide_tbl->destroy((_dbGuide*) guide);
}

dbSet<dbGuide>::iterator dbGuide::destroy(dbSet<dbGuide>::iterator& itr)
{
  dbGuide* g = *itr;
  dbSet<dbGuide>::iterator next = ++itr;
  destroy(g);
  return next;
}

bool dbGuide::isJumper() const
{
  bool is_jumper = false;
  _dbGuide* guide = (_dbGuide*) this;
  _dbDatabase* db = guide->getDatabase();
  if (db->isSchema(db_schema_has_jumpers)) {
    is_jumper = guide->is_jumper_;
  }
  return is_jumper;
}

void dbGuide::setIsJumper(bool jumper)
{
  _dbGuide* guide = (_dbGuide*) this;
  _dbDatabase* db = guide->getDatabase();
  if (db->isSchema(db_schema_has_jumpers)) {
    guide->is_jumper_ = jumper;
  }
}

bool dbGuide::isConnectedToTerm() const
{
  bool is_connected_to_term = false;
  _dbGuide* guide = (_dbGuide*) this;
  is_connected_to_term = guide->is_connect_to_term_;
  return is_connected_to_term;
}

void dbGuide::setIsConnectedToTerm(bool is_connected)
{
  _dbGuide* guide = (_dbGuide*) this;
  guide->is_connect_to_term_ = is_connected;
}

// User Code End dbGuidePublicMethods
}  // namespace odb
// Generator Code End Cpp
