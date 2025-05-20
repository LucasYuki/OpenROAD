#include "epl/EPlace.h"

#include "EDensity.h"
#include "sta/StaMain.hh"

namespace sta {
// Tcl files encoded into strings.
extern const char* eplace_tcl_inits[];
}  // namespace sta

namespace epl {

extern "C" {
extern int Epl_Init(Tcl_Interp* interp);
}

EPlace::EPlace() : e_density_(nullptr)
{
}

EPlace::~EPlace()
{
  if (e_density_) {
    delete e_density_;
  }
}

void EPlace::init(odb::dbDatabase* db, utl::Logger* logger)
{
  db_ = db;
  log_ = logger;
  e_density_ = new EDensity(db_, log_);
}

}  // namespace epl
