// -------------------------------------------------------------------------
//                            impconst.h
// Typy i stale wykorzystywane w MRROC++
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#if !defined(_POLYCRANK_CONST_H)
#define _POLYCRANK_CONST_H

#include <stdint.h>

namespace mrrocpp
{

}


using namespace mrrocpp;


namespace mrrocpp {
namespace lib {


#ifdef __cplusplus
extern "C" {
#endif



#define EDP_POLYCRACNK_SECTION "[edp_polycrank]"
#define ECP_POLYCRACNK_SECTION "[ecp_polycrank]"

#define POLYCRANK_NUM_OF_SERVOS	8

#ifdef __cplusplus
}
#endif

} // namespace lib
} // namespace mrrocpp

#endif /* _IMPCONST_H */