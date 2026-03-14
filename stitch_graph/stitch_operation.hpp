#ifndef YARNPATH_STITCH_OPERATION_HPP
#define YARNPATH_STITCH_OPERATION_HPP

#include <cstdint>
#include <variant>
#include <vector>

namespace yarnpath {

using StitchId = uint32_t;

/// Per-node stitch operations in the StitchGraph.
///
/// These mirror the instruction types but are per-node (no counts, no repeats).
/// Each node in the graph holds exactly one StitchOperation.
/// Cable operations additionally store the specific StitchIds involved,
/// recording which stitches were held and which were crossed.
namespace stitch {

struct Knit {};
struct Purl {};
struct Slip {};
struct CastOn {};
struct BindOff {};
struct YarnOver {};
struct KFB {};
struct M1L {};
struct M1R {};
struct K2tog {};
struct SSK {};
struct S2KP {};

/// Cable operations store the specific StitchIds of the held and crossed groups.
struct CableLeft {
    std::vector<StitchId> held;    ///< Stitches held to front.
    std::vector<StitchId> crossed; ///< Stitches crossed behind (worked first).
};

struct CableRight {
    std::vector<StitchId> held;    ///< Stitches held to back.
    std::vector<StitchId> crossed; ///< Stitches crossed in front (worked second).
};

}  // namespace stitch

using StitchOperation = std::variant<
    stitch::Knit, stitch::Purl, stitch::Slip,
    stitch::CastOn, stitch::BindOff,
    stitch::YarnOver, stitch::KFB, stitch::M1L, stitch::M1R,
    stitch::K2tog, stitch::SSK, stitch::S2KP,
    stitch::CableLeft, stitch::CableRight
>;

}  // namespace yarnpath

#endif // YARNPATH_STITCH_OPERATION_HPP
