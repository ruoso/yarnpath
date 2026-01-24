#ifndef YARNPATH_STITCH_OPERATION_HPP
#define YARNPATH_STITCH_OPERATION_HPP

#include <cstdint>
#include <variant>
#include <vector>

namespace yarnpath {

using StitchId = uint32_t;

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

// Cables: which specific stitches crossed
struct CableLeft {
    std::vector<StitchId> held;    // Stitches held to front
    std::vector<StitchId> crossed; // Stitches worked first
};

struct CableRight {
    std::vector<StitchId> held;    // Stitches held to back
    std::vector<StitchId> crossed; // Stitches worked first
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
