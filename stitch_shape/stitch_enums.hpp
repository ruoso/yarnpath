#ifndef YARNPATH_STITCH_SHAPE_STITCH_ENUMS_HPP
#define YARNPATH_STITCH_SHAPE_STITCH_ENUMS_HPP

namespace yarnpath {

// Loop orientation - which direction the loop faces
enum class LoopOrientation {
    Front,      // Knit - loop faces front of fabric
    Back,       // Purl - loop faces back of fabric
    Neutral     // CastOn, YarnOver, etc.
};

// Wrap/twist direction for decreases and increases
enum class WrapDirection {
    Clockwise,        // Right-leaning (K2tog, M1R)
    CounterClockwise, // Left-leaning (SSK, M1L)
    None              // Not applicable
};

// Work semantics - how the yarn interacts with parent loops
enum class WorkType {
    Worked,      // Yarn goes through and forms new loop (Knit/Purl)
    Transferred, // Loop transferred without working (Slip)
    Created      // New loop created without parent (CastOn, YarnOver, M1L/M1R)
};

}  // namespace yarnpath

#endif // YARNPATH_STITCH_SHAPE_STITCH_ENUMS_HPP
