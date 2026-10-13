#pragma once

#include "mark/detector_config.hpp"
#include "mark/detector_types.hpp"
#include "core/prepared_frame.hpp"

namespace mark
{

    PreparedFrame preprocess(
        const FrameInput &frame,
        const PreprocessConfig &config);

} // namespace mark