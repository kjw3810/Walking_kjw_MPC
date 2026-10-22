// LoggingThread (Th5): on a "save start"/"save end" command (see
// CommandInputThread), snapshots DataWarehouse to a CSV file.
// See Controller/docs/architecture.md sections 3 and 8.5 (format was
// unspecified in the original diagram; CSV chosen here as the simplest
// option that satisfies "save 트리거를 받아 DataWarehouse 상태를 기록").
#pragma once

#include <atomic>

#include "data_warehouse.h"

namespace walking_kjw {

void RunLoggingThread(DataWarehouse& dw, std::atomic<bool>& stop);

}  // namespace walking_kjw
