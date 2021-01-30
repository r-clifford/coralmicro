#ifndef THIRD_PARTY_DARWINN_DRIVER_CONFIG_CHIP_STRUCTURES_H_
#define THIRD_PARTY_DARWINN_DRIVER_CONFIG_CHIP_STRUCTURES_H_

#include "third_party/darwinn/port/integral_types.h"

namespace platforms {
namespace darwinn {
namespace driver {
namespace config {

// This struct holds various values that is derived from system_builder.proto.
struct ChipStructures {
  // Hardware required minimum alignment on buffers.
  uint64 minimum_alignment_bytes;

  // Buffer allocation alignment and granularity. Typically this would be same
  // as minimum_alignment_bytes above, however may also factor in other
  // requirements such as host cache line size, cache API constraints etc.
  uint64 allocation_alignment_bytes;

  // Controls AXI burst length.
  uint64 axi_dma_burst_limiter;

  // Number of wire interrupts.
  uint64 num_wire_interrupts;

  // Number of page table entries.
  uint64 num_page_table_entries;

  // Number of physical address bits generated by the hardware.
  uint64 physical_address_bits;

  // Addressable byte size of TPU DRAM (if any). This must be divisible by host
  // table size.
  uint64 tpu_dram_size_bytes;
};

}  // namespace config
}  // namespace driver
}  // namespace darwinn
}  // namespace platforms

#endif  // THIRD_PARTY_DARWINN_DRIVER_CONFIG_CHIP_STRUCTURES_H_