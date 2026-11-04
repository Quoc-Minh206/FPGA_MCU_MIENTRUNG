// =============================================================================
// File:        Project_FPGA_2026.sdc
// Thiết bị:    GW1NSR-LV4CQN48PC6/I5 (Kit OneKiwi 4K / Tang Nano 4K)
// Mô tả:       Ràng buộc định thời tĩnh chuẩn Gowin Timing Analyzer
//               Hệ thống đồng bộ trên xung nhịp duy nhất được sinh từ PLLVR.
// =============================================================================

// 1. Xung nhịp gốc 27 MHz từ chân thạch anh onboard
create_clock -name sys_clkin -period 37.037 -waveform {0 18.518} [get_ports {sys_clkin}]
