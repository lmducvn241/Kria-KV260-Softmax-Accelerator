# AXI4-Lite Register Map — Softmax IP v6.0

| Offset | Tên | R/W | Mô tả |
|--------|-----|-----|-------|
| 0x00   | CTRL        | W   | [0]=start_p1, [1]=start_p2, [2]=clear_error, [3]=auto_p2_en |
| 0x04   | STATUS      | R   | [0]=busy, [1]=p1_done, [2]=p2_done, [3]=error |
| 0x08   | K_CONFIG    | R/W | Số phần tử K (phải là bội số của 8) |
| 0x0C   | CYCLE_COUNT | R   | Tổng chu kỳ xử lý gần nhất |
