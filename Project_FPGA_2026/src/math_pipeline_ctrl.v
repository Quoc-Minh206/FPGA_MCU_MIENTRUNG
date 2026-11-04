// =============================================================================
// Tên Module:   math_pipeline_ctrl
// Dự án:        Hệ thống Edge AI Phát Hiện Tai Nạn Xe Máy (Motorcycle Accident Detection)
// Phân hệ:      FPGA Math Core & Feature Extraction
// Kỹ sư:        Vương Thành Luân
// Nền tảng:     Gowin LittleBee GW1NSR-LV4CQN48PC6/I5 (OneKiwi 4K)
// Mô tả:        Bộ điều khiển trung tâm (Master Orchestrator) quản lý toàn bộ luồng
//               dữ liệu và đồng bộ hóa pipeline:
//               1. Bắt tay an toàn với CDC FIFO Buffer (Minh) qua fifo_rd_en / fifo_valid.
//               2. Quản lý bộ đếm cửa sổ khối N = 32 mẫu và phát xung pipe_window_last.
//               3. Đồng bộ hóa đa nhánh Barrier Sync (chờ cả CORDIC và Stat hoàn tất).
//               4. Đóng gói vector 4 đặc trưng và giao tiếp bắt tay với SPI Master (Thắng)
//                  kèm cơ chế xử lý nghẽn dòng (Backpressure Handling).
// =============================================================================

`timescale 1ns / 1ps

module math_pipeline_ctrl (
    // -------------------------------------------------------------------------
    // CỔNG GIAO TIẾP HỆ THỐNG (SYSTEM CLOCK & RESET)
    // -------------------------------------------------------------------------
    input  wire        clk,               // Xung nhịp hệ thống 50 MHz
    input  wire        rst_n,             // Reset tích cực mức thấp (Active-Low)

    // -------------------------------------------------------------------------
    // GIAO TIẾP NGOÀI: BỘ ĐỆM CDC FIFO BUFFER (KẾT NỐI VỚI PHÂN HỆ CỦA MINH)
    // -------------------------------------------------------------------------
    input  wire        fifo_empty,        // Cờ báo FIFO rỗng (Active-High). Chỉ đọc khi fifo_empty == 0
    input  wire        fifo_valid,        // Cờ báo dữ liệu từ FIFO đang hợp lệ (trễ 1 cycle sau rd_en)
    output reg         fifo_rd_en,        // Lệnh yêu cầu đọc mẫu tiếp theo từ FIFO
    input  wire signed [15:0] fifo_data_acc_x, // Dữ liệu gia tốc trục X từ FIFO (Signed Q4.12)
    input  wire signed [15:0] fifo_data_acc_y, // Dữ liệu gia tốc trục Y từ FIFO (Signed Q4.12)
    input  wire signed [15:0] fifo_data_acc_z, // Dữ liệu gia tốc trục Z từ FIFO (Signed Q4.12)

    // -------------------------------------------------------------------------
    // GIAO TIẾP NGOÀI: KHỐI SPI MASTER (KẾT NỐI VỚI PHÂN HỆ CỦA THẮNG)
    // -------------------------------------------------------------------------
    input  wire        spi_busy,          // Cờ báo khối SPI đang bận đẩy gói tin cũ sang MCU (Active-High)
    output reg         feature_valid,     // Xung 1 chu kỳ clock báo gói 4 đặc trưng sẵn sàng nạp vào thanh ghi dịch SPI
    output reg  signed [15:0] feature_pitch, // Đặc trưng góc Pitch ước lượng (Signed Q8.8, độ)
    output reg  signed [15:0] feature_roll,  // Đặc trưng góc Roll ước lượng (Signed Q8.8, độ)
    output reg         [15:0] feature_rms,   // Đặc trưng năng lượng hiệu dụng RMS^2 (Unsigned Q8.8)
    output reg         [15:0] feature_var,   // Đặc trưng phương sai dao động (Unsigned Q12.4)

    // -------------------------------------------------------------------------
    // GIAO TIẾP NỘI BỘ: PHÂN PHỐI TỚI 3 SUB-MODULES
    // -------------------------------------------------------------------------
    output wire        ctrl_ce,           // Tín hiệu Clock-Enable toàn cục cho các sub-modules
    output reg         pipe_sample_valid, // Xung 1 chu kỳ clock báo mẫu mới đã được chốt và hợp lệ
    output reg         pipe_window_last,  // Xung 1 chu kỳ clock báo mẫu thứ 32 (mẫu cuối cửa sổ) cho stat_feature_extractor
    output reg  signed [15:0] pipe_acc_x, // Gia tốc trục X đã chốt thanh ghi, fan-out tới threshold và cordic
    output reg  signed [15:0] pipe_acc_y, // Gia tốc trục Y đã chốt thanh ghi, fan-out tới threshold và cordic
    output reg  signed [15:0] pipe_acc_z, // Gia tốc trục Z đã chốt thanh ghi, fan-out tới threshold và cordic

    // -------------------------------------------------------------------------
    // GIAO TIẾP NỘI BỘ: NHẬN KẾT QUẢ TỪ cordic_attitude_estimator
    // -------------------------------------------------------------------------
    input  wire        cordic_done,       // Xung báo khối CORDIC hoàn tất (sau 32 chu kỳ)
    input  wire signed [15:0] cordic_roll_out,  // Góc Roll tính toán được (Signed Q8.8)
    input  wire signed [15:0] cordic_pitch_out, // Góc Pitch tính toán được (Signed Q8.8)

    // -------------------------------------------------------------------------
    // GIAO TIẾP NỘI BỘ: NHẬN KẾT QUẢ TỪ stat_feature_extractor
    // -------------------------------------------------------------------------
    input  wire        stat_done,         // Xung báo khối Thống kê hoàn tất (sau 4 chu kỳ post-window)
    input  wire [15:0] stat_rms_out,      // Năng lượng hiệu dụng RMS^2 (Unsigned Q8.8)
    input  wire [15:0] stat_var_out       // Phương sai dao động (Unsigned Q12.4)
);

    // =========================================================================
    // ĐỊNH NGHĨA CÁC TRẠNG THÁI FSM (ONE-HOT ENCODING)
    // =========================================================================
    // Sử dụng mã hóa One-Hot giúp tiết kiệm logic giải mã và tối ưu hóa timing
    localparam [4:0]
        ST_IDLE          = 5'b00001, // Trạng thái nghỉ: Chờ FIFO có mẫu mới và hệ thống rảnh
        ST_READ_FIFO     = 5'b00010, // Trạng thái phát xung đọc: Kích hoạt fifo_rd_en = 1
        ST_DISPATCH      = 5'b00100, // Trạng thái chốt & phân phối: Phát pipe_sample_valid cho 3 sub-modules
        ST_WAIT_PIPELINE = 5'b01000, // Trạng thái rào cản đồng bộ: Chờ cả CORDIC và Stat hoàn tất
        ST_OUTPUT_STROBE = 5'b10000; // Trạng thái xuất gói đặc trưng: Bắn xung feature_valid cho SPI

    reg [4:0] state_reg, state_next;

    // Bộ đếm số lượng mẫu đã nạp trong cửa sổ N = 32 (giá trị từ 0 đến 31)
    reg [4:0] window_cnt;

    // Các cờ chốt (Latch Flags) phục vụ rào cản đồng bộ Barrier Synchronization
    // Do CORDIC và Stat kết thúc ở các thời điểm lệch nhau, cần cờ latch để ghi nhớ
    reg cordic_latch;
    reg stat_latch;

    // Clock-Enable toàn cục luôn duy trì mức 1 khi hệ thống hoạt động
    assign ctrl_ce = 1'b1;

    // =========================================================================
    // KHỐI 1: CHUYỂN TRẠNG THÁI FSM ĐỒNG BỘ THEO XUNG NHỊP (STATE REGISTER)
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_reg <= ST_IDLE;
        end else begin
            state_reg <= state_next;
        end
    end

    // =========================================================================
    // KHỐI 2: LOGIC TÍNH TOÁN TRẠNG THÁI TIẾP THEO (NEXT STATE LOGIC)
    // =========================================================================
    always @(*) begin
        state_next = state_reg;
        case (state_reg)
            // -----------------------------------------------------------------
            // ST_IDLE: Kiểm tra FIFO có dữ liệu không
            // -----------------------------------------------------------------
            ST_IDLE: begin
                // Nếu FIFO không rỗng -> Chuyển sang đọc FIFO
                if (!fifo_empty)
                    state_next = ST_READ_FIFO;
            end

            // -----------------------------------------------------------------
            // ST_READ_FIFO: Chờ FIFO phản hồi dữ liệu hợp lệ (fifo_valid)
            // -----------------------------------------------------------------
            ST_READ_FIFO: begin
                if (fifo_valid)
                    state_next = ST_DISPATCH;
            end

            // -----------------------------------------------------------------
            // ST_DISPATCH: Phân phối mẫu mới vào pipeline
            // -----------------------------------------------------------------
            ST_DISPATCH: begin
                // Nếu vừa nạp xong mẫu thứ 32 (window_cnt == 31) -> Chuyển sang chờ pipeline
                // Nếu chưa đủ 32 mẫu -> Quay về ST_IDLE đón mẫu tiếp theo
                if (window_cnt == 5'd31)
                    state_next = ST_WAIT_PIPELINE;
                else
                    state_next = ST_IDLE;
            end

            // -----------------------------------------------------------------
            // ST_WAIT_PIPELINE: Rào cản đồng bộ Barrier Sync
            // -----------------------------------------------------------------
            ST_WAIT_PIPELINE: begin
                // Chỉ chuyển sang xuất kết quả khi CẢ CORDIC VÀ Stat đều đã xong
                if ((cordic_latch || cordic_done) && (stat_latch || stat_done))
                    state_next = ST_OUTPUT_STROBE;
            end

            // -----------------------------------------------------------------
            // ST_OUTPUT_STROBE: Xuất gói đặc trưng sang SPI Master
            // -----------------------------------------------------------------
            ST_OUTPUT_STROBE: begin
                // Nếu SPI không bận -> Bắn xong xung feature_valid và quay lại ST_IDLE
                // Nếu SPI đang bận -> Tiếp tục dừng tại ST_OUTPUT_STROBE chờ SPI rảnh
                if (!spi_busy)
                    state_next = ST_IDLE;
            end

            default: state_next = ST_IDLE;
        endcase
    end

    // =========================================================================
    // KHỐI 3: QUẢN LÝ BỘ ĐẾM CỬA SỔ & CỜ LATCH ĐỒNG BỘ
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            window_cnt   <= 5'd0;
            cordic_latch <= 1'b0;
            stat_latch   <= 1'b0;
        end else begin
            case (state_reg)
                ST_DISPATCH: begin
                    // Tăng bộ đếm mẫu mỗi khi phân phối thành công một mẫu mới
                    if (window_cnt == 5'd31) begin
                        window_cnt   <= 5'd0; // Reset đếm sau khi đủ cửa sổ 32 mẫu
                        cordic_latch <= 1'b0; // Xóa sạch cờ cũ trước khi bước vào ST_WAIT_PIPELINE
                        stat_latch   <= 1'b0;
                    end else begin
                        window_cnt <= window_cnt + 1'b1;
                    end
                end

                ST_WAIT_PIPELINE: begin
                    // CHỈ bắt cờ hoàn tất khi FSM đang thực sự đứng chờ pipeline mẫu chốt cửa sổ (mẫu 31)
                    if (cordic_done) 
                        cordic_latch <= 1'b1;
                    if (stat_done)   
                        stat_latch   <= 1'b1;
                end

                ST_OUTPUT_STROBE: begin
                    // Khi đã bàn giao xong gói đặc trưng sang SPI thành công -> Xóa cờ latch
                    if (!spi_busy) begin
                        cordic_latch <= 1'b0;
                        stat_latch   <= 1'b0;
                    end
                end

                default: begin
                    // Ở các trạng thái khác (ST_IDLE, ST_READ_FIFO), giữ cờ latch luôn ở mức 0
                    cordic_latch <= 1'b0;
                    stat_latch   <= 1'b0;
                end
            endcase
        end
    end

    // =========================================================================
    // KHỐI 4: BẮT TAY ĐỌC FIFO & PHÂN PHỐI DỮ LIỆU NỘI BỘ
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_rd_en        <= 1'b0;
            pipe_sample_valid <= 1'b0;
            pipe_window_last  <= 1'b0;
            pipe_acc_x        <= 16'sd0;
            pipe_acc_y        <= 16'sd0;
            pipe_acc_z        <= 16'sd0;
        end else begin
            // Mặc định các xung điều khiển về 0 (Pulse generation)
            fifo_rd_en        <= 1'b0;
            pipe_sample_valid <= 1'b0;
            pipe_window_last  <= 1'b0;

            case (state_reg)
                ST_IDLE: begin
                    // Bắn xung đọc FIFO khi có mẫu mới
                    if (!fifo_empty)
                        fifo_rd_en <= 1'b1;
                end

                ST_READ_FIFO: begin
                    // Khi FIFO phản hồi dữ liệu hợp lệ -> Chốt dữ liệu và broadcast
                    if (fifo_valid) begin
                        pipe_acc_x        <= fifo_data_acc_x;
                        pipe_acc_y        <= fifo_data_acc_y;
                        pipe_acc_z        <= fifo_data_acc_z;
                        pipe_sample_valid <= 1'b1;

                        // Nếu là mẫu thứ 32 -> Bật cờ pipe_window_last báo cho stat_feature_extractor
                        if (window_cnt == 5'd31)
                            pipe_window_last <= 1'b1;
                    end
                end

                default: ;
            endcase
        end
    end

    // =========================================================================
    // KHỐI 5: CHỐT GÓI 4 ĐẶC TRƯNG & XUẤT XUNG BẮT TAY SPI
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            feature_valid <= 1'b0;
            feature_pitch <= 16'sd0;
            feature_roll  <= 16'sd0;
            feature_rms   <= 16'd0;
            feature_var   <= 16'd0;
        end else begin
            feature_valid <= 1'b0; // Mặc định feature_valid là xung 1 chu kỳ clock

            if (state_reg == ST_WAIT_PIPELINE) begin
                // Chốt trước kết quả từ các nhánh tính toán khi từng nhánh hoàn tất
                if (cordic_done || cordic_latch) begin
                    feature_pitch <= cordic_pitch_out;
                    feature_roll  <= cordic_roll_out;
                end
                if (stat_done || stat_latch) begin
                    feature_rms <= stat_rms_out;
                    feature_var <= stat_var_out;
                end
            end else if (state_reg == ST_OUTPUT_STROBE) begin
                // Khi SPI không bận -> Bắn xung feature_valid = 1 đúng 1 chu kỳ để nạp vào SPI
                if (!spi_busy) begin
                    feature_valid <= 1'b1;
                end
            end
        end
    end

endmodule
