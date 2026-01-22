# Development Log: STM32 EKF Flight Controller

**Project:** Real-time quaternion-based attitude estimation and flight control  
**Hardware:** STM32F411 (Nucleo-64), ST-Link V2, Simulated IMU via UART  
**Software Stack:** Custom EKF implementation, PID control loop, HAL UART with DMA  
**Objective:** Hardware-accelerated flight stabilization for FlightGear simulator

---

## Technical Challenges and Solutions

### 1. UART Receive Interrupt Configuration

**Problem:** UART transmit functionality operational, but receive interrupts failed to trigger despite proper DMA initialization.

**Root Cause:** NVIC (Nested Vectored Interrupt Controller) for USART2 was not enabled. The peripheral generated interrupt signals, but the CPU interrupt handler never received them.

**Solution:** Enabled USART2 global interrupt in NVIC configuration via STM32CubeMX.

**Lesson:** Peripheral initialization requires both hardware configuration (DMA, GPIO, clocks) and interrupt routing (NVIC). CubeMX sometimes does not auto-enable NVIC when adding DMA.

---

### 2. String Buffer Sizing in Embedded C

**Problem:** Data truncation and corruption in received UART packets.

**Root Cause:** Incorrect assumptions about string handling in embedded C versus Python/C++. Initial `MESSAGE_LEN` definition was insufficient for CSV packet format plus null terminator.

**Solution:** Increased `MESSAGE_LEN` to 256 bytes to accommodate maximum expected packet size with safety margin.

**Lesson:** Embedded systems require explicit memory allocation. Dynamic allocation is typically avoided in real-time systems. Buffer sizes must be calculated based on worst-case scenarios.

---

### 3. Stack Memory and Variable Scope

**Problem:** EKF processing garbage data. Pointer address remained valid but referenced incorrect values.

**Root Cause:** `EKF_State_t` struct instantiated inside `main()` loop. Each iteration created a new instance on the stack, invalidating the previous memory location while external functions held stale pointers.

**Debugging Method:** Used GDB via JTAG interface in Eclipse to inspect memory addresses and stack frames during execution.

**Solution:** Moved `EKF_State_t ekf` to file scope with `static` storage class, ensuring persistence throughout program lifetime.

**Lesson:** Pointers in embedded C require careful lifetime management. Stack variables must not outlive their scope when referenced by other functions.

---

### 4. Time Synchronization and dt Calculation

**Problem:** Erratic EKF prediction behavior with unstable orientation estimates.

**Root Cause:** Hardcoded time step (`dt = 0.01s`) assumed 100Hz data rate. Actual UART/simulator communication operated at approximately 30Hz (0.033s), causing temporal mismatch in prediction model.

**Diagnostic Approach:**

- Enabled UART1 for debug telemetry output
- Developed Python visualization tool using Matplotlib for 3D quaternion representation
- Measured actual inter-packet timing

**Solution:** Implemented dynamic time step calculation: `dt = sim_time - last_time`, allowing EKF to adapt to variable data rates.

**Lesson:** Real-time systems must account for timing jitter. Simulation time should be preferred over wall-clock time when available.

---

### 5. Error State Dimensionality in EKF

**Problem:** Mathematical errors in EKF update step, resulting in matrix dimension mismatches.

**Root Cause:** Covariance matrix `P` dimensioned as 4×4 to match quaternion representation. However, EKF error state is parameterized by three rotation angles (roll, pitch, yaw) and three gyroscope biases, not the four-element quaternion.

**Solution:** Corrected `P` matrix to 6×6 dimensions (3 attitude errors + 3 bias errors).

**Lesson:** Quaternion-based EKF uses multiplicative error representation. Error state dimensionality differs from state vector dimensionality. Proper understanding of error parameterization is critical.

---

### 6. Quaternion Normalization Bug

**Problem:** Orientation drift followed by discontinuous jumps. Eventually quaternion diverged to `NaN`, triggering EKF reinitialization.

**Root Cause:** Local variables (`qw, qi, qj, qk`) used for readability held stale quaternion values. After updating `state->q` array, normalization step incorrectly operated on local copies rather than updated values. This caused quaternion magnitude to decay toward zero over time, eventually causing division by zero during normalization.

**Mathematical Context:**

```
Given q = [qw, qi, qj, qk] with ||q|| → 0
Normalization: q_normalized = q / ||q||
As ||q|| → 0: division by zero → NaN
```

**Solution:** Modified normalization to directly reference `state->q[0..3]` instead of local variable copies.

**Lesson:** Variable aliasing in embedded systems can introduce subtle temporal bugs. When state updates occur, all dependent calculations must use fresh values.

---

### 7. Real-Time Performance and Latency Optimization

**Problem:** System functional but exhibited approximately 500ms control lag with visible jitter in control response.

**Root Cause:** Pseudo-circular DMA implementation using `HAL_UART_Receive_DMA(&huart2, &rx_byte, 1)`. This transferred single bytes with interrupt per character, effectively creating interrupt-driven byte-by-byte reception rather than streaming DMA. Combined with blocking transmit operations and heavy `sscanf()` parsing, packet queue buildup caused EKF to integrate stale data.

**Diagnosis Process:**

- Measured actual data flow rates and interrupt frequency
- Identified blocking operations in critical control loop
- Analyzed DMA configuration vs. actual behavior
- Profiled CPU usage during packet processing

**Solution Implemented:**

**1. True Circular DMA with Ring Buffer:**

```c
#define RX_DMA_SIZE 256
uint8_t rx_dma_buf[RX_DMA_SIZE];
volatile uint16_t rx_last_pos = 0;

HAL_UART_Receive_DMA(&huart2, rx_dma_buf, RX_DMA_SIZE);
__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
```

**2. UART IDLE Line Interrupt Detection:**

```c
void USART2_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        uint16_t pos = RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
        process_rx_data(rx_last_pos, pos);
        rx_last_pos = pos;
    }
    HAL_UART_IRQHandler(&huart2);
}
```

**3. Newest-Packet-Wins Processing:**

```c
void process_rx_data(uint16_t old_pos, uint16_t new_pos) {
    // Parse circular buffer, overwrite old packets
    data_ready = 1; // Latest packet always wins
}
```

**4. Non-Blocking Transmit:**

```c
void uart_output(uint8_t port, char *msg) {
    if (uart_tx_busy) return; // Drop instead of blocking
    uart_tx_busy = 1;
    HAL_UART_Transmit_DMA(...);
}
```

**5. Time Step Clamping:**

```c
if (dt < 0.001f || dt > 0.05f) {
    dt = 0.025f; // Prevent EKF explosion
}
```

**Result:** Latency reduced from ~500ms (observed) to estimated <10ms. Control response became smooth and immediate. System now operates in true real-time with no packet queue buildup.

**Lesson:** DMA configuration must match intended behavior. Single-byte DMA transfers negate performance benefits. Circular buffers with IDLE detection provide zero-copy, zero-latency streaming for variable-length packets. Control loops must never block on I/O operations.

---

### 8. Parsing Performance Optimization

**Problem:** `sscanf()` function causing CPU overhead in control loop despite DMA optimization. String parsing remained a bottleneck in packet processing pipeline.

**Root Cause:** `sscanf()` is a heavyweight function designed for general-purpose formatted input. It performs multiple passes over the string, allocates stack space for format parsing, and includes locale handling and error checking that are unnecessary for fixed-format CSV data.

**Performance Analysis (Estimated):**

- `sscanf()` with 7 floats: ~150-200 microseconds per packet (typical for STM32F411 at 100MHz)
- Creates temporary buffers and validates format string each call
- Includes overhead for handling arbitrary format specifiers

**Solution Implemented:**

Replaced `sscanf()` with `strtof()` stepwise parsing:

```c
char *ptr = (char *)rx_buffer;
char *end;

sim_time = strtof(ptr, &end);
if (*end != ',') continue; ptr = end + 1;

ax = strtof(ptr, &end);
if (*end != ',') continue; ptr = end + 1;

ay = strtof(ptr, &end);
if (*end != ',') continue; ptr = end + 1;

az = strtof(ptr, &end);
if (*end != ',') continue; ptr = end + 1;

gx = strtof(ptr, &end);
if (*end != ',') continue; ptr = end + 1;

gy = strtof(ptr, &end);
if (*end != ',') continue; ptr = end + 1;

gz = strtof(ptr, &end);
```

**Additional Improvements:**

**1. Removed redundant validation:**

```c
// Old: if (items == 7) { ... }
// New: Fail-fast comma checks in parsing loop
```

**2. Enhanced time step validation:**

```c
// Old: if (dt < 0.001f || dt > 0.05f)
// New: if (dt <= 0.0f || dt > 0.05f)  // Catches zero and negative
```

**3. Optimized NaN detection:**

```c
// Old: isnan(ekf.q[0]) || isnan(ekf.q[1]) || isnan(ekf.q[2]) || isnan(ekf.q[3])
// New: isnan(ekf.q[0])  // If any component is NaN, entire quaternion is invalid
```

**Result (Estimated Performance):**

- Parsing time reduced from ~150µs to ~15-20µs (estimated 10× improvement)
- Fail-fast error handling with comma validation
- Cleaner code structure with explicit pointer arithmetic
- Better error detection (catches negative dt)

**Lesson:** Built-in library functions trade generality for performance. For fixed-format data in real-time systems, custom parsers or lightweight alternatives like `strtof()` provide significant performance gains. Validation should be fast and explicit rather than implicit post-parsing checks.

---

## System Performance Metrics

**Current Status (Post-Optimization - Estimated Values):**

- Control loop latency: <10ms (estimated)
- Packet processing: Non-blocking, newest-wins
- Parsing performance: ~15-20µs per packet (estimated, strtof)
- CPU utilization: ~20% at 100MHz (estimated)
- DMA overhead: <1% (interrupt-driven completion only)

**Before vs. After:**

- Latency: 500ms (observed) → <10ms (estimated, 50× improvement)
- Parsing: 150µs → 20µs (estimated 7-10× improvement)
- Jitter: High → Minimal (observed)
- Packet drops: Queue buildup → Intentional (old data)
- Blocking operations: Multiple → Zero

---

## Future Optimization Opportunities

### Communication Protocol

- Binary packet format with CRC
- Command acknowledgment and retry logic
- Telemetry downsampling for bandwidth optimization

### Control Algorithm

- Add integral term for steady-state error elimination
- Implement adaptive gain scheduling based on flight regime
- Consider switching to Mahony filter for reduced computational cost

### Communication Protocol

- Binary packet format with CRC
- Command acknowledgment and retry logic
- Telemetry downsampling for bandwidth optimization

---

## Current Focus: Control Tuning and Flight Testing

**Status:** Core infrastructure complete. Focus shifted to control law refinement and closed-loop testing in FlightGear simulator.

**Next Steps:**

- Tune proportional gains for different flight conditions
- Implement yaw control via rudder
- Add integral term if steady-state errors observed
- Test under turbulence and sensor noise conditions

---

## System Architecture

### Current Configuration

**Communication:**

- USART2: Bidirectional communication with FlightGear (RX: sensor data, TX: control commands)
- USART1: Debug telemetry output
- Both configured with DMA for non-blocking operation

**Control Algorithm:**

- Extended Kalman Filter for attitude estimation
- Quaternion representation to avoid gimbal lock
- Proportional control law (relying on aircraft dynamics for damping)

**State Vector:**

- Quaternion: `q = [qw, qi, qj, qk]`
- Gyroscope bias: `[bx, by, bz]`

**Measurements:**

- Angular rates from gyroscope: `[gx, gy, gz]` (rad/s)
- Acceleration from accelerometer: `[ax, ay, az]` (normalized)

---

## Build and Test Environment

**IDE:** STM32CubeIDE  
**Debugger:** ST-Link V2 with GDB  
**Simulator:** FlightGear 2020.3.x  
**Visualization:** Python 3 with Matplotlib  
**Version Control:** Git

---

## References

- Madgwick, S. O. H. (2010). "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
- Quaternion-based Extended Kalman Filter theory
- STM32F4xx HAL Driver documentation
- ARM Cortex-M4 Technical Reference Manual

---

**Last Updated:** January 22, 2026  
**Status:** Active Development - Performance optimization phase
