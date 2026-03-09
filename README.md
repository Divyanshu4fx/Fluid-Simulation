# M5StickC Plus2 — Fluid Simulation

A real-time particle-based fluid simulation running on the M5StickC Plus2. Tilt the device and watch 300 particles slosh around like liquid on the screen.

---
![am54j4](https://github.com/user-attachments/assets/ad3ce087-bd02-4a4c-9784-fea3cbfe8009)

## How It Works

The workload is split across both ESP32 cores:

- **Core 0 — Physics:** Reads accelerometer (MPU6886) data, applies gravity to each particle, resolves collisions using a spatial grid, and enforces screen boundaries.
- **Core 1 — Render:** Takes a snapshot of particle positions, builds a density field using a soft stamp kernel, applies a threshold to draw blob-like liquid shapes, and pushes the frame to the ST7789 display via SPI.

The two cores stay in sync using a pair of binary semaphores — Core 1 signals when it's done reading, Core 0 signals when the next frame is ready.

---

## Parameters

| Constant | Value | What it does |
|---|---|---|
| `NUM_PARTICLES` | 300 | Number of simulated particles |
| `PARTICLE_RADIUS` | 6.5 | Collision radius per particle |
| `DAMPING` | 0.8 | Energy kept after bouncing off a wall |
| `REPULSION` | 0.1 | How hard particles push each other apart |
| `THERMAL_JITTER` | 0.02 | Small random nudge to prevent particles from locking up |
| `CELL_SIZE` | 14 | Spatial grid cell size in pixels |

---

## Controls

| Input | Action |
|---|---|
| Button A (GPIO 37) | Reset all particles to random positions |
| Tilt device | Changes gravity direction |

---

## Dependencies

- ESP-IDF (tested with v5.x)
- `esp_lcd` panel driver (ST7789)
- FreeRTOS (included with ESP-IDF)
- A `pins.h` file with your board's pin definitions

---

## Building & Flashing

```bash
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## Notes

- The frame buffer is allocated in DMA-capable memory for fast SPI transfers.
- Particle colors are hardcoded as two shades of green (RGB565). Change `swapped_core` and `swapped_edge` in the render task to use different colors.
