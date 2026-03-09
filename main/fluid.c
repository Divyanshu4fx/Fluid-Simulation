#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"

// Include your custom hardware mapping
#include "pins.h"

static const char *TAG1 = "M5_APP";
static const char *TAG_PHYS = "CORE0_PHYSICS";
static const char *TAG_REND = "CORE1_RENDER";

// --- Display Definitions ---
#define LCD_H_RES 135
#define LCD_V_RES 240
#define SCREEN_W 240
#define SCREEN_H 135
#define LCD_HOST SPI2_HOST

// --- M5StickC Plus2 Buttons ---
#define BUTTON_A_PIN 37 // Center Button

// --- I2C & MPU6886 Definitions ---
#define I2C_MASTER_NUM    I2C_NUM_0
#define MPU6886_ADDR      0x68
#define MPU6886_PWR_MGMT  0x6B
#define MPU6886_ACCEL_X   0x3B

// --- Fluid Simulation Definitions ---
#define NUM_PARTICLES 300
#define PARTICLE_RADIUS 6.5f
#define DAMPING 0.8f         // Bounce energy lost off walls
#define REPULSION 0.1f      // Stiff repulsion to prevent volume loss
#define THERMAL_JITTER 0.02f // Tiny random force to prevent perfect stacking lockups

// --- Spatial Grid Definitions ---
#define CELL_SIZE 14
#define GRID_COLS (SCREEN_W / CELL_SIZE + 1) // 25 columns
#define GRID_ROWS (SCREEN_H / CELL_SIZE + 1) // 14 rows
#define NUM_CELLS (GRID_COLS * GRID_ROWS)    // 350 cells total

// An 15x15 soft density stamp for massive liquid blobs
const uint8_t liquid_stamp[15][15] = {
    {  0,   0,   0,   0,   5,  10,  15,  20,  15,  10,   5,   0,   0,   0,   0},
    {  0,   0,   5,  15,  25,  40,  50,  55,  50,  40,  25,  15,   5,   0,   0},
    {  0,   5,  20,  40,  60,  80,  95, 105,  95,  80,  60,  40,  20,   5,   0},
    {  0,  15,  40,  70,  95, 120, 140, 150, 140, 120,  95,  70,  40,  15,   0},
    {  5,  25,  60,  95, 125, 150, 170, 180, 170, 150, 125,  95,  60,  25,   5},
    { 10,  40,  80, 120, 150, 180, 200, 210, 200, 180, 150, 120,  80,  40,  10},
    { 15,  50,  95, 140, 170, 200, 220, 230, 220, 200, 170, 140,  95,  50,  15},
    { 20,  55, 105, 150, 180, 210, 230, 240, 230, 210, 180, 150, 105,  55,  20},
    { 15,  50,  95, 140, 170, 200, 220, 230, 220, 200, 170, 140,  95,  50,  15},
    { 10,  40,  80, 120, 150, 180, 200, 210, 200, 180, 150, 120,  80,  40,  10},
    {  5,  25,  60,  95, 125, 150, 170, 180, 170, 150, 125,  95,  60,  25,   5},
    {  0,  15,  40,  70,  95, 120, 140, 150, 140, 120,  95,  70,  40,  15,   0},
    {  0,   5,  20,  40,  60,  80,  95, 105,  95,  80,  60,  40,  20,   5,   0},
    {  0,   0,   5,  15,  25,  40,  50,  55,  50,  40,  25,  15,   5,   0,   0},
    {  0,   0,   0,   0,   5,  10,  15,  20,  15,  10,   5,   0,   0,   0,   0}
};

// Particle Structure
typedef struct {
    float x, y;
    float vx, vy;
} Particle_t;

// --- Multi-Core Sync Primitives & Buffers ---
SemaphoreHandle_t frame_ready_sem;
SemaphoreHandle_t render_done_sem;

Particle_t live_particles[NUM_PARTICLES];   // Core 0 updates this
Particle_t render_snapshot[NUM_PARTICLES];  // Core 1 reads this

// Resets particles to random positions
void reset_particles(Particle_t *particles) {
    for (int i = 0; i < NUM_PARTICLES; i++) {
        particles[i].x = (rand() % (SCREEN_W - 20)) + 10;
        particles[i].y = (rand() % (SCREEN_H - 20)) + 10;
        particles[i].vx = (rand() % 10 - 5) * 0.1f;
        particles[i].vy = (rand() % 10 - 5) * 0.1f;
    }
}

// --- CORE 0: PHYSICS TASK ---
void core0_physics_task(void *pvParameters)
{
    ESP_LOGI(TAG_PHYS, "Starting Physics Engine on Core 0");

    // Init MPU6886 I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = M5_I2C_SDA_PIN,
        .scl_io_num = M5_I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    uint8_t wake_data[2] = {MPU6886_PWR_MGMT, 0x00};
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6886_ADDR, wake_data, 2, 1000 / portTICK_PERIOD_MS);

    // Init Button
    gpio_set_direction(BUTTON_A_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_A_PIN);

    // Allocate Grid Memory
    int16_t *grid_head = (int16_t *)malloc(NUM_CELLS * sizeof(int16_t));
    int16_t *particle_next = (int16_t *)malloc(NUM_PARTICLES * sizeof(int16_t));

    reset_particles(live_particles);

    uint8_t reg = MPU6886_ACCEL_X;
    uint8_t raw_accel[6];

    while(1) {
        // --- 1. User Input ---
        if (gpio_get_level(BUTTON_A_PIN) == 0) {
            reset_particles(live_particles);
            vTaskDelay(pdMS_TO_TICKS(200)); 
        }

        // --- 2. Read IMU Data ---
        i2c_master_write_read_device(I2C_MASTER_NUM, MPU6886_ADDR, &reg, 1, raw_accel, 6, 100 / portTICK_PERIOD_MS);
        
        int16_t ax_raw = (raw_accel[0] << 8) | raw_accel[1];
        int16_t ay_raw = (raw_accel[2] << 8) | raw_accel[3];

        float gx =  (float)ay_raw * 0.0001f; 
        float gy =  (float)ax_raw * 0.0001f; 

        // --- 3. Physics & Collisions ---
        memset(grid_head, 0xFF, NUM_CELLS * sizeof(int16_t));

        // Step A: Apply forces
        for (int i = 0; i < NUM_PARTICLES; i++) {
            float jitter_x = ((rand() % 100) / 100.0f - 0.5f) * THERMAL_JITTER;
            float jitter_y = ((rand() % 100) / 100.0f - 0.5f) * THERMAL_JITTER;

            live_particles[i].vx += gx + jitter_x;
            live_particles[i].vy += gy + jitter_y;
            
            live_particles[i].vx *= 0.95f; 
            live_particles[i].vy *= 0.95f;

            live_particles[i].x += live_particles[i].vx;
            live_particles[i].y += live_particles[i].vy;

            int cx = (int)(live_particles[i].x) / CELL_SIZE;
            int cy = (int)(live_particles[i].y) / CELL_SIZE;

            if (cx < 0) cx = 0; else if (cx >= GRID_COLS) cx = GRID_COLS - 1;
            if (cy < 0) cy = 0; else if (cy >= GRID_ROWS) cy = GRID_ROWS - 1;

            int cell_idx = cy * GRID_COLS + cx;
            particle_next[i] = grid_head[cell_idx];
            grid_head[cell_idx] = i;
        }

        // Step B: Resolve Collisions
        float min_dist = PARTICLE_RADIUS * 2.0f;
        float min_dist_sq = min_dist * min_dist;

        for (int i = 0; i < NUM_PARTICLES; i++) {
            int cx = (int)(live_particles[i].x) / CELL_SIZE;
            int cy = (int)(live_particles[i].y) / CELL_SIZE;

            for (int ny = cy - 1; ny <= cy + 1; ny++) {
                if (ny < 0 || ny >= GRID_ROWS) continue; 
                for (int nx = cx - 1; nx <= cx + 1; nx++) {
                    if (nx < 0 || nx >= GRID_COLS) continue;

                    int cell_idx = ny * GRID_COLS + nx;
                    int j = grid_head[cell_idx];

                    while (j != -1) {
                        if (i < j) { 
                            float dx = live_particles[i].x - live_particles[j].x;
                            float dy = live_particles[i].y - live_particles[j].y;
                            float dist_sq = dx * dx + dy * dy;
                            
                            if (dist_sq < min_dist_sq && dist_sq > 0.001f) { 
                                float dist = sqrtf(dist_sq); 
                                float overlap = min_dist - dist;
                                
                                float inv_dist = 1.0f / dist; 
                                float nx_dir = dx * inv_dist;
                                float ny_dir = dy * inv_dist;

                                float force = overlap * REPULSION;
                                float fx = nx_dir * force;
                                float fy = ny_dir * force;

                                live_particles[i].x += fx;
                                live_particles[i].y += fy;
                                live_particles[j].x -= fx;
                                live_particles[j].y -= fy;
                            }
                        }
                        j = particle_next[j]; 
                    }
                }
            }
        }

        // Step C: Screen Boundaries
        for (int i = 0; i < NUM_PARTICLES; i++) {
            if (live_particles[i].x < PARTICLE_RADIUS) {
                live_particles[i].x = PARTICLE_RADIUS;
                live_particles[i].vx *= -DAMPING;
            } else if (live_particles[i].x > SCREEN_W - PARTICLE_RADIUS) {
                live_particles[i].x = SCREEN_W - PARTICLE_RADIUS;
                live_particles[i].vx *= -DAMPING;
            }

            if (live_particles[i].y < PARTICLE_RADIUS) {
                live_particles[i].y = PARTICLE_RADIUS;
                live_particles[i].vy *= -DAMPING;
            } else if (live_particles[i].y > SCREEN_H - PARTICLE_RADIUS) {
                live_particles[i].y = SCREEN_H - PARTICLE_RADIUS;
                live_particles[i].vy *= -DAMPING;
            }
        }

        // --- 4. Synchronization ---
        // Wait for Core 1 to finish drawing the last frame
        xSemaphoreTake(render_done_sem, portMAX_DELAY);
        
        // Copy the computed frame into the snapshot buffer for Core 1 to use
        memcpy(render_snapshot, live_particles, sizeof(live_particles));
        
        // Tell Core 1 the new snapshot is ready
        xSemaphoreGive(frame_ready_sem);
    }
}


// --- CORE 1: RENDER TASK ---
void core1_render_task(void *pvParameters)
{
    ESP_LOGI(TAG_REND, "Starting Render Engine on Core 1");

    // Init SPI and Display
    spi_bus_config_t buscfg = {
        .sclk_io_num = M5_TFT_SCLK_PIN,
        .mosi_io_num = M5_TFT_MOSI_PIN,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = M5_TFT_DC_PIN,
        .cs_gpio_num = M5_TFT_CS_PIN,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = M5_TFT_RST_PIN,
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 40, 53));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // Turn on Backlight
    gpio_reset_pin(M5_TFT_BACKLIGHT_PIN);
    gpio_set_direction(M5_TFT_BACKLIGHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(M5_TFT_BACKLIGHT_PIN, 1);

    // Allocate Rendering Buffers
    uint16_t *frame_buffer = (uint16_t *)heap_caps_malloc(SCREEN_W * SCREEN_H * sizeof(uint16_t), MALLOC_CAP_DMA);
    uint8_t *density_buffer = (uint8_t *)malloc(SCREEN_W * SCREEN_H * sizeof(uint8_t));

    uint16_t swapped_core = 0x1F01; 
    uint16_t swapped_edge = 0xFF07; 

    // Kickoff the pipeline: Tell Physics that we are ready for the first frame
    xSemaphoreGive(render_done_sem);

    while(1) {
        // --- 1. Synchronization ---
        // Wait for Core 0 to provide a new snapshot
        xSemaphoreTake(frame_ready_sem, portMAX_DELAY);

        // --- 2. Metaball Rasterization ---
        memset(density_buffer, 0, SCREEN_W * SCREEN_H * sizeof(uint8_t));

        for (int i = 0; i < NUM_PARTICLES; i++) {
            // WE ONLY READ FROM render_snapshot HERE!
            int px = (int)render_snapshot[i].x;
            int py = (int)render_snapshot[i].y;

            int start_y = (py - 7 < 0) ? 0 : py - 7;
            int end_y = (py + 7 >= SCREEN_H) ? SCREEN_H - 1 : py + 7;
            int start_x = (px - 7 < 0) ? 0 : px - 7;
            int end_x = (px + 7 >= SCREEN_W) ? SCREEN_W - 1 : px + 7;

            int stamp_x_base = 7 - px;

            for (int y = start_y; y <= end_y; y++) {
                int stamp_y = (y - py) + 7;
                
                uint8_t *row_ptr = &density_buffer[y * SCREEN_W]; 
                const uint8_t *stamp_row = liquid_stamp[stamp_y]; 
                
                for (int x = start_x; x <= end_x; x++) {
                    uint16_t new_density = row_ptr[x] + stamp_row[x + stamp_x_base];
                    row_ptr[x] = (new_density > 255) ? 255 : (uint8_t)new_density;
                }
            }
        }

        // --- 3. Thresholding & Frame Construction ---
        uint32_t total_pixels = SCREEN_W * SCREEN_H;
        uint16_t *fb_ptr = frame_buffer;
        uint8_t *db_ptr = density_buffer;
        
        for (uint32_t i = 0; i < total_pixels; i++) {
            uint8_t d = *db_ptr++;
            if (d > 110) {
                *fb_ptr++ = swapped_core;
            } else if (d > 70) {
                *fb_ptr++ = swapped_edge;
            } else {
                *fb_ptr++ = 0x0000; 
            }
        }

        // --- 4. Blast to Display ---
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, SCREEN_W, SCREEN_H, frame_buffer);

        // Tell Core 0 we are done processing the snapshot and it can overwrite it
        xSemaphoreGive(render_done_sem);
    }
}

void app_main(void)
{
    // Initialize Power Hold
    gpio_reset_pin(M5_POWER_HOLD_PIN);
    gpio_set_direction(M5_POWER_HOLD_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(M5_POWER_HOLD_PIN, 1);
    ESP_LOGI(TAG1, "Power hold engaged.");

    // Create FreeRTOS Binary Semaphores for task synchronization
    frame_ready_sem = xSemaphoreCreateBinary();
    render_done_sem = xSemaphoreCreateBinary();

    // Pin Physics Engine to Core 0 (PRO_CPU)
    xTaskCreatePinnedToCore(core0_physics_task, "PhysicsTask", 16384, NULL, 5, NULL, 0);
    
    // Pin Render Engine to Core 1 (APP_CPU)
    xTaskCreatePinnedToCore(core1_render_task, "RenderTask", 16384, NULL, 5, NULL, 1);

    while (1) {
        // Main task can go to sleep, the dual cores are handling everything
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}