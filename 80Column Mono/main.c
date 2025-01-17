/*
 * Terminal software for Pi Pico
 * USB keyboard input, VGA video output, communication with RC2014 via UART on GPIO 20 & 21
 * Shiela Dixon, https://peacockmedia.software  
 * Refactored on feb/22 by Daniel Quadros, https://dqsoft.blogspot.com
 *
 *
 * much of what's in this main file is taken from the VGA textmode example
 * and the TinyUSB hid_app
 *
 * picoterm.c handles the behaviour of the terminal and storing the text
 * serial.c   handles the UART
 * keybd.c handles the USB keyboard
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */


#include "main.h"
#include "picoterm.h"
#include "serial.h"
#include "keybd.h"

#define LED             25

// LED control
static LED_STATUS led_status;

// Video mode
#define vga_mode vga_mode_640x480_60
#define COUNT ((vga_mode.width/8)-1)

// Video rendering
#define RENDER_ON_CORE1
typedef bool (*render_scanline_func)(struct scanvideo_scanline_buffer *dest, int core);
bool render_scanline_bg(struct scanvideo_scanline_buffer *dest, int core);
render_scanline_func render_scanline = render_scanline_bg;

// Font for the characters
const lv_font_t *font = &ubuntu_mono8;
uint32_t *font_raw_pixels;

// This is 4 for the font we're using
#define FRAGMENT_WORDS 4

// FONT_WIDTH_WORDS = number of 32 bit words for character width
// FONT_HEIGHT = character height
// FONT_SIZE_WORDS = number of 32 bit words to store a character
#define FONT_WIDTH_WORDS FRAGMENT_WORDS
#define FONT_HEIGHT (font->line_height)
#define FONT_SIZE_WORDS (FONT_HEIGHT * FONT_WIDTH_WORDS)


// This block contains a single line (8 pixels) of the space character
const uint32_t block[] = {
                    PICO_SCANVIDEO_PIXEL_FROM_RGB5(0,0,0) << 16 |
                    PICO_SCANVIDEO_PIXEL_FROM_RGB5(0,0,0),
                    PICO_SCANVIDEO_PIXEL_FROM_RGB5(0,0,0) << 16 |
                    PICO_SCANVIDEO_PIXEL_FROM_RGB5(0,0,0), 
                    PICO_SCANVIDEO_PIXEL_FROM_RGB5(0,0,0) << 16 |
                    PICO_SCANVIDEO_PIXEL_FROM_RGB5(0,0,0), 
                    PICO_SCANVIDEO_PIXEL_FROM_RGB5(0,0,0) << 16 |
                    PICO_SCANVIDEO_PIXEL_FROM_RGB5(0,0,0) 
};

// to make sure only one core updates the state when the frame number changes
// todo note we should actually make sure here that the other core isn't still rendering (i.e. all must arrive before either can proceed - a la barrier)
//auto_init_mutex(frame_logic_mutex);
struct mutex frame_logic_mutex;

// rotines to execute the rendering
void go_core1(void (*execute)());
void init_render_state(int core);

// "Tasks" (rotines that will be continuous called in the main loop)
void led_blinking_task(void);


// This rotine will continuously render scanlines
void render_loop() {
    static uint32_t last_frame_num = 0;
    int core_num = get_core_num();
    assert(core_num >= 0 && core_num < 2);
    //printf("Rendering on core %d\n", core_num);

    while (true) {
        struct scanvideo_scanline_buffer *scanline_buffer = scanvideo_begin_scanline_generation(true);
        
        mutex_enter_blocking(&frame_logic_mutex);
        uint32_t frame_num = scanvideo_frame_number(scanline_buffer->scanline_id);
        // note that with multiple cores we may have got here not for the first scanline, however one of the cores will do this logic first before either does the actual generation
        if (frame_num != last_frame_num) {
            // this could should be during vblank as we try to create the next line
            // todo should we ignore if we aren't attempting the next line
            last_frame_num = frame_num;
        }
        mutex_exit(&frame_logic_mutex);
        render_scanline(scanline_buffer, core_num);

        // release the scanline into the wild
        scanvideo_end_scanline_generation(scanline_buffer);
        // do this outside mutex and scanline generation
    } // end while(true) loop
}

struct semaphore video_setup_complete;

// initialize video generation
void setup_video() {
    scanvideo_setup(&vga_mode);
    scanvideo_timing_enable(true);
    sem_release(&video_setup_complete);
}

// funtion to run the render loop on core 1
void core1_func() {
    render_loop();
}

#define TEST_WAIT_FOR_SCANLINE

#ifdef TEST_WAIT_FOR_SCANLINE
volatile uint32_t scanline_color = 0;
#endif

// Expand font
// We will generate two copies of the font, one normal and one reverse
void build_font() {

    // Font uses 4 bit per pixel (16 levels of grey)
    // colors has the equivalent 16 bit color used by the hardware
    uint16_t colors[16];
    for (int i = 0; i < count_of(colors); i++) {
        colors[i] = PICO_SCANVIDEO_PIXEL_FROM_RGB5(1, 1, 1) * ((i * 3) / 2);
        //if (i) i != 0x8000;
    }

    // allocate memory for the font
    // (2 copies, each whith range_length chars, each using FONT_SIZE_WORDS*4 bytes
    font_raw_pixels = (uint32_t *) calloc(4, font->dsc->cmaps->range_length * FONT_SIZE_WORDS * 2);

    // Normal characters at the begining, reverse characters next
    uint32_t *p = font_raw_pixels;
    uint32_t *pr = font_raw_pixels+(font->dsc->cmaps->range_length * FONT_SIZE_WORDS);

    assert(font->line_height == FONT_HEIGHT);

    // for each character in font, generate the two raw pixels images
    for (int c = 0; c < (font->dsc->cmaps->range_length); c++) {
        // glyph for character
        const lv_font_fmt_txt_glyph_dsc_t *g = &font->dsc->glyph_dsc[c + 1];
        // glyph bytes
        const uint8_t *b = font->dsc->glyph_bitmap + g->bitmap_index;

        int bi = 0; // index in b
        // lets process each line
        // the glyph has only box_w by box_h pixels at (ofs_x, base_line + ofs_y)
        for (int y = 0; y < FONT_HEIGHT; y++) {
            int ey = y - FONT_HEIGHT + font->base_line + g->ofs_y + g->box_h;
            for (int x = 0; x < FONT_WIDTH_WORDS * 2; x++) {
              uint16_t pixel;
              int ex = x - g->ofs_x;

              if (ex >= 0 && ex < g->box_w && ey >= 0 && ey < g->box_h) {
                  // glyph has a pixel for this position
                  pixel = bi & 1 ? colors[b[bi >> 1] & 0xf] : colors[b[bi >> 1] >> 4];
                  bi++;
              } else {
                  // this position is outside the glyph box
                  pixel = 0;
              }

              // store pixel
              if (!(x & 1)) {
                  *p = pixel;
                  *pr = pixel ^ 0x7FFF;
              } else {
                  *p++ |= pixel << 16;
                  *pr++ |= (pixel ^ 0x7FFF) << 16;
              }

            }
            if (ey >= 0 && ey < g->box_h) {
                for (int x = FONT_WIDTH_WORDS * 2 - g->ofs_x; x < g->box_w; x++) {
                    bi++;
                }
            }
        }
    }
}


// setus up video and start rendering
int video_main(void) {

    mutex_init(&frame_logic_mutex);

    build_font();
    sem_init(&video_setup_complete, 0, 1);

    setup_video();

#ifdef RENDER_ON_CORE1
    go_core1(core1_func);   // render_loop() on core 1
#endif
#ifdef RENDER_ON_CORE0
    render_loop();          
#endif

    return 0;
}

// Beginning of a line
static __not_in_flash("x") uint16_t beginning_of_line[] = {
        // todo we need to be able to shift scanline to absorb these extra pixels
#if FRAGMENT_WORDS == 5
        COMPOSABLE_RAW_1P, 0,
#endif
#if FRAGMENT_WORDS >= 4
        COMPOSABLE_RAW_1P, 0,
#endif
        COMPOSABLE_RAW_1P, 0,
        // main run, 2 more black pixels
        COMPOSABLE_RAW_RUN, 0,
        0/*COUNT * 2 * FRAGMENT_WORDS -3 + 2*/, 0
};
static __not_in_flash("y") uint16_t end_of_line[] = {
#if FRAGMENT_WORDS == 5 || FRAGMENT_WORDS == 3
        COMPOSABLE_RAW_1P, 0,
#endif
#if FRAGMENT_WORDS == 3
        COMPOSABLE_RAW_1P, 0,
#endif
#if FRAGMENT_WORDS >= 4
        COMPOSABLE_RAW_2P, 0,
        0, COMPOSABLE_RAW_1P_SKIP_ALIGN,
        0, 0,
#endif
        COMPOSABLE_EOL_SKIP_ALIGN, 0xffff // eye catcher
};


// This rotine renders a scan line
// this is done by filling the 'dest' structre see scanvidio doc for details 
bool render_scanline_bg(struct scanvideo_scanline_buffer *dest, int core) {
    uint32_t *buf = dest->data;
    int y = scanvideo_scanline_number(dest->scanline_id);

#undef COUNT
    // todo for SOME REASON, 80 is the max we can do without starting to really get bus delays (even with priority)... not sure how this could be
    // todo actually it seems it can work, it just mostly starts incorrectly synced!?
#define COUNT MIN(vga_mode.width/(FRAGMENT_WORDS*2)-1, 80)

    dest->fragment_words = FRAGMENT_WORDS;

    beginning_of_line[FRAGMENT_WORDS * 2 - 2] = COUNT * 2 * FRAGMENT_WORDS - 3 + 2;
    assert(FRAGMENT_WORDS * 2 == count_of(beginning_of_line));
    assert(FRAGMENT_WORDS * 2 == count_of(end_of_line));

    uint32_t *output32 = buf;

    *output32++ = host_safe_hw_ptr(beginning_of_line);
    uint32_t *dbase = font_raw_pixels + FONT_WIDTH_WORDS * (y % FONT_HEIGHT);
    
    char ch = 0;

    int tr = (y/FONT_HEIGHT);
    unsigned char *rowslots = slotsForRow(tr); // I want a better word for slots. (Character positions).

    // loop over the characters and copy pointer to the raw pixels for the current scanline
    for (int i = 0; i < COUNT; i++) {

      ch = *rowslots;
      rowslots++;

      if(ch==0){
          *output32++ = host_safe_hw_ptr(&block); 
          // shortcut
          // there's likely to be a lot of spaces on the screen.
          // if this character is a space, just use this predefined zero block rather than the calculation below
      }
      else{
        *output32++ = host_safe_hw_ptr(dbase + ch * FONT_HEIGHT * FONT_WIDTH_WORDS); 
      }

    }

    *output32++ = host_safe_hw_ptr(end_of_line);
    *output32++ = 0; // end of chain

    assert(0 == (3u & (intptr_t) output32));
    assert((uint32_t *) output32 <= (buf + dest->data_max));

    dest->data_used = (uint16_t) (output32 -
                                  buf); // todo we don't want to include the off the end data in the "size" for the dma
    dest->status = SCANLINE_OK;

    return true;
}

// launch execution on core 1
void go_core1(void (*execute)()) {
    multicore_launch_core1(execute);
}

// Handle received characters
void handle_rx() {
  if (has_rx()) {
    clear_cursor();
    do {
        handle_new_character(get_rx());
    } while (has_rx());
    show_cursor();
  }
}


//--------------------------------------------------------------------+
// Start of execution
//--------------------------------------------------------------------+
int main(void) {

  // initialize the LED
  setLED (LED_BLINK);

  gpio_init(LED);
  gpio_set_dir(LED, GPIO_OUT);
  gpio_put(LED,false);    

  // init stdio (for debug messages)
  stdio_init_all();

   // initialize tinyusb stack
  tusb_init();

  // init UART
  serial_init();

  // init video
  prepare_text_buffer();
  video_main();

  // main loop for core 0
  while(true){

    tuh_task();
    led_blinking_task();
    serial_tx_task();

    #if CFG_TUH_CDC
        cdc_task();
    #endif

    #if CFG_TUH_HID
        hid_app_task();
    #endif

    handle_rx();

  }

  return 0;
}


// Set new LED status
void setLED(LED_STATUS status) {
  led_status = status;
}

//--------------------------------------------------------------------+
// LED Blinking Task
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  const uint32_t interval_ms = 1000;
  static uint32_t start_ms = 0;
  static bool led_state = false;


  switch(led_status){
    case LED_OFF:
      if (led_state) {
        board_led_write(false);
        led_state = false;
      }
     break;
    case LED_ON:
      if (!led_state) {
        board_led_write(true);
        led_state = true;
      }
     break;
    case LED_BLINK:
      // Blink every interval ms
      if ( (board_millis() - start_ms) > interval_ms) {
        start_ms += interval_ms;
        board_led_write(led_state);
        led_state = !led_state; // toggle
      }
    break;
  }


}

