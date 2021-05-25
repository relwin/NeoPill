"""
stripsim.py

Graphical NeoPixel LED strip emulator.
Connect target's Neopixel output to NeoPill's (STM32 Bluepill) inputs.
Connect PC via USB to NeoPill for Neopixel->USB serial bridge. Find the appropriate COM port to use.

This code reads a yaml file for configuration.
Each LED is a simple rectangle or circle, with an optional small gap in-between.

Using a COM port, we sync to the pixel bitstream. Once sync'd each frame of pixel data is gamma-corrected and displayed as fast as possible.
Display frame rate dictated by the target device and config item 'fps_limit'. However, many LCD monitors only go to 60Hz.

Simple runtime controls (window active):
    mouseclick : shows runtime stats
    Up/Dn arrow: adjust gamma
    's': re-sync to the bitstream

COM port not connecting? Go into Windows DM and "disable/enable" the port.
This may be required after Bluepill reset.

Need to emulate with different bitstream timing?
timing_T1H, timing_RES config items.

For python 3.8.1

R. Elwin 5/2021
"""
import threading
import queue
from typing import List, Any
import yaml  # 5.4.1
import pygame  # 2.0!
# from pygame.locals import *     # doing this floods debug with symbols so specify ones we actually use
from pygame.locals import QUIT, MOUSEBUTTONDOWN, KEYDOWN, K_UP, K_DOWN, K_s
import os
import time
import sys
import serial  # 3.5

os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (20, 20)  # default window position


# Init USB serial port, send optional timing string
def serial_init(com_port, timing_mod=None):
    try:
        ser = serial.Serial(com_port, baudrate=12000000, timeout=None)  # max USB speed, read timeout adjusted for debugging, timeout = None waits forever
        if timing_mod is not None:
            ser.write(bytearray(timing_mod.encode('utf-8')))  # send timing mod
            # ser.write(bytearray("T 38 3601".encode('utf-8')))      # test timing mod
            time.sleep(0.25)  # can't use flushInput() so delay instead
        return ser

    except serial.SerialException:
        print("SerialException: Can't Open", com_port)
        return None


# sync with NeoPill, may fail occasionally...
def serial_sync(ser):
    ser.write(bytearray("F".encode('utf-8')))  # send 'Flush' byte
    ser.reset_input_buffer()  # having issues with USB data not clearing (on Windows10) so loop a few times
    for d in range(0, 3):
        time.sleep(0.25)  # wait for USB drain
        ser.reset_input_buffer()  # in case frame data in the pipe
    ser.write(bytearray("S".encode('utf-8')))  # send sync start byte


class LEDStrip(object):
    """
    reads YAML config
    configures pygame window display
    generates LED display
    formats LED frames to window
    """
    # indicies for LED array surf_obj[0], rect[1], color[2]
    IDX_surf_obj = 0
    IDX_rect = 1
    IDX_color = 2

    def __init__(self, config_path):
        self.ledarray: List[Any] = []  # LED graphics elements
        self.led_display_format = None
        self.led_show = None
        self.screen = None
        self.ser = None
        self.serial_port = None
        self.timing_mod_T1H = None
        self.timing_mod_RES = None
        self.timing_mod = None
        self.strip_type = {}  # data from yaml file
        self.matrix_type = {}  # data from yaml file
        self.led_count = 0
        self.led_width = 1
        self.led_height = 1
        self.radius = 0
        self.center = 0
        self.gamma = 0.25  # LEDs are much brighter than LCD, smaller gamma=brighter. HW gamma not supported.
        self.frame_size = 0
        self.display_rate_limit = 1 / 120  # 120Hz display rate limit default, might be too fast still
        self.led_create = None
        self.generate_display = None
        self.read_config(config_path)
        self.format_timing_mod()
        self.generate_display()
        self.led_create(self.led_count)  # only using 1 strip/matrix at a time so far

    # load the yaml file into ledlist{}
    def read_config(self, config_path):
        with open(config_path) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            # print("Key: Value")
            for key, value in data.items():
                if key == "strip":
                    self.strip_type = value
                    self.generate_display = self.generate_display_strip
                    # print(self.strip_type)
                elif key == "matrix":
                    self.matrix_type = value
                    self.generate_display = self.generate_display_matrix
                elif key == "timing_T1H":
                    self.timing_mod_T1H = value  # modified timing string
                    print("Using Modified T1H timing", f"{value}")
                elif key == "timing_RES":
                    self.timing_mod_RES = value  # modified timing string
                    print("Using Modified RES timing", f"{value}")
                elif key == "serial_port":
                    self.serial_port = value
                elif key == "fps_limit":
                    self.display_rate_limit = 1 / (value * 1.1)  # use time per frame, plus 10% slop

    # format timing mod msg, specific to NeoPill timing. Calcs TIM2,4 delay, in clock counts (not ns).
    def format_timing_mod(self):
        cpu_clk = 13.88  # 13.88ns clk cycle, 1/72MHz
        T1H = 0
        if self.timing_mod_T1H is not None:
            T1H = int((int(self.timing_mod_T1H) - 3 * cpu_clk) / cpu_clk)  # TIM4 ARR has 3 clk delay
        RES = 0
        if self.timing_mod_RES is not None:
            RES = int(int(self.timing_mod_RES) / cpu_clk)  # TIM2 CCR1 count
        # format a timing mod if using either
        if self.timing_mod_T1H is not None or self.timing_mod_RES is not None:
            self.timing_mod = "T " + str(T1H) + " " + str(RES)

    # generate pygame display window filled with LED strip
    def generate_display_strip(self):
        self.led_create = self.led_create_strip
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (self.strip_type[0]['wposx'], self.strip_type[0]['wposy'])  # window position
        self.led_count = self.strip_type[0]['num']
        self.led_width = int(self.strip_type[0]['length'] / self.strip_type[0]['num']) - self.strip_type[0]['gap']
        strip_width = (self.led_width + self.strip_type[0]['gap']) * self.strip_type[0]['num']
        size = (strip_width, self.strip_type[0]['height'])  # create a strip
        self.screen = pygame.display.set_mode(size, flags=0, display=0)  # vsync=0 default
        pygame.display.set_caption(self.strip_type[0]['wname'])
        print(pygame.display.get_driver())
        print(pygame.display.Info())

    # populate LED array with LED surface objects, filled circles or plain squares
    def led_create_strip(self, num):
        for i in range(0, num):
            surf = pygame.Surface((self.led_width, self.strip_type[0]['height']))  # surface object
            rect = surf.get_rect()
            rect.left = i * (self.led_width + self.strip_type[0]['gap'])
            rect.top = 0
            color = pygame.Color(0, (i * 3) % 256, 10, 255)  # initial RGBA color, also fills in Alpha of RGBA
            self.ledarray.append((surf, rect, color))  # surf_obj[0], rect[1], color[2]
        self.led_format(self.strip_type[0]['style'], self.strip_type[0]['ledcolor'], num)

    # generate pygame display window filled with LED matrix
    def generate_display_matrix(self):
        self.led_create = self.led_create_matrix
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (self.matrix_type[0]['wposx'], self.matrix_type[0]['wposy'])  # window position
        self.led_count = self.matrix_type[0]['matrix_w'] * self.matrix_type[0]['matrix_h']
        self.led_width = int(self.matrix_type[0]['length'] / self.matrix_type[0]['matrix_w']) - self.matrix_type[0]['gap']  # gap is symmetric side & bottom
        strip_width = (self.led_width + self.matrix_type[0]['gap']) * self.matrix_type[0]['matrix_w'] - self.matrix_type[0]['gap']
        self.led_height = int(self.matrix_type[0]['height'] / self.matrix_type[0]['matrix_h']) - self.matrix_type[0]['gap']
        strip_height = (self.led_height + self.matrix_type[0]['gap']) * self.matrix_type[0]['matrix_h'] - self.matrix_type[0]['gap']
        size = (strip_width, strip_height)  # display (w,h), adjusted width, height
        self.screen = pygame.display.set_mode(size, flags=0, display=0)  # vsync=0 default
        pygame.display.set_caption(self.matrix_type[0]['wname'])
        print(pygame.display.get_driver())
        print(pygame.display.Info())

    # matrix option, populate LED array with LED surface objects, filled circles or plain squares
    def led_create_matrix(self, num):
        for j in range(0, self.matrix_type[0]['matrix_h']):
            for i in range(0, self.matrix_type[0]['matrix_w']):
                surf = pygame.Surface((self.led_width, self.led_height))  # surface object
                rect = surf.get_rect()
                rect.left = i * (self.led_width + self.matrix_type[0]['gap'])
                rect.top = j * (self.led_height + self.matrix_type[0]['gap'])
                color = pygame.Color(j, (i * 3) % 256, 10, 255)  # initial RGBA color, also fills in Alpha of RGBA
                self.ledarray.append((surf, rect, color))  # surf_obj[0], rect[1], color[2]
        self.led_format(self.matrix_type[0]['style'], self.matrix_type[0]['ledcolor'], num)

    # assign LED style, color order, frame size
    def led_format(self, style, ledcolor, num):
        if style == 'circle':
            self.led_show = self.led_show_circle
            # use ledarray[0] to get rect attributes to calc circle radius and center
            self.radius = min(self.ledarray[0][LEDStrip.IDX_rect].h, self.ledarray[0][LEDStrip.IDX_rect].w) / 2
            self.center = self.ledarray[0][LEDStrip.IDX_rect].w / 2, self.ledarray[0][LEDStrip.IDX_rect].h / 2
        else:
            self.led_show = self.led_show_rect
        if ledcolor == 'GRBW':
            self.led_display_format = self.led_frame_copy_GRBW
            self.frame_size = 4 * num
        elif ledcolor == 'GRB':
            self.led_display_format = self.led_frame_copy_GRB
            self.frame_size = 3 * num
        else:
            print("Error! No LED format, ledcolor: not assigned")

    # blit rectangular LEDs, then display
    def led_show_rect(self):
        for led in self.ledarray:
            color = led[LEDStrip.IDX_color].correct_gamma(self.gamma)  # adjust gamma for each LED
            led[LEDStrip.IDX_surf_obj].fill(color)  # color square
            self.screen.blit(led[LEDStrip.IDX_surf_obj], led[LEDStrip.IDX_rect])
        pygame.display.flip()

    # blit circular LEDs, then display
    def led_show_circle(self):
        for led in self.ledarray:
            color = led[LEDStrip.IDX_color].correct_gamma(self.gamma)  # adjust gamma for each LED
            pygame.draw.circle(led[LEDStrip.IDX_surf_obj], color, self.center, self.radius)
            self.screen.blit(led[LEDStrip.IDX_surf_obj], led[LEDStrip.IDX_rect])
        pygame.display.flip()

    # copy a frame of GRB LED data to RGBA LED array
    def led_frame_copy_GRB(self, led_data_frame):
        i = 0
        for k in range(0, self.led_count):
            self.ledarray[k][LEDStrip.IDX_color][1] = led_data_frame[i]  # G
            i += 1
            self.ledarray[k][LEDStrip.IDX_color][0] = led_data_frame[i]  # R
            i += 1
            self.ledarray[k][LEDStrip.IDX_color][2] = led_data_frame[i]  # B
            i += 1

    # copy a frame of GRBW LED data to RGBA LED array
    def led_frame_copy_GRBW(self, led_data_frame):
        i = 0
        for k in range(0, self.led_count):
            self.ledarray[k][LEDStrip.IDX_color][1] = led_data_frame[i]  # G
            i += 1
            self.ledarray[k][LEDStrip.IDX_color][0] = led_data_frame[i]  # R
            i += 1
            self.ledarray[k][LEDStrip.IDX_color][2] = led_data_frame[i]  # B
            i += 1
            w = led_data_frame[i]
            # what to do w/white LED? just saturate add to each color
            if w > 0:
                for j in range(0, 3):
                    c = self.ledarray[k][LEDStrip.IDX_color][j] + w
                    if c > 255:
                        c = 255
                    self.ledarray[k][LEDStrip.IDX_color][j] = c
            # ledarray[k][5][3] = 255     # alpha already populated
            i += 1


class SerialReader(object):
    """
    from Miniterm Terminal application.
    Copy blksize data bytes from serial port to Queue q
    """

    def __init__(self, serial_instance, q, blksize):
        self.serial = serial_instance
        self.alive = None
        self._reader_alive = None
        self.receiver_thread = None
        self.q = q
        self.blksize = blksize

    def _start_reader(self):
        """Start reader thread"""
        self._reader_alive = True
        self.receiver_thread = threading.Thread(target=self.reader, name='rx')
        self.receiver_thread.daemon = True
        self.receiver_thread.start()

    def _stop_reader(self):
        """Stop reader thread only, wait for clean exit of thread"""
        self._reader_alive = False
        self.receiver_thread.join()

    def start(self):
        """start worker threads"""
        self.alive = True
        self._start_reader()

    def stop(self):
        """set flag to stop worker threads"""
        self.alive = False
        self._stop_reader()

    def join(self, transmit_only=False):
        """wait for worker threads to terminate"""
        self.receiver_thread.join()

    def close(self):
        self.serial.close()

    def reader(self):
        """loop and queue serial data frames"""
        try:
            while self.alive and self._reader_alive:
                data = self.serial.read(self.blksize)
                self.q.put(data)

        except serial.SerialException:
            self.alive = False
            raise  # XXX handle instead of re-raise

    ##===============================================


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Missing config.yaml")
        exit(-2)
    print("pyserial ver:", serial.__version__)
    print("pyYAML ver:", yaml.__version__)
    pygame.init()
    strip = LEDStrip(sys.argv[1])
    strip.led_show()
    ser = serial_init(strip.serial_port, strip.timing_mod)
    if ser is None:
        exit(-1)  # many reasons to not connect
    print("Using COM Port:", ser.name)

    dataQueue = queue.Queue(1024)  # input data Q, in frames. Rather large in case you're moving the window.
    sr = SerialReader(ser, dataQueue, strip.frame_size)
    # initial sync, if this fails press then 's' ...
    sr.start()
    serial_sync(ser)  # resync and flush

    maxqsz = 0
    frames_displayed = 0
    frames_read = 0
    t = time.perf_counter()
    fps_t = time.perf_counter()

    while True:
        if dataQueue.qsize() > 0:
            frame_data = dataQueue.get()
            frames_read += 1
            # rate limit display
            fps_et = time.perf_counter() - fps_t
            if fps_et >= strip.display_rate_limit:
                fps_t = time.perf_counter()
                strip.led_display_format(frame_data)
                strip.led_show()
                frames_displayed += 1
            if dataQueue.qsize() > maxqsz:
                maxqsz = dataQueue.qsize()
        else:
            time.sleep(0.0001)  # let input stream dictate FPS, but need a small sleep to keep CPU% lower

        # Cycles through all occurring events
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == MOUSEBUTTONDOWN:  # show some stats
                et = time.perf_counter() - t
                print("maxQ=", maxqsz, ", fps displayed=", int(frames_displayed / et), ", displayed rate b/s", int(frames_displayed / et) * strip.frame_size, ", input rate b/s=",
                      int(frames_read / et) * strip.frame_size)
                t = time.perf_counter()
                frames_displayed = 0
                frames_read = 0
                maxqsz = 0
            elif event.type == KEYDOWN:  # change gamma
                if event.key == K_UP:
                    strip.gamma -= 0.01
                elif event.key == K_DOWN:
                    strip.gamma += 0.01
                elif event.key == K_s:
                    print("Sync")
                    sr.stop()
                    serial_sync(ser)  # try to resync, flush
                    sr.start()

                if strip.gamma > 1:
                    strip.gamma = 1
                elif strip.gamma < 0:
                    strip.gamma = 0.1
                print('{:.2f}'.format(strip.gamma))
