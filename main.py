import time
import config
import network
import ubinascii
import json

from machine import Pin, UART
from picographics import PicoGraphics, DISPLAY_ENVIRO_PLUS
from pimoroni import RGBLED, Button
from breakout_bme68x import BreakoutBME68X, STATUS_HEATER_STABLE
from pimoroni_i2c import PimoroniI2C
from breakout_ltr559 import BreakoutLTR559
from adcfft import ADCFFT
# comment out the next line if no particulate sensor
from pms5003 import PMS5003

from umqtt.robust import MQTTClient
from umqtt.simple import MQTTException

"""
This example reads from all the sensors on Enviro+.
(plus the optional particulate sensor)
Displays the results on screen.
Press A and B to turn the backlight on and off
Press X and Y to switch between sensor mode and graphic equaliser mode!
"""

# change this to adjust temperature compensation
TEMPERATURE_OFFSET = 3

BRIGHTNESS = 0.8

# change this to adjust pressure based on your altitude
altitude = 0

# light the LED red if the gas reading is less than 50%
GAS_ALERT = 0.5


def graphic_equaliser():
    m_arr = [0 for _ in range(16)]
    i = 0

    adcfft.update()
    m = 0
    for x in range(5, 240):
        v = adcfft.get_scaled(x, 144)
        m = max(m, v)
        v = min(239, v)
        v = 239 - v
        display.line(x - 5, v, x - 5, 239)
    m_arr[i] = min(255, m)
    i += 1
    if i >= len(m_arr):
        i = 0
    ms = int(sum(m_arr) / len(m_arr))
    led.set_rgb(0, ms, 0)


def adjust_to_sea_pressure(pressure_hpa, temperature, altitude):
    """
    Adjust pressure based on your altitude.

    credits to @cubapp https://gist.github.com/cubapp/23dd4e91814a995b8ff06f406679abcf
    """

    # Adjusted-to-the-sea barometric pressure
    adjusted_hpa = pressure_hpa + ((pressure_hpa * 9.80665 * altitude) / (287 * (273 + temperature + (altitude / 400))))
    return adjusted_hpa


def describe_pressure(pressure_hpa):
    """Convert pressure into barometer-type description."""
    pressure_hpa += 0.5
    if pressure_hpa < 970:
        description = "storm"
    elif 970 <= pressure_hpa < 990:
        description = "rain"
    elif 990 <= pressure_hpa < 1010:
        description = "change"
    elif 1010 <= pressure_hpa < 1030:
        description = "fair"
    elif pressure_hpa >= 1030:
        description = "dry"
    else:
        description = ""
    return description


def describe_humidity(corrected_humidity):
    """Convert relative humidity into good/bad description."""
    corrected_humidity += 0.5
    if 40 < corrected_humidity < 60:
        description = "good"
    else:
        description = "bad"
    return description


def describe_light(lux):
    """Convert light level in lux to descriptive value."""
    lux += 0.5
    if lux < 50:
        description = "dark"
    elif 50 <= lux < 100:
        description = "dim"
    elif 100 <= lux < 500:
        description = "light"
    elif lux >= 500:
        description = "bright"
    return description

def describe_pm10(pm10):
    """Convert PM10 level into a descriptive value,

    see https://uk-air.defra.gov.uk/air-pollution/daqi?view=more-info&pollutant=pm10#pollutant"""
    if(pm10) <= 35:
        return "Low"
    elif pm10 <= 53:
        return "Moderate"
    elif pm10 <= 70:
        return "High"
    
    return "Very High"

def describe_pm2_5(pm2_5):
    """Convert PM2.5 level into a descriptive value,

    see https://uk-air.defra.gov.uk/air-pollution/daqi?view=more-info&pollutant=pm10#pollutant"""
    if(pm10) <= 50:
        return "Low"
    elif pm10 <= 75:
        return "Moderate"
    elif pm10 <= 100:
        return "High"
    
    return "Very High"


def draw_hist(results_array):
    result_index = 0
    for result in results_array:
        display.set_pen(YELLOW)
        display.rectangle(2 * result_index, 240 - (result.pm_ug_per_m3(10) * 2), 2, 240)
        display.set_pen(MAGENTA)
        display.rectangle(2 * result_index, 240 - (result.pm_ug_per_m3(2.5) * 2), 2, 240)
        display.set_pen(BLUE)
        display.rectangle(2 * result_index, 240 - (result.pm_ug_per_m3(1.0) * 2), 2, 240)
        result_index += 1

def draw_connection_status(conn):
    display.set_pen(GREEN)
    
    if not conn.isconnected():
        display.set_pen(RED)
        
    display.circle(200, 10, 4)
    
    if not mqtt_connected:
        display.set_pen(RED)

    display.circle(210, 10, 4)

def ha_config():
    config = json.dumps({ "name": "PM1", "device_class": "pm1", "stat_t": f"homeassistant/sensor/enviroplus_{unique_id}/state", "value_template": "{{ value_json.pm1 }}", "unique_id": f"enviroplus_{unique_id}_pm1", "unit_of_measurement": "µg/m³","device": { "identifiers": f"enviroplus_{unique_id}", "name": "Workshop"}})

    mqtt.publish(f"homeassistant/sensor/enviroplus_{unique_id}_pm1/config", config)
#    mqtt.publish(f"homeassistant/sensor/enviroplus_{unique_id}_pm25/config", f'{{ "name": "PM2.5",  "device_class": "pm25",  "state_topic": "homeassistant/sensor/enviroplus_{unique_id}/state",  "value_template": "{{{{ value_json.pm25 }}}}",  "unique_id": "enviroplus_{unique_id}_pm25", "unit_of_measurement": "µg/m³",  "device": {{"identifiers": ["enviroplus_{unique_id}"], "name": "Workshop"}}}} }}')
#    mqtt.publish(f"homeassistant/sensor/enviroplus_{unique_id}_pm10/config", f'{{ "name":  "PM10",  "device_class": "pm10",  "state_topic": "homeassistant/sensor/enviroplus_{unique_id}/state",  "value_template": "{{{{ value_json.pm10 }}}}",  "unique_id": "enviroplus_{unique_id}_pm10", "unit_of_measurement": "µg/m³",  "device": {{"identifiers": ["enviroplus_{unique_id}"], "name": "Workshop"}}}} }}')
    
#    mqtt.publish("homeassistant/sensor/enviro_plus_T/config", "")
#    mqtt.publish("homeassistant/sensor/enviro_plus_H/config", "")
    
#    mqtt.publish("homeassistant/sensor/enviro_plus_gas/config", "")
    
#    mqtt.publish("homeassistant/sensor/enviro_plus_lux/config", "")
#    mqtt.publish("homeassistant/sensor/enviro_plus_hPa/config", "")

def ha_update():
    mqtt.publish(f"homeassistant/sensor/enviroplus_{unique_id}/state", f'{{ "pm1": {pm1}, "pm25": {pm2_5}, "pm10": {pm10} }}')
    
# Connect to the wifi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(config.SSID, config.PASSWORD)
unique_id = ubinascii.hexlify(wlan.config('mac')).decode().upper()


# Connect to the MQTT server.
mqtt_connected = False;
try:
    mqtt = MQTTClient("enviro_plus", config.MQTT_SERVER, config.MQTT_PORT, config.MQTT_USER, config.MQTT_PASSWORD)
    mqtt.connect()
    mqtt_connected = True;
    
    ha_config()
except MQTTException as e:
    print(f"Unable to connect to MQTT: {e}")


# set up the display
display = PicoGraphics(display=DISPLAY_ENVIRO_PLUS)

# set up the LED
led = RGBLED(6, 7, 10, invert=True)

# set up the buttons
button_a = Button(12, invert=True)
button_b = Button(13, invert=True)
button_x = Button(14, invert=True)
button_y = Button(15, invert=True)

# set up the Pico W's I2C
PINS_BREAKOUT_GARDEN = {"sda": 4, "scl": 5}
i2c = PimoroniI2C(**PINS_BREAKOUT_GARDEN)

# set up BME688 and LTR559 sensors
bme = BreakoutBME68X(i2c, address=0x77)
ltr = BreakoutLTR559(i2c)

# set up ADCFFT library to read mic with fast fourier transform
adcfft = ADCFFT()

# configure the PMS5003 for Enviro+
# comment out this section if no particulate sensor
pms5003 = PMS5003(
    uart=UART(1, tx=Pin(8), rx=Pin(9), baudrate=9600),
    pin_enable=Pin(3),
    pin_reset=Pin(2),
    mode="active"
)

# some constants we'll use for drawing
WHITE = display.create_pen(255, 255, 255)
BLACK = display.create_pen(0, 0, 0)
RED = display.create_pen(255, 0, 0)
GREEN = display.create_pen(0, 255, 0)
CYAN = display.create_pen(0, 255, 255)
MAGENTA = display.create_pen(200, 0, 200)
YELLOW = display.create_pen(200, 200, 0)
BLUE = display.create_pen(0, 0, 200)
FFT_COLOUR = display.create_pen(255, 0, 255)
GREY = display.create_pen(75, 75, 75)

WIDTH, HEIGHT = display.get_bounds()
display.set_font("sans")

# some other variables we'll use to keep track of stuff

mode = "sensors"  # start off in sensor mode

# these values will get updated later on
min_temperature = 100.0
max_temperature = 0.0
min_gas = 100000.0
max_gas = 0.0

# array for storing particulate readings
results = []

# setup
led.set_rgb(255, 0, 0)
display.set_backlight(BRIGHTNESS)
display.set_pen(RED)
display.text("waiting for sensors", 0, 0, WIDTH, scale=0.7)
display.update()

# the gas sensor gives a few weird readings to start, lets discard them
temperature, pressure, humidity, gas, status, _, _ = bme.read()
time.sleep(0.5)
temperature, pressure, humidity, gas, status, _, _ = bme.read()
time.sleep(0.5)

while True:
    # turn on the backlight with A and turn it back off with B
    # switch between sensor and equaliser mode with X and Y
    if button_a.is_pressed:
        display.set_backlight(BRIGHTNESS)
        time.sleep(0.2)
    elif button_b.is_pressed:
        display.set_backlight(0)
        time.sleep(0.2)
    elif button_x.is_pressed:
        mode = "sensors"
        display.set_backlight(BRIGHTNESS)
        time.sleep(0.2)
    elif button_y.is_pressed:
        mode = "equaliser"
        display.set_backlight(BRIGHTNESS)
        time.sleep(0.2)

    if mode == "sensors":
        # read BME688
        temperature, pressure, humidity, gas, status, _, _ = bme.read()
        heater = "Stable" if status & STATUS_HEATER_STABLE else "Unstable"

        # correct temperature and humidity using an offset
        corrected_temperature = temperature - TEMPERATURE_OFFSET
        dewpoint = temperature - ((100 - humidity) / 5)
        corrected_humidity = 100 - (5 * (corrected_temperature - dewpoint))

        # record min and max temperatures
        if corrected_temperature >= max_temperature:
            max_temperature = corrected_temperature
        if corrected_temperature <= min_temperature:
            min_temperature = corrected_temperature

        # record min and max gas readings
        if gas > max_gas:
            max_gas = gas
        if gas < min_gas:
            min_gas = gas

        # convert pressure into hpa
        pressure_hpa = pressure / 100

        # correct pressure
        pressure_hpa = adjust_to_sea_pressure(pressure_hpa, corrected_temperature, altitude)

        # read LTR559
        ltr_reading = ltr.get_reading()
        lux = ltr_reading[BreakoutLTR559.LUX]
        prox = ltr_reading[BreakoutLTR559.PROXIMITY]

        # read particulate sensor and put the results into the array
        # comment out if no PM sensor
        data = pms5003.read()
        results.append(data)
        
        if (len(results) > 120):  # Scroll the result list by removing the first value
            results.pop(0)

        if heater == "Stable" and ltr_reading is not None:
            led.set_rgb(0, 0, 0)

            # draw some stuff on the screen
            display.set_pen(BLACK)
            display.clear()
            display.set_thickness(2)
            
            draw_connection_status(wlan)

            # draw particulate graph on screen, comment out if no PM sensor
            draw_hist(results)

            # pick a pen colour based on the temperature
            display.set_pen(GREEN)
            if corrected_temperature > 24:
                display.set_pen(YELLOW)
            if corrected_temperature > 30:
                display.set_pen(RED)
            if corrected_temperature < 10:
                display.set_pen(CYAN)
            display.text(f"{corrected_temperature:.1f}°C", 5, 20, WIDTH, scale=0.8)

            # Particulates
            pm10 = data.pm_ug_per_m3(10)
            pm2_5 = data.pm_ug_per_m3(2.5)
            pm1 = data.pm_ug_per_m3(1.0)
            
            ha_update()

            # draw the first column of text
            display.set_pen(WHITE)
            display.text(f"rh {corrected_humidity:.0f}%", 0, 50, WIDTH, scale=0.8)
            display.text(f"{pressure_hpa:.0f}hPa", 0, 80, WIDTH, scale=0.8)
            display.text(f"{lux:.0f} lux", 0, 110, WIDTH, scale=0.8)
            display.text(f"{pm10:.0f} pm10", 0, 140, WIDTH, scale=0.8)
            display.text(f"{pm2_5:.0f} pm2.5", 0, 170, WIDTH, scale=0.8)
            display.text(f"{pm1:.0f} pm1.0", 0, 200, WIDTH, scale=0.8)

            # draw the second column of text
#            display.text(f"{describe_humidity(corrected_humidity)}", 125, 75, WIDTH, scale=0.8)
 #           display.text(f"{describe_pressure(pressure_hpa)}", 125, 125, WIDTH, scale=0.8)
  #          display.text(f"{describe_light(lux)}", 125, 175, WIDTH, scale=0.8)

            # draw bar for gas
            if min_gas != max_gas:
                # light the LED and set pen to red if the gas / air quality reading is less than 50%
                if (gas - min_gas) / (max_gas - min_gas) < GAS_ALERT:
                    led.set_rgb(255, 0, 0)
                    display.set_pen(RED)
                else:
                    display.set_pen(GREEN)

                display.rectangle(236, HEIGHT - round((gas - min_gas) / (max_gas - min_gas) * HEIGHT), 4, round((gas - min_gas) / (max_gas - min_gas) * HEIGHT))
                display.text("gas", 185, 210, WIDTH, scale=0.8)

            display.update()
            time.sleep(0.5)

    elif mode == "equaliser":
        display.set_pen(BLACK)
        display.clear()
        display.set_pen(FFT_COLOUR)
        display.text("mic", 0, 0, WIDTH, scale=3)
        graphic_equaliser()
        display.update()
