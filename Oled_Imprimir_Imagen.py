from board import SCL, SDA
import busio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import time

#INICIAR PANTALLA
i2c = busio.I2C(SCL, SDA)
oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c) #Crear la clase oled y definir limites de la pantalla
oled.fill(0)#Limpiar
oled.show()#Pantalla

# Alternatively load a different format image, resize it, and convert to 1 bit color.
image = Image.open('ALLAH.jpg').resize((oled.width, oled.height), Image.ANTIALIAS).convert('1')
# Mostrar
oled.image(image)
oled.show()