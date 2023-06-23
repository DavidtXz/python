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


image = Image.new('1', (128, 64))
draw = ImageDraw.Draw(image)
#font = ImageFont.load_default()
font = ImageFont.truetype('PixelOperator.ttf', 16)
        
#  Escribe 2 lineas texto
draw.text((15, 15),    'David Monroy',  font=font, fill=255)
draw.text((0, 30), 'Oled-Raspberry Pi4', font=font, fill=255)

# Muestra Texto
oled.image(image)
oled.show()
time.sleep(5)
oled.fill(0)
oled.show()