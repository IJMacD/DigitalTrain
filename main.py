import conf

# Device properties
speed = 0
target_speed = 0
acceleration = 10
freq = 20
debug = True
light = False
light_brightness = 1000

# Running vars
led_status = False
connected = False

def do_connect():
	global name
	import network
	ap = network.WLAN(network.AP_IF)
	ap.active(False)
	wlan = network.WLAN(network.STA_IF)
	wlan.active(True)
	if not wlan.isconnected():
		print('connecting to network...')
		wlan.connect(conf.WLAN_SSID, conf.WLAN_PASS)
		while not wlan.isconnected():
			pass
	print('network config:', wlan.ifconfig())

def get_name ():
	import machine
	uid = machine.unique_id()
	return "".join("%02X" % uid[2-i] for i in range(3))

def led (value):
	from machine import Pin
	p = Pin(conf.LED_PIN, Pin.OUT)
	p.value(not value)

def setLight (value):
	from machine import Pin, PWM
	global light
	light = value
	pin = Pin(conf.LIGHT_PIN, Pin.OUT)
	
	if conf.LIGHT_PIN == 16:
		# PWM not available on GPIO16
		pin.value(not value if conf.LIGHT_INV else value)
		return
	
	pwm = PWM(pin)
	if light_brightness == 1000:
		pwm.deinit()
		pin.value(not value if conf.LIGHT_INV else value)
	elif value:
		pwm.duty(light_brightness)
	else:
		pwm.deinit()
		pin.value(0)

def stop ():
	from machine import Pin, PWM
	global speed
	global target_speed
	speed = 0
	target_speed = 0
	pin = Pin(conf.MOTOR_PIN, Pin.OUT)
	pwm = PWM(pin)
	pwm.deinit()
	pin.value(0)
	print("STOP")

def setSpeed (value):
	from machine import Pin, PWM
	global speed
	speed = value
	pin = Pin(conf.MOTOR_PIN, Pin.OUT)
	pwm = PWM(pin, freq=freq)
	if value == 0:
		pwm.deinit()
		pin.value(0)
	else:
		pwm.duty(abs(value))
	# print("PWM on pin %d (duty: %d, freq: %d)" % (MOTOR_PIN, pwm.duty(), pwm.freq()))

def setTargetSpeed (value):
	global target_speed
	target_speed = value

def setPWMFreq (value):
	from machine import Pin, PWM
	global freq
	freq = value
	pin = Pin(conf.MOTOR_PIN, Pin.OUT)
	pwm = PWM(pin, freq=value)
	# print("PWM on pin %d (duty: %d, freq: %d)" % (MOTOR_PIN, pwm.duty(), pwm.freq()))

do_connect()

def flip_led ():
	global led_status
	led_status = not led_status
	led(debug and led_status)
	
def control_loop (t):
	if speed > target_speed:
		setSpeed(max(speed - acceleration, target_speed))
	elif speed < target_speed:
		setSpeed(min(speed + acceleration, target_speed))

# Speed Control Loop
from machine import Timer
tmr = Timer(-1)
tmr.init(period=100, mode=Timer.PERIODIC, callback=control_loop)

import usocket
import utime
last_time=0
# Main loop to reconnect from scratch
while True:
	sock = usocket.socket()
	try:
		addr_info = usocket.getaddrinfo(conf.SERVER_NAME, conf.SERVER_PORT)
		# print("Server addresses:")
		# for ai in addr_info:
			# print(ai[-1])
		addr = addr_info[0][-1]
		print("%d Connecting %s" % (utime.ticks_ms(), addr))
		sock.connect(addr)
		sock.write("id,%s" % get_name())
		# set timeout here as we only care about timeout for reading
		sock.settimeout(2) # in seconds
		connected = True
		last_time = utime.ticks_ms()
		print("%d Connected" % last_time)
		
		# Loop to allow for missed packets without reconnecting
		while True:
			try:

				# Normal readline loop
				while True:
					line = str(sock.readline(), 'utf-8')[:-1]
					last_time = utime.ticks_ms()
					# print("%d Packet Recieved" % last_time)
					flip_led()
					
					if line == "stop":
						stop()
						continue
						
					if line == "?":
						sock.write("speed,%d" % speed)
						continue
					
					parts = line.split(",")
					i = 0
					
					while i + 1 < len(parts):
						name = parts[i]
						value = parts[i+1]
						
						if name == "light":
							setLight(value == "1")
							sock.write("light,%s" % ("on" if light else "off"))
						elif name == "speed":
							s = int(value)
							setTargetSpeed(s)
							setSpeed(s)
						elif name == "target_speed":
							setTargetSpeed(int(value))
						elif name == "freq":
							setPWMFreq(int(value))
						elif name == "debug":
							debug = value == "1"
						elif name == "light_brightness":
							light_brightness = int(value)
							setLight(light)
						
						i = i + 2
						
			except OSError as e:
				import uerrno
				
				emsg = ""
				if e.args[0] == uerrno.ETIMEDOUT:
					emsg = "Connection timed out"
					
					t = utime.ticks_ms()
					delta = t-last_time
					print("%d %s Delta: %d" % (t, emsg, delta))
					# We missed a packet so we need to stop
					stop()
					
					if delta > 10000:
						# if we haven't heard anything for 10 seconds then reconnect
						raise OSError(uerrno.ETIMEDOUT)
						
	except OSError as e:
		import uerrno
		
		emsg = ""
		if e.args[0] == uerrno.ETIMEDOUT:
			emsg = "Connection timed out too many times"
		elif e.args[0] == uerrno.EINPROGRESS:
			emsg = "Error: Still Connecting"
		elif e.args[0] == uerrno.ECONNRESET:
			emsg = "Connection Reset"
		elif e.args[0] == 103:
			emsg = "Connection Aborted"
		elif e.args[0] == 113:
			emsg = "Host Unreachable"
		else:
			emsg = "Connection error %d" % e.args[0]
			
		t = utime.ticks_ms()
		delta = t-last_time
		print("%d %s Delta: %d" % (t, emsg, delta))
		
		sock.close()
		connected = False
		utime.sleep(1)