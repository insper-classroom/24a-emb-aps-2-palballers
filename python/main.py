import serial
import uinput

ser = serial.Serial('/dev/ttyACM0', 115200) # Mude a porta para rfcomm0 se estiver usando bluetooth no linux
# Caso você esteja usando windows você deveria definir uma porta fixa para seu dispositivo (para facilitar sua vida mesmo)
# Siga esse tutorial https://community.element14.com/technologies/internet-of-things/b/blog/posts/standard-serial-over-bluetooth-on-windows-10 e mude o código acima para algo como: ser = serial.Serial('COMX', 9600) (onde X é o número desejado)

# (Mais códigos aqui https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/include/uapi/linux/input-event-codes.h?h=v4.7)
buttons = [
    uinput.REL_X,
    uinput.REL_Y,
    uinput.BTN_LEFT,
    uinput.BTN_RIGHT,
          ]

keyboard_keys = [
    uinput.KEY_Q,
    uinput.KEY_1,
    uinput.KEY_2,
    uinput.KEY_3,
]
total_keys = len(keyboard_keys)
# Criando gamepad emulado
device = uinput.Device(buttons + keyboard_keys)


# Função para analisar os dados recebidos do dispositivo externo
def parse_data(data):
    button = data[0]
    value = int.from_bytes(data[1:3], byteorder='little', signed=True)
    print(f"Received data: {data}")
    print(f"button: {button}, value: {value}")
    return button, value

def emulate_controller(button, value):
    if button < 2:
        device.emit(buttons[button], value)
    elif button < 2 + total_keys:
        device.emit(keyboard_keys[button - 2], value)
        device.emit(keyboard_keys[button - 2], 0)
try:
    # Pacote de sync
    while True:
        print('Waiting for sync package...')
        while True:
            data = ser.read(1)
            if data == b'\xff':
                break

        # Lendo 4 bytes da uart
        data = ser.read(3)
        button, value = parse_data(data)
        emulate_controller(button, value)

except KeyboardInterrupt:
    print("Program terminated by user")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    ser.close()