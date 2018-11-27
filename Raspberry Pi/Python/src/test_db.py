import sepyrebase
import time

ADDR_NODO_1 = 0x6001

print("Inicio Test...")

def f11(mensaje):
    print("Event: ",end="")
    print(mensaje["event"])
    print("Path: ",end="")
    print(mensaje["path"])
    print("Data: ",end="")
    print(mensaje["data"])
    print("Funcion f11 stream callback: ", end="")
    # print(type(data))

sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_1)),"Actuador1",f11)

while True:
    try:
        print("Espeando evento...")
        time.sleep(1)
    except KeyboardInterrupt:
        break

print("Finalizando Test...")

sepyrebase.close_db()