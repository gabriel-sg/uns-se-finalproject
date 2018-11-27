#!/usr/bin/env python

import sys
import os
import threading
import pigpio
import time
import mrf24j40
import sepyrebase
import queue

PANID = 0xcafe
ADDR_NODO_1 = 0x6001
ADDR_NODO_2 = 0x6002
ADDR_NODO_3 = 0x6003

###### Raspberry Pi GPIO ######
pin_reset = 16
pin_cs = 12
pin_interrupt = 20
spi_channel = 0

# Connect to PIGPIO Daemon
pi = pigpio.pi()
if not pi.connected:
    exit()

###### Test Functions ######
run_test = 1

totalRead = 0
totalReadSucc = 0
totalWrite = 0
totalWriteSucc = 0

def testRW(printResults):
    init_time = 0
    contReadReg = 9
    contReadSucc = 0
    if (printResults):
        init_time = int(round(time.time() * 1000))
        print("\nLeyendo " + str(contReadReg) + " registros...\n")
    
    contReadSucc += assertReadShort("MRF_PACON2", mrf24j40.MRF_PACON2, 0b10001000, printResults)
    contReadSucc += assertReadShort("MRF_TXSTBL", mrf24j40.MRF_TXSTBL, 0b01110101, printResults)
    contReadSucc += assertReadShort("MRF_TXBCON1", mrf24j40.MRF_TXBCON1, 0b00110000, printResults)
    contReadSucc += assertReadLong("MRF_WAKETIMEL", mrf24j40.MRF_WAKETIMEL, 0b00001010, printResults)
    contReadSucc += assertReadShort("MRF_BBREG2", mrf24j40.MRF_BBREG2, 0b01001000, printResults)
    contReadSucc += assertReadShort("MRF_BBREG3", mrf24j40.MRF_BBREG3, 0b11011000, printResults)
    contReadSucc += assertReadShort("MRF_BBREG4", mrf24j40.MRF_BBREG4, 0b10011100, printResults)
    contReadSucc += assertReadShort("MRF_BBREG6", mrf24j40.MRF_BBREG6, 0b00000001, printResults)
    contReadSucc += assertReadShort("MRF_INTCON", mrf24j40.MRF_INTCON, 0b11111111, printResults)

    contWriteReg = 11
    contWriteSucc = 0
    if (printResults):
        print("\nEscribiendo " + str(contWriteReg) + " registros...\n")

    contWriteSucc += assertWriteShort("MRF_PACON2", mrf24j40.MRF_PACON2, 0x98, printResults)
    contWriteSucc += assertWriteShort("MRF_TXSTBL", mrf24j40.MRF_TXSTBL, 0x95, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON0", mrf24j40.MRF_RFCON0, 0x03, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON1", mrf24j40.MRF_RFCON1, 0x01, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON2", mrf24j40.MRF_RFCON2, 0x80, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON6", mrf24j40.MRF_RFCON6, 0x90, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON7", mrf24j40.MRF_RFCON7, 0x80, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON8", mrf24j40.MRF_RFCON8, 0x10, printResults)
    contWriteSucc += assertWriteLong("MRF_SLPCON1", mrf24j40.MRF_SLPCON1, 0x21, printResults)
    contWriteSucc += assertWriteShort("MRF_BBREG2", mrf24j40.MRF_BBREG2, 0x80, printResults)
    contWriteSucc += assertWriteShort("MRF_CCAEDTH", mrf24j40.MRF_CCAEDTH, 0x60, printResults)

    if (printResults):
        print("\n------------------Resumen------------------\n")
        print("Lecturas exitosas: " + str(contReadSucc))
        print("Lecturas fallidas: " + str(contReadReg - contReadSucc))
        print("\n")
        print("Escrituras exitosas: " + str(contWriteSucc))
        print("Escrituras fallidas: " + str(contWriteReg - contWriteSucc))
        print("\n")
        init_time = int(round(time.time() * 1000)) - init_time
        print("Test duration: " + str(init_time) + " ms\n")
        print("-------------------------------------------")

    #totalRead += contReadReg
    #totalReadSucc += contReadSucc
    #totalWrite += contWriteReg
    #totalWriteSucc += contWriteSucc

def assertWriteShort(name, address, toWriteValue, printResults):
    mrf.write_short(address, toWriteValue)
    readValue = mrf.read_short(address)
    if (printResults):
        print(str(name) + ": write value: " + str(toWriteValue) + ", read value: " + str(readValue))
    return toWriteValue == readValue

def assertWriteLong(name, address, toWriteValue, printResults):
    mrf.write_long(address, toWriteValue)
    readValue = mrf.read_long(address)
    if (printResults):
        print(str(name) + ": write value: " + str(toWriteValue) + ", read value: " + str(readValue))
    return toWriteValue == readValue

def assertReadShort(name, address, shouldBeValue, printResults):
    readValue = mrf.read_short(address)
    if (printResults):
        print(str(name) + ": read value: " + str(readValue) + ", should be: " + str(shouldBeValue))
    return shouldBeValue == readValue

def assertReadLong(name, address, shouldBeValue, printResults):
    readValue = mrf.read_long(address)
    if (printResults):
        print(str(name) + ": read value: " + str(readValue) + ", should be: " + str(shouldBeValue))
    return shouldBeValue == readValue

############ HANDLERS ############

def handle_rx():
    print("received a packet {} bytes long\n".format(mrf.get_rxinfo().frame_length))
    
    if (mrf.get_bufferPHY()):
        print("Packet data (PHY Payload):\n")
        for i in range(0,mrf.get_rxinfo().frame_length):
            print(mrf.get_rxbuf()[i])
    
    print("\r\nASCII data (relevant data):")
    p_aux = mrf.get_rxinfo().panid
    s_aux = mrf.get_rxinfo().srcaddr
    d_aux = mrf.get_rxinfo().destaddr

    msg_type = mrf.get_rxinfo().rx_data[0]
    if (msg_type == 0):     # ie. tipo sensor -> data
        sens_id = mrf.get_rxinfo().rx_data[1]
        data_type = mrf.get_rxinfo().rx_data[2]
        valor = (mrf.get_rxinfo().rx_data[3] << 8) | mrf.get_rxinfo().rx_data[4]
        m_aux = sepyrebase.Mensaje(s_aux,sens_id,data_type,valor)
        data_queue.put(m_aux)
    
    for i in range(0,mrf.rx_datalength()):
        print(str(mrf.get_rxinfo().rx_data[i]) + "-",end="")
    
    print("\r\nLQI/RSSI=")
    print(mrf.get_rxinfo().lqi)
    print("/")
    print(mrf.get_rxinfo().rssi)

def handle_tx():
    print("handle_tx()" + str(mrf.get_txinfo().tx_ok))
    if (mrf.get_txinfo().tx_ok):
        print("TX went ok, got ack")
    else:
        print("TX failed after {} retries\n".format(mrf.get_txinfo().retries))

def interrupt_routine(gpio,level,tick):
    mrf.interrupt_handler()
    print("Ocurrio una interrupcion")

# def dummy_cb(gpio,level,tick):
#     print("Ocurrio una dummy")

def check_queue_data():

    while True:
        if not (data_queue.empty()):
            m_data = data_queue.get()
            m_data.send_sens_val()
            print("Elimine elemento de la cola")

        time.sleep(1)
        print("Hilo check cola datos")

def check_queue_action():

    while True:
        if not (action_queue.empty()):
            m_data = action_queue.get()
            print("Hilo quitando elemento cola: ",end="")
            print(m_data)
            if (m_data[0] == ADDR_NODO_1):
                if (m_data[2]):
                    mrf.send16(ADDR_NODO_1,str(m_data[1])+"1")
                else:
                    mrf.send16(ADDR_NODO_1,str(m_data[1])+"0")
            elif (m_data[0] == ADDR_NODO_2):
                if (m_data[2]):
                    mrf.send16(ADDR_NODO_2,str(m_data[1])+"1")
                else:
                    mrf.send16(ADDR_NODO_2,str(m_data[1])+"0")
            elif (m_data[0] == ADDR_NODO_3):
                if (m_data[2]):
                    mrf.send16(ADDR_NODO_3,str(m_data[1])+"1")
                else:
                    mrf.send16(ADDR_NODO_3,str(m_data[1])+"0")

        time.sleep(1)

############ Stream DB Functions ############

# NodoId - ActId - Accion

def f11(mensaje):
    data = mensaje["data"]
    arr = [ADDR_NODO_1,1,data]
    action_queue.put(arr)

def f12(mensaje):
    data = mensaje["data"]
    arr = [ADDR_NODO_1,2,data]
    action_queue.put(arr)

def f13(mensaje):
    data = mensaje["data"]
    arr = [ADDR_NODO_1,3,data]
    action_queue.put(arr)

def f21(mensaje):
    data = mensaje["data"]
    arr = [ADDR_NODO_2,1,data]
    action_queue.put(arr)

def f22(mensaje):
    data = mensaje["data"]
    arr = [ADDR_NODO_2,2,data]
    action_queue.put(arr)

def f23(mensaje):
    data = mensaje["data"]
    arr = [ADDR_NODO_2,3,data]
    action_queue.put(arr)

def f31(mensaje):
    data = mensaje["data"]
    arr = [ADDR_NODO_3,1,data]
    action_queue.put(arr)

def f32(mensaje):
    data = mensaje["data"]
    arr = [ADDR_NODO_3,2,data]
    action_queue.put(arr)

def f33(mensaje):
    data = mensaje["data"]
    arr = [ADDR_NODO_3,3,data]
    action_queue.put(arr)

################## MAIN ##################

last_time = 0
tx_interval = 2000
check_queue_interval = 5
data_queue = queue.Queue()
action_queue = queue.Queue()

############ Stream DB ############
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_1)),sepyrebase.ACT1,f11)
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_1)),sepyrebase.ACT2,f12)
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_1)),sepyrebase.ACT3,f13)

sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_2)),sepyrebase.ACT1,f21)
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_2)),sepyrebase.ACT2,f22)
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_2)),sepyrebase.ACT3,f23)

sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_3)),sepyrebase.ACT1,f31)
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_3)),sepyrebase.ACT2,f32)
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_3)),sepyrebase.ACT3,f33)

############ Módulo MRF24J40 ############
# Crea objeto del módulo MRF
mrf = mrf24j40.Mrf24j(pin_reset, pin_cs, pin_interrupt)

if (run_test):
    mrf.reset()
    mrf.init()

    print("\nRealizando test de lectura y escritura...")

    count = 10
    testDuration = int(round(time.time() * 1000))
    for i in range(0,count):
        mrf.reset()
        testRW(1)

    testDuration = int(round(time.time() * 1000)) - testDuration

    print("\n------------------Resumen------------------\n")
    print("Lecturas exitosas: " + str(totalReadSucc))
    print("Lecturas fallidas: " + str(totalRead - totalReadSucc))
    print("\n")
    print("Escrituras exitosas: " + str(totalWriteSucc))
    print("Escrituras fallidas: " + str(totalWrite - totalWriteSucc))
    print("\n")
    print("Test duration: " + str(testDuration) + " ms")
    print("\n")
    print("-------------------------------------------")

mrf.reset()
mrf.init()
mrf.set_channel(18)
mrf.set_pan(PANID)
mrf.address16_write(ADDR_NODO_1)
mrf.set_promiscuous(False)
mrf.set_palna(True)
mrf.set_interrupts()
# mrf.set_bufferPHY(true)
# Set interrupt edge rising. (default = falling)
# mrf.write_long(MRF_SLPCON0, 0b00000010)

###### Attach Interruption Handler ######
cb0 = pi.callback(pin_interrupt,pigpio.EITHER_EDGE,interrupt_routine)
# cb1 = pi.callback(4,pigpio.EITHER_EDGE,dummy_cb)

print("PANId: 0x{:04X} - Addr: 0x{:04X}".format(mrf.get_pan(),mrf.address16_read()))

last_time = int(round(time.time() * 1000))

td = threading.Thread(target=check_queue_data,args="")
td.setDaemon(True)
td.start()

# ta = threading.Thread(target=check_queue_action,args="")
# ta.setDaemon(True)
# ta.start()

############ LOOP ############

while True:
    try:
        current_time = int(round(time.time() * 1000))
        if (current_time - last_time > tx_interval):
            last_time = current_time
            print("rxxxing...")
            # mrf.send16(0x6005, "puto el que lee porque el que lee se la come")
            # time.sleep(1)
    
        time.sleep(1)
        mrf.check_flags(handle_rx, handle_tx)
    except KeyboardInterrupt:
        break

print("\nSaliendo...")

pi.stop()

mrf.close()

cb0.cancel()
# cb1.cancel()

sepyrebase.close_db()

td.join()
# ta.join()

print("Finalizado.")

############ END MAIN ############

# def clear_all():
#     """Clears all the variables from the workspace of the spyder application."""
#     gl = globals().copy()
#     for var in gl:
#         if var[0] == '_': continue
#         if 'func' in str(globals()[var]): continue
#         if 'module' in str(globals()[var]): continue

#         del globals()[var]

# def restart_program():
#     python = sys.executable
#     os.execl(python, python, * sys.argv)

# if __name__ == "__main__":
#     answer = input("Do you want to restart this program ? ")
#     if answer.lower().strip() in "y yes".split():
#         restart_program()
#     else:
#         clear_all()